/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*      Title     : servo.cpp
 *      Project   : moveit_servo
 *      Created   : 05/17/2023
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson, V Mohammed Ibrahim
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <moveit_servo/utils/command.hpp>

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.servo");
constexpr double ROBOT_STATE_WAIT_TIME = 5.0;  // seconds
}  // namespace

namespace moveit_servo
{

Servo::Servo(const rclcpp::Node::SharedPtr& node, std::shared_ptr<const servo::ParamListener> servo_param_listener,
             const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
  : node_(node)
  , servo_param_listener_{ servo_param_listener }
  , planning_scene_monitor_{ planning_scene_monitor }
  , transform_buffer_(node_->get_clock())
  , transform_listener_(transform_buffer_)
{
  servo_params_ = servo_param_listener_->get_params();

  validateParams(servo_params_);

  if (!planning_scene_monitor_->getStateMonitor()->waitForCompleteState(servo_params_.move_group_name,
                                                                        ROBOT_STATE_WAIT_TIME))
  {
    RCLCPP_ERROR(LOGGER, "Timeout waiting for current state");
    std::exit(EXIT_FAILURE);
  }

  // Planning scene monitor is passed in.
  if (servo_params_.is_primary_planning_scene_monitor)
  {
    planning_scene_monitor_->providePlanningSceneService();
  }
  else
  {
    planning_scene_monitor_->requestPlanningSceneState();
  }

  // Get the robot state and joint model group info.
  robot_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  joint_model_group_ = robot_state_->getJointModelGroup(servo_params_.move_group_name);

  if (joint_model_group_ == nullptr)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Invalid move group name: `" << servo_params_.move_group_name << '`');
    throw std::runtime_error("Invalid move group name");
  }
  else
  {
    // Get necessary information about joints
    joint_names_ = joint_model_group_->getActiveJointModelNames();
    joint_bounds_ = joint_model_group_->getActiveJointModelsBounds();
    num_joints_ = joint_names_.size();
  }

  checkIKSolver();

  // Check if the transforms to planning frame and end effector frame exist.
  if (!robot_state_->knowsFrameTransform(servo_params_.planning_frame))
  {
    servo_status_ = StatusCode::INVALID;
    RCLCPP_ERROR_STREAM(LOGGER, "No transform available for planning frame " << servo_params_.planning_frame);
  }
  else if (!robot_state_->knowsFrameTransform(servo_params_.ee_frame))
  {
    servo_status_ = StatusCode::INVALID;
    RCLCPP_ERROR_STREAM(LOGGER, "No transform available for end effector frame " << servo_params_.ee_frame);
  }
  else
  {
    // Load the smoothing plugin
    if (servo_params_.use_smoothing)
    {
      setSmoothingPlugin();
    }
    else
    {
      RCLCPP_WARN(LOGGER, "No smoothing plugin loaded");
    }

    // Create the collision checker and start collision checking.
    collision_monitor_ =
        std::make_unique<CollisionMonitor>(planning_scene_monitor_, servo_params_, collision_velocity_scale_);
    collision_monitor_->start();

    servo_status_ = StatusCode::NO_WARNING;
    RCLCPP_INFO_STREAM(LOGGER, "Servo initialized successfully");
  }
}

Servo::~Servo()
{
  setCollisionChecking(false);
}

KinematicState Servo::getNextJointState(const ServoInput& command)
{
  // Update the parameters
  if (servo_params_.enable_parameter_update)
  {
    updateParams();
  }

  // Set status to clear
  servo_status_ = StatusCode::NO_WARNING;

  // State variables
  KinematicState current_state(num_joints_), target_state(num_joints_);
  target_state.joint_names = joint_names_;
  // Copy current kinematic data from RobotState.
  robot_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  robot_state_->copyJointGroupPositions(joint_model_group_, current_state.positions);
  robot_state_->copyJointGroupVelocities(joint_model_group_, current_state.velocities);
  // Create Eigen maps for cleaner operations.
  Eigen::Map<Eigen::VectorXd> current_joint_positions(current_state.positions.data(), num_joints_);
  Eigen::Map<Eigen::VectorXd> target_joint_positions(target_state.positions.data(), num_joints_);
  Eigen::Map<Eigen::VectorXd> current_joint_velocities(current_state.velocities.data(), num_joints_);
  Eigen::Map<Eigen::VectorXd> target_joint_velocities(target_state.velocities.data(), num_joints_);

  // Compute the change in joint position due to the incoming command
  Eigen::VectorXd joint_position_delta = jointDeltaFromCommand(command);

  // Continue rest of the computations only if the command was valid
  if (servo_status_ != StatusCode::INVALID)
  {
    // Apply collision scaling to the joint position delta
    if (collision_velocity_scale_ > 0 && collision_velocity_scale_ < 1)
    {
      servo_status_ = StatusCode::DECELERATE_FOR_COLLISION;
    }
    else if (collision_velocity_scale_ == 0)
    {
      servo_status_ = StatusCode::HALT_FOR_COLLISION;
    }
    joint_position_delta *= collision_velocity_scale_;

    // Compute the next joint positions based on the joint position deltas
    target_joint_positions = current_joint_positions + joint_position_delta;

    // TODO : apply filtering to the velocity instead of position
    // Apply smoothing to the positions
    // Update filter state
    if (smoother_)
    {
      smoother_->reset(current_state.positions);
      smoother_->doSmoothing(target_state.positions);
    }

    // Compute velocities based on smoothed joint positions
    target_joint_velocities = (target_joint_positions - current_joint_positions) / servo_params_.publish_period;

    // TODO : print warning if scaling applied for joint limit.
    // Scale down the velocity based on joint velocity limit or user defined scaling if applicable.
    target_joint_velocities *= jointLimitVelocityScalingFactor(target_joint_velocities, joint_bounds_,
                                                               servo_params_.override_velocity_scaling_factor);

    // Adjust joint position based on scaled down velocity
    target_joint_positions = current_joint_positions + (target_joint_velocities * servo_params_.publish_period);

    // Check if any joints are going past joint position limits
    std::vector<int> joints_to_halt =
        jointsToHalt(target_joint_positions, target_joint_velocities, joint_bounds_, servo_params_.joint_limit_margin);

    // Apply halting if any joints need to be halted.
    if (!joints_to_halt.empty())
    {
      servo_status_ = StatusCode::JOINT_BOUND;
      target_state = haltJoints(joints_to_halt, current_state, target_state);
    }
  }

  return target_state;
}

Eigen::VectorXd Servo::jointDeltaFromCommand(const ServoInput& command)
{
  Eigen::VectorXd joint_position_deltas(num_joints_);
  joint_position_deltas.setZero();

  JointDeltaResult delta_result;

  const CommandType expected_type = expectedCommandType();
  if (command.index() == static_cast<size_t>(expected_type))
  {
    if (expected_type == CommandType::JOINT_JOG)
    {
      delta_result = jointDeltaFromJointJog(std::get<JointJog>(command), robot_state_, servo_params_);
      servo_status_ = delta_result.first;
    }
    else if (expected_type == CommandType::TWIST)
    {
      delta_result = jointDeltaFromTwist(std::get<Twist>(command), robot_state_, servo_params_);
      servo_status_ = delta_result.first;
    }
    else if (expected_type == CommandType::POSE)
    {
      delta_result = jointDeltaFromPose(std::get<Pose>(command), robot_state_, servo_params_);
      servo_status_ = delta_result.first;
    }
    if (servo_status_ != StatusCode::INVALID)
    {
      joint_position_deltas = delta_result.second;
    }
  }
  else
  {
    servo_status_ = StatusCode::INVALID;
    RCLCPP_WARN_STREAM(LOGGER, "SERVO : Incoming command type does not match expected command type.");
  }

  return joint_position_deltas;
}

KinematicState Servo::haltJoints(const std::vector<int>& joints_to_halt, const KinematicState& current_state,
                                 const KinematicState& target_state)
{
  KinematicState bounded_state(num_joints_);
  bounded_state.joint_names = target_state.joint_names;

  std::stringstream halting_joint_names;
  for (const int idx : joints_to_halt)
  {
    halting_joint_names << joint_names_[idx] + " ";
  }
  RCLCPP_WARN_STREAM(LOGGER, "Joint position limit reached on joints: " << halting_joint_names.str());

  const bool all_joint_halt =
      (expectedCommandType() == CommandType::JOINT_JOG && servo_params_.halt_all_joints_in_joint_mode) ||
      (expectedCommandType() == CommandType::TWIST && servo_params_.halt_all_joints_in_cartesian_mode);

  if (all_joint_halt)
  {
    // The velocities are initialized to zero by default, so we dont need to set it here.
    bounded_state.positions = current_state.positions;
  }
  else
  {
    // Halt only the joints that are out of bounds
    bounded_state.positions = target_state.positions;
    bounded_state.velocities = target_state.velocities;
    for (const int idx : joints_to_halt)
    {
      bounded_state.positions[idx] = current_state.positions[idx];
      bounded_state.velocities[idx] = 0.0;
    }
  }

  return bounded_state;
}

void Servo::setSmoothingPlugin()
{
  // Load the smoothing plugin
  try
  {
    pluginlib::ClassLoader<online_signal_smoothing::SmoothingBaseClass> smoothing_loader(
        "moveit_core", "online_signal_smoothing::SmoothingBaseClass");
    smoother_ = smoothing_loader.createUniqueInstance(servo_params_.smoothing_filter_plugin_name);
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(LOGGER, "Exception while loading the smoothing plugin '%s': '%s'",
                 servo_params_.smoothing_filter_plugin_name.c_str(), ex.what());
    std::exit(EXIT_FAILURE);
  }

  // Initialize the smoothing plugin
  if (!smoother_->initialize(node_, planning_scene_monitor_->getRobotModel(), num_joints_))
  {
    RCLCPP_ERROR(LOGGER, "Smoothing plugin could not be initialized");
    std::exit(EXIT_FAILURE);
  }
}

void Servo::checkIKSolver()
{
  // Get the IK solver for the group
  kinematics::KinematicsBaseConstPtr ik_solver = joint_model_group_->getSolverInstance();
  if (!ik_solver)
  {
    RCLCPP_WARN(
        LOGGER,
        "No kinematics solver instantiated for group '%s'. Will use inverse Jacobian for servo calculations instead.",
        joint_model_group_->getName().c_str());
  }
  else if (!ik_solver->supportsGroup(joint_model_group_))
  {
    RCLCPP_WARN(LOGGER,
                "The loaded kinematics plugin does not support group '%s'. Will use inverse Jacobian for servo "
                "calculations instead.",
                joint_model_group_->getName().c_str());
  }
  else
  {
    RCLCPP_INFO(LOGGER, "IK solver available for robot, will use it.");
  }
}

void Servo::updateParams()
{
  if (servo_param_listener_->is_old(servo_params_))
  {
    auto params = servo_param_listener_->get_params();
    if (params.override_velocity_scaling_factor != servo_params_.override_velocity_scaling_factor)
    {
      RCLCPP_INFO_STREAM(LOGGER, "override_velocity_scaling_factor changed to : "
                                     << std::to_string(params.override_velocity_scaling_factor));
    }

    servo_params_ = params;
  }
}

StatusCode Servo::getStatus()
{
  return servo_status_;
}

const std::string Servo::getStatusMessage()
{
  return SERVO_STATUS_CODE_MAP.at(servo_status_);
}

CommandType Servo::expectedCommandType()
{
  return expected_command_type_;
}

void Servo::expectedCommandType(const CommandType& command_type)
{
  expected_command_type_ = command_type;
}

void Servo::validateParams(const servo::Params& servo_params)
{
  bool has_error = false;
  if (servo_params.hard_stop_singularity_threshold <= servo_params.lower_singularity_threshold)
  {
    RCLCPP_ERROR(LOGGER, "Parameter 'hard_stop_singularity_threshold' "
                         "should be greater than 'lower_singularity_threshold.' "
                         "Check the parameters YAML file used to launch this node.");
    has_error = true;
  }

  if (!servo_params.publish_joint_positions && !servo_params.publish_joint_velocities &&
      !servo_params.publish_joint_accelerations)
  {
    RCLCPP_ERROR(LOGGER, "At least one of publish_joint_positions / "
                         "publish_joint_velocities / "
                         "publish_joint_accelerations must be true. "
                         "Check the parameters YAML file used to launch this node.");
    has_error = true;
  }

  if ((servo_params.command_out_type == "std_msgs/Float64MultiArray") && servo_params.publish_joint_positions &&
      servo_params.publish_joint_velocities)
  {
    RCLCPP_ERROR(LOGGER, "When publishing a std_msgs/Float64MultiArray, "
                         "you must select positions OR velocities."
                         "Check the parameters YAML file used to launch this node.");
    has_error = true;
  }

  if (servo_params.scene_collision_proximity_threshold < servo_params.self_collision_proximity_threshold)
  {
    RCLCPP_ERROR(LOGGER, "Parameter 'self_collision_proximity_threshold' should probably be less "
                         "than or equal to 'scene_collision_proximity_threshold'."
                         "Check the parameters YAML file used to launch this node.");
    has_error = true;
  }

  if (has_error)
  {
    throw std::runtime_error("Servo failed to initialize : Invalid parameter values");
  }
}

const Eigen::Isometry3d Servo::getEndEffectorPose()
{
  // Robot base (panda_link0) to end effector frame (panda_link8)
  robot_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  return robot_state_->getGlobalLinkTransform(servo_params_.ee_frame);
}

Pose Servo::toPlanningFrame(const Pose& command)
{
  if (command.frame_id != servo_params_.planning_frame)
  {
    auto target_pose = convertIsometryToTransform(command.pose, servo_params_.planning_frame, command.frame_id);
    auto command_to_planning_frame =
        transform_buffer_.lookupTransform(servo_params_.planning_frame, command.frame_id, rclcpp::Time(0));
    tf2::doTransform(target_pose, target_pose, command_to_planning_frame);
    return Pose{ servo_params_.planning_frame, tf2::transformToEigen(target_pose) };
  }
  else
  {
    return command;
  }
}

Twist Servo::toPlanningFrame(const Twist& command)
{
  Twist transformed_twist = command;

  if (command.frame_id != servo_params_.planning_frame)
  {
    const bool can_transform = robot_state_->knowsFrameTransform(servo_params_.planning_frame);
    if (can_transform)
    {
      // TODO : Find out why transform obtained from tf2 does not work properly.
      // RobotState::GetGlobalLinkTransform does not work for all frames in the environment.
      // The behaviour is same if robot_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState()
      // is called before computing the transform.
      const Eigen::Isometry3d planning_frame_transform =
          robot_state_->getGlobalLinkTransform(servo_params_.planning_frame).inverse() *
          robot_state_->getGlobalLinkTransform(command.frame_id);
      // Apply the transformation to the command vector
      transformed_twist.frame_id = servo_params_.planning_frame;
      transformed_twist.velocities.head<3>() = planning_frame_transform.linear() * command.velocities.head<3>();
      transformed_twist.velocities.tail<3>() = planning_frame_transform.linear() * command.velocities.tail<3>();
    }
    else
    {
      servo_status_ = StatusCode::INVALID;
      RCLCPP_ERROR_STREAM(LOGGER,
                          "Cannot transform from " << command.frame_id << " to " << servo_params_.planning_frame);
    }
  }
  return transformed_twist;
}

void Servo::setCollisionChecking(const bool check_collision)
{
  check_collision ? collision_monitor_->start() : collision_monitor_->stop();
}

}  // namespace moveit_servo
