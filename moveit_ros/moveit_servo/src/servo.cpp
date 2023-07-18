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

#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/command.hpp>
#include <moveit_servo/utils/common.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.servo");
constexpr double ROBOT_STATE_WAIT_TIME = 5.0;  // seconds
}  // namespace

namespace moveit_servo
{

Servo::Servo(const rclcpp::Node::SharedPtr& node, std::shared_ptr<const servo::ParamListener> servo_param_listener,
             const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
  : node_(node), servo_param_listener_{ servo_param_listener }, planning_scene_monitor_{ planning_scene_monitor }
{
  servo_params_ = servo_param_listener_->get_params();

  const bool params_valid = validateParams(servo_params_);
  if (!params_valid)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Got invalid parameters, exiting.");
    std::exit(EXIT_FAILURE);
  }

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

  moveit::core::RobotStatePtr robot_state = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  // Check if the transforms to planning frame and end effector frame exist.
  if (!robot_state->knowsFrameTransform(servo_params_.planning_frame))
  {
    servo_status_ = StatusCode::INVALID;
    RCLCPP_ERROR_STREAM(LOGGER, "No transform available for planning frame " << servo_params_.planning_frame);
  }
  else if (!robot_state->knowsFrameTransform(servo_params_.ee_frame))
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
        std::make_unique<CollisionMonitor>(planning_scene_monitor_, servo_params_, std::ref(collision_velocity_scale_));
    collision_monitor_->start();

    servo_status_ = StatusCode::NO_WARNING;
    RCLCPP_INFO_STREAM(LOGGER, "Servo initialized successfully");
  }
}

Servo::~Servo()
{
  setCollisionChecking(false);
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
  moveit::core::RobotStatePtr robot_state = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  const int num_joints =
      robot_state->getJointModelGroup(servo_params_.move_group_name)->getActiveJointModelNames().size();
  if (!smoother_->initialize(node_, planning_scene_monitor_->getRobotModel(), num_joints))
  {
    RCLCPP_ERROR(LOGGER, "Smoothing plugin could not be initialized");
    std::exit(EXIT_FAILURE);
  }
}

void Servo::setCollisionChecking(const bool check_collision)
{
  check_collision ? collision_monitor_->start() : collision_monitor_->stop();
}

bool Servo::validateParams(const servo::Params& servo_params)
{
  bool params_valid = true;

  auto robot_state = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  auto joint_model_group = robot_state->getJointModelGroup(servo_params.move_group_name);
  if (joint_model_group == nullptr)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Invalid move group name: `" << servo_params.move_group_name << '`');
    params_valid = false;
  }

  if (servo_params.hard_stop_singularity_threshold <= servo_params.lower_singularity_threshold)
  {
    RCLCPP_ERROR(LOGGER, "Parameter 'hard_stop_singularity_threshold' "
                         "should be greater than 'lower_singularity_threshold.' "
                         "Check the parameters YAML file used to launch this node.");
    params_valid = false;
  }

  if (!servo_params.publish_joint_positions && !servo_params.publish_joint_velocities &&
      !servo_params.publish_joint_accelerations)
  {
    RCLCPP_ERROR(LOGGER, "At least one of publish_joint_positions / "
                         "publish_joint_velocities / "
                         "publish_joint_accelerations must be true. "
                         "Check the parameters YAML file used to launch this node.");
    params_valid = false;
  }

  if ((servo_params.command_out_type == "std_msgs/Float64MultiArray") && servo_params.publish_joint_positions &&
      servo_params.publish_joint_velocities)
  {
    RCLCPP_ERROR(LOGGER, "When publishing a std_msgs/Float64MultiArray, "
                         "you must select positions OR velocities."
                         "Check the parameters YAML file used to launch this node.");
    params_valid = false;
  }

  if (servo_params.scene_collision_proximity_threshold < servo_params.self_collision_proximity_threshold)
  {
    RCLCPP_ERROR(LOGGER, "Parameter 'self_collision_proximity_threshold' should probably be less "
                         "than or equal to 'scene_collision_proximity_threshold'."
                         "Check the parameters YAML file used to launch this node.");
    params_valid = false;
  }

  return params_valid;
}

bool Servo::updateParams()
{
  bool params_updated = false;

  if (servo_param_listener_->is_old(servo_params_))
  {
    auto params = servo_param_listener_->get_params();

    const bool params_valid = validateParams(params);
    if (params_valid)
    {
      if (params.override_velocity_scaling_factor != servo_params_.override_velocity_scaling_factor)
      {
        RCLCPP_INFO_STREAM(LOGGER, "override_velocity_scaling_factor changed to : "
                                       << std::to_string(params.override_velocity_scaling_factor));
      }
      if (params.move_group_name != servo_params_.move_group_name)
      {
        RCLCPP_INFO_STREAM(LOGGER, "Move group changed from " << servo_params_.move_group_name << " to "
                                                              << params.move_group_name);
      }

      servo_params_ = params;
      params_updated = true;
    }
    else
    {
      RCLCPP_WARN_STREAM(LOGGER, "Parameters will not be updated.");
    }
  }
  return params_updated;
}

servo::Params& Servo::getParams()
{
  return servo_params_;
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

const Eigen::Isometry3d Servo::getEndEffectorPose()
{
  // Robot base (panda_link0) to end effector frame (panda_link8)
  return planning_scene_monitor_->getStateMonitor()->getCurrentState()->getGlobalLinkTransform(servo_params_.ee_frame);
}

KinematicState Servo::haltJoints(const std::vector<int>& joints_to_halt, const KinematicState& current_state,
                                 const KinematicState& target_state)
{
  KinematicState bounded_state(target_state.joint_names.size());
  bounded_state.joint_names = target_state.joint_names;

  std::stringstream halting_joint_names;
  for (const int idx : joints_to_halt)
  {
    halting_joint_names << bounded_state.joint_names[idx] + " ";
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

Eigen::VectorXd Servo::jointDeltaFromCommand(const ServoInput& command, moveit::core::RobotStatePtr& robot_state)
{
  const int num_joints =
      robot_state->getJointModelGroup(servo_params_.move_group_name)->getActiveJointModelNames().size();
  Eigen::VectorXd joint_position_deltas(num_joints);
  joint_position_deltas.setZero();

  JointDeltaResult delta_result;

  const CommandType expected_type = expectedCommandType();
  if (command.index() == static_cast<size_t>(expected_type))
  {
    if (expected_type == CommandType::JOINT_JOG)
    {
      delta_result = jointDeltaFromJointJog(std::get<JointJogCommand>(command), robot_state, servo_params_);
      servo_status_ = delta_result.first;
    }
    else if (expected_type == CommandType::TWIST)
    {
      delta_result = jointDeltaFromTwist(std::get<TwistCommand>(command), robot_state, servo_params_);
      servo_status_ = delta_result.first;
    }
    else if (expected_type == CommandType::POSE)
    {
      delta_result = jointDeltaFromPose(std::get<PoseCommand>(command), robot_state, servo_params_);
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

KinematicState Servo::getNextJointState(const ServoInput& command)
{
  // Set status to clear
  servo_status_ = StatusCode::NO_WARNING;

  // Update the parameters
  updateParams();

  // Get the robot state and joint model group info.
  moveit::core::RobotStatePtr robot_state = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  const moveit::core::JointModelGroup* joint_model_group =
      robot_state->getJointModelGroup(servo_params_.move_group_name);

  // Get necessary information about joints
  const std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();
  moveit::core::JointBoundsVector joint_bounds = joint_model_group->getActiveJointModelsBounds();
  const int num_joints = joint_names.size();

  // State variables
  KinematicState current_state(num_joints), target_state(num_joints);
  target_state.joint_names = joint_names;

  // Copy current kinematic data from RobotState.
  robot_state->copyJointGroupPositions(joint_model_group, current_state.positions);
  robot_state->copyJointGroupVelocities(joint_model_group, current_state.velocities);

  // Create Eigen maps for cleaner operations.
  Eigen::Map<Eigen::VectorXd> current_joint_positions(current_state.positions.data(), num_joints);
  Eigen::Map<Eigen::VectorXd> target_joint_positions(target_state.positions.data(), num_joints);
  Eigen::Map<Eigen::VectorXd> current_joint_velocities(current_state.velocities.data(), num_joints);
  Eigen::Map<Eigen::VectorXd> target_joint_velocities(target_state.velocities.data(), num_joints);

  // Compute the change in joint position due to the incoming command
  Eigen::VectorXd joint_position_delta = jointDeltaFromCommand(command, robot_state);

  if (collision_velocity_scale_ > 0 && collision_velocity_scale_ < 1)
  {
    servo_status_ = StatusCode::DECELERATE_FOR_COLLISION;
  }
  else if (collision_velocity_scale_ == 0)
  {
    servo_status_ = StatusCode::HALT_FOR_COLLISION;
  }

  // Continue rest of the computations only if the command is valid
  // The computations can be skipped also in case we are halting.
  if (servo_status_ != StatusCode::INVALID || servo_status_ != StatusCode::HALT_FOR_COLLISION)
  {
    // Apply collision scaling to the joint position delta
    joint_position_delta *= collision_velocity_scale_;

    // Compute the next joint positions based on the joint position deltas
    target_joint_positions = current_joint_positions + joint_position_delta;

    // TODO : apply filtering to the velocity instead of position
    // Apply smoothing to the positions if a smoother was provided.
    // Update filter state and apply filtering in position domain
    if (smoother_)
    {
      smoother_->reset(current_state.positions);
      smoother_->doSmoothing(target_state.positions);
    }

    // Compute velocities based on smoothed joint positions
    target_joint_velocities = (target_joint_positions - current_joint_positions) / servo_params_.publish_period;

    // TODO : print warning if scaling applied for joint limit.
    // Scale down the velocity based on joint velocity limit or user defined scaling if applicable.
    target_joint_velocities *= jointLimitVelocityScalingFactor(target_joint_velocities, joint_bounds,
                                                               servo_params_.override_velocity_scaling_factor);

    // Adjust joint position based on scaled down velocity
    target_joint_positions = current_joint_positions + (target_joint_velocities * servo_params_.publish_period);

    // Check if any joints are going past joint position limits
    std::vector<int> joints_to_halt =
        jointsToHalt(target_joint_positions, target_joint_velocities, joint_bounds, servo_params_.joint_limit_margin);

    // Apply halting if any joints need to be halted.
    if (!joints_to_halt.empty())
    {
      servo_status_ = StatusCode::JOINT_BOUND;
      target_state = haltJoints(joints_to_halt, current_state, target_state);
    }
  }

  return target_state;
}

}  // namespace moveit_servo
