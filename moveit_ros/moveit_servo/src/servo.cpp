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
#include <moveit/utils/logger.hpp>

// Disable -Wold-style-cast because all _THROTTLE macros trigger this
#pragma GCC diagnostic ignored "-Wold-style-cast"

namespace
{
constexpr double ROBOT_STATE_WAIT_TIME = 5.0;  // seconds
constexpr double STOPPED_VELOCITY_EPS = 1e-4;
}  // namespace

namespace moveit_servo
{

Servo::Servo(const rclcpp::Node::SharedPtr& node, std::shared_ptr<const servo::ParamListener> servo_param_listener,
             const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
  : node_(node)
  , logger_(moveit::getLogger("servo"))
  , servo_param_listener_{ std::move(servo_param_listener) }
  , planning_scene_monitor_{ planning_scene_monitor }
{
  servo_params_ = servo_param_listener_->get_params();

  if (!validateParams(servo_params_))
  {
    RCLCPP_ERROR_STREAM(logger_, "Got invalid parameters, exiting.");
    std::exit(EXIT_FAILURE);
  }

  if (!planning_scene_monitor_->getStateMonitor()->waitForCompleteState(servo_params_.move_group_name,
                                                                        ROBOT_STATE_WAIT_TIME))
  {
    RCLCPP_ERROR(logger_, "Timeout waiting for current state");
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

  // Load the smoothing plugin
  if (servo_params_.use_smoothing)
  {
    setSmoothingPlugin();
  }
  else
  {
    RCLCPP_WARN(logger_, "No smoothing plugin loaded");
  }

  // Create the collision checker and start collision checking.
  collision_monitor_ =
      std::make_unique<CollisionMonitor>(planning_scene_monitor_, servo_params_, std::ref(collision_velocity_scale_));
  collision_monitor_->start();

  servo_status_ = StatusCode::NO_WARNING;

  const auto& move_group_joint_names = planning_scene_monitor_->getRobotModel()
                                           ->getJointModelGroup(servo_params_.move_group_name)
                                           ->getActiveJointModelNames();
  // Create subgroup map
  for (const auto& sub_group_name : planning_scene_monitor_->getRobotModel()->getJointModelGroupNames())
  {
    // Skip move group
    if (sub_group_name == servo_params_.move_group_name)
    {
      continue;
    }
    const auto& subgroup_joint_names =
        planning_scene_monitor_->getRobotModel()->getJointModelGroup(sub_group_name)->getActiveJointModelNames();

    JointNameToMoveGroupIndexMap new_map;
    // For each joint name of the subgroup calculate the index in the move group joint vector
    for (const auto& joint_name : subgroup_joint_names)
    {
      // Find subgroup joint name in move group joint names
      const auto move_group_iterator =
          std::find(move_group_joint_names.cbegin(), move_group_joint_names.cend(), joint_name);
      // Calculate position and add a new mapping of joint name to move group joint vector position
      new_map.insert(std::make_pair<std::string, std::size_t>(
          std::string(joint_name), std::distance(move_group_joint_names.cbegin(), move_group_iterator)));
    }
    // Add new joint name to index map to existing maps
    joint_name_to_index_maps_.insert(
        std::make_pair<std::string, JointNameToMoveGroupIndexMap>(std::string(sub_group_name), std::move(new_map)));
  }
  RCLCPP_INFO_STREAM(logger_, "Servo initialized successfully");
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
    RCLCPP_ERROR(logger_, "Exception while loading the smoothing plugin '%s': '%s'",
                 servo_params_.smoothing_filter_plugin_name.c_str(), ex.what());
    std::exit(EXIT_FAILURE);
  }

  // Initialize the smoothing plugin
  moveit::core::RobotStatePtr robot_state = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  const int num_joints =
      robot_state->getJointModelGroup(servo_params_.move_group_name)->getActiveJointModelNames().size();
  if (!smoother_->initialize(node_, planning_scene_monitor_->getRobotModel(), num_joints))
  {
    RCLCPP_ERROR(logger_, "Smoothing plugin could not be initialized");
    std::exit(EXIT_FAILURE);
  }
  resetSmoothing(getCurrentRobotState());
}

void Servo::doSmoothing(KinematicState& state)
{
  if (smoother_)
  {
    smoother_->doSmoothing(state.positions, state.velocities, state.accelerations);
  }
}

void Servo::resetSmoothing(const KinematicState& state)
{
  if (smoother_)
  {
    smoother_->reset(state.positions, state.velocities, state.accelerations);
  }
}

void Servo::setCollisionChecking(const bool check_collision)
{
  check_collision ? collision_monitor_->start() : collision_monitor_->stop();
}

bool Servo::validateParams(const servo::Params& servo_params) const
{
  bool params_valid = true;
  auto robot_state = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  auto joint_model_group = robot_state->getJointModelGroup(servo_params.move_group_name);
  if (joint_model_group == nullptr)
  {
    RCLCPP_ERROR_STREAM(logger_, "Invalid move group name: `" << servo_params.move_group_name << '`');
    params_valid = false;
  }

  if (servo_params.hard_stop_singularity_threshold <= servo_params.lower_singularity_threshold)
  {
    RCLCPP_ERROR(logger_, "Parameter 'hard_stop_singularity_threshold' "
                          "should be greater than 'lower_singularity_threshold.' "
                          "Check the parameters YAML file used to launch this node.");
    params_valid = false;
  }

  if (!servo_params.publish_joint_positions && !servo_params.publish_joint_velocities &&
      !servo_params.publish_joint_accelerations)
  {
    RCLCPP_ERROR(logger_, "At least one of publish_joint_positions / "
                          "publish_joint_velocities / "
                          "publish_joint_accelerations must be true. "
                          "Check the parameters YAML file used to launch this node.");
    params_valid = false;
  }

  if ((servo_params.command_out_type == "std_msgs/Float64MultiArray") && servo_params.publish_joint_positions &&
      servo_params.publish_joint_velocities)
  {
    RCLCPP_ERROR(logger_, "When publishing a std_msgs/Float64MultiArray, "
                          "you must select positions OR velocities."
                          "Check the parameters YAML file used to launch this node.");
    params_valid = false;
  }

  if (servo_params.scene_collision_proximity_threshold < servo_params.self_collision_proximity_threshold)
  {
    RCLCPP_ERROR(logger_, "Parameter 'self_collision_proximity_threshold' should probably be less "
                          "than or equal to 'scene_collision_proximity_threshold'."
                          "Check the parameters YAML file used to launch this node.");
    params_valid = false;
  }

  if (!servo_params.active_subgroup.empty() && servo_params.active_subgroup != servo_params.move_group_name &&
      !joint_model_group->isSubgroup(servo_params.active_subgroup))
  {
    RCLCPP_ERROR(logger_,
                 "The value '%s' Parameter 'active_subgroup' does not name a valid subgroup of joint group '%s'.",
                 servo_params.active_subgroup.c_str(), servo_params.move_group_name.c_str());
    params_valid = false;
  }

  return params_valid;
}

bool Servo::updateParams()
{
  bool params_updated = false;
  if (servo_param_listener_->is_old(servo_params_))
  {
    const auto params = servo_param_listener_->get_params();

    if (validateParams(params))
    {
      if (params.override_velocity_scaling_factor != servo_params_.override_velocity_scaling_factor)
      {
        RCLCPP_INFO_STREAM(logger_, "override_velocity_scaling_factor changed to : "
                                        << std::to_string(params.override_velocity_scaling_factor));
      }

      servo_params_ = params;
      params_updated = true;
    }
    else
    {
      RCLCPP_WARN_STREAM(logger_, "Parameters will not be updated.");
    }
  }
  return params_updated;
}

servo::Params& Servo::getParams()
{
  return servo_params_;
}

StatusCode Servo::getStatus() const
{
  return servo_status_;
}

std::string Servo::getStatusMessage() const
{
  return SERVO_STATUS_CODE_MAP.at(servo_status_);
}

CommandType Servo::getCommandType() const
{
  return expected_command_type_;
}

void Servo::setCommandType(const CommandType& command_type)
{
  expected_command_type_ = command_type;
}

KinematicState Servo::haltJoints(const std::vector<int>& joints_to_halt, const KinematicState& current_state,
                                 const KinematicState& target_state) const
{
  KinematicState bounded_state(target_state.joint_names.size());
  bounded_state.joint_names = target_state.joint_names;

  std::stringstream halting_joint_names;
  for (const int idx : joints_to_halt)
  {
    halting_joint_names << bounded_state.joint_names[idx] + " ";
  }
  RCLCPP_WARN_STREAM(logger_, "Joint position limit reached on joints: " << halting_joint_names.str());

  const bool all_joint_halt =
      (getCommandType() == CommandType::JOINT_JOG && servo_params_.halt_all_joints_in_joint_mode) ||
      (getCommandType() == CommandType::TWIST && servo_params_.halt_all_joints_in_cartesian_mode);

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

Eigen::VectorXd Servo::jointDeltaFromCommand(const ServoInput& command, const moveit::core::RobotStatePtr& robot_state)
{
  // Determine joint_name_group_index_map, if no subgroup is active, the map is empty
  const auto& active_subgroup_name =
      servo_params_.active_subgroup.empty() ? servo_params_.move_group_name : servo_params_.active_subgroup;
  const auto& joint_name_group_index_map = (active_subgroup_name != servo_params_.move_group_name) ?
                                               joint_name_to_index_maps_.at(servo_params_.active_subgroup) :
                                               JointNameToMoveGroupIndexMap();

  const int num_joints =
      robot_state->getJointModelGroup(servo_params_.move_group_name)->getActiveJointModelNames().size();
  Eigen::VectorXd joint_position_deltas(num_joints);
  joint_position_deltas.setZero();

  JointDeltaResult delta_result;

  const CommandType expected_type = getCommandType();
  if (command.index() == static_cast<size_t>(expected_type))
  {
    if (expected_type == CommandType::JOINT_JOG)
    {
      delta_result = jointDeltaFromJointJog(std::get<JointJogCommand>(command), robot_state, servo_params_,
                                            joint_name_group_index_map);
      servo_status_ = delta_result.first;
    }
    else if (expected_type == CommandType::TWIST)
    {
      // Transform the twist command to the planning frame, which is the base frame of the active subgroup's IK solver,
      // before applying it. Additionally verify there is an IK solver, and that the transformation is successful.
      const auto planning_frame_maybe = getIKSolverBaseFrame(robot_state, active_subgroup_name);
      if (planning_frame_maybe.has_value())
      {
        const auto& planning_frame = *planning_frame_maybe;
        const auto command_in_planning_frame_maybe = toPlanningFrame(std::get<TwistCommand>(command), planning_frame);
        if (command_in_planning_frame_maybe.has_value())
        {
          delta_result = jointDeltaFromTwist(*command_in_planning_frame_maybe, robot_state, servo_params_,
                                             planning_frame, joint_name_group_index_map);
          servo_status_ = delta_result.first;
        }
        else
        {
          servo_status_ = StatusCode::INVALID;
          RCLCPP_ERROR_STREAM(logger_, "Could not transform twist command to planning frame.");
        }
      }
      else
      {
        servo_status_ = StatusCode::INVALID;
        RCLCPP_ERROR(logger_, "No IK solver for planning group %s.", active_subgroup_name.c_str());
      }
    }
    else if (expected_type == CommandType::POSE)
    {
      // Transform the twist command to the planning frame, which is the base frame of the active subgroup's IK solver,
      // before applying it. The end effector frame is also extracted as the tip frame of the IK solver.
      // Additionally verify there is an IK solver, and that the transformation is successful.
      const auto planning_frame_maybe = getIKSolverBaseFrame(robot_state, active_subgroup_name);
      const auto ee_frame_maybe = getIKSolverTipFrame(robot_state, active_subgroup_name);
      if (planning_frame_maybe.has_value() && ee_frame_maybe.has_value())
      {
        const auto& planning_frame = *planning_frame_maybe;
        const auto command_in_planning_frame_maybe = toPlanningFrame(std::get<PoseCommand>(command), planning_frame);
        if (command_in_planning_frame_maybe.has_value())
        {
          delta_result = jointDeltaFromPose(*command_in_planning_frame_maybe, robot_state, servo_params_,
                                            planning_frame, *ee_frame_maybe, joint_name_group_index_map);
          servo_status_ = delta_result.first;
        }
        else
        {
          servo_status_ = StatusCode::INVALID;
          RCLCPP_ERROR_STREAM(logger_, "Could not transform pose command to planning frame.");
        }
      }
      else
      {
        servo_status_ = StatusCode::INVALID;
        RCLCPP_ERROR(logger_, "No IK solver for planning group %s.", active_subgroup_name.c_str());
      }
    }

    if (servo_status_ != StatusCode::INVALID)
    {
      joint_position_deltas = delta_result.second;
    }
  }
  else
  {
    servo_status_ = StatusCode::INVALID;
    RCLCPP_WARN_STREAM(logger_, "Incoming servo command type does not match known command types.");
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
  const moveit::core::JointBoundsVector joint_bounds = joint_model_group->getActiveJointModelsBounds();
  const int num_joints = joint_names.size();

  // State variables
  KinematicState current_state(num_joints), target_state(num_joints);
  target_state.joint_names = joint_names;

  // Copy current kinematic data from RobotState.
  robot_state->copyJointGroupPositions(joint_model_group, current_state.positions);
  robot_state->copyJointGroupVelocities(joint_model_group, current_state.velocities);

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
  if (servo_status_ != StatusCode::INVALID && servo_status_ != StatusCode::HALT_FOR_COLLISION)
  {
    // Apply collision scaling to the joint position delta
    joint_position_delta *= collision_velocity_scale_;

    // Compute the next joint positions based on the joint position deltas
    target_state.positions = current_state.positions + joint_position_delta;

    // Apply smoothing to the positions if a smoother was provided.
    doSmoothing(target_state);

    // Compute velocities based on smoothed joint positions
    target_state.velocities = (target_state.positions - current_state.positions) / servo_params_.publish_period;

    // Scale down the velocity based on joint velocity limit or user defined scaling if applicable.
    const double joint_limit_scale = jointLimitVelocityScalingFactor(target_state.velocities, joint_bounds,
                                                                     servo_params_.override_velocity_scaling_factor);
    if (joint_limit_scale < 1.0)  // 1.0 means no scaling.
    {
      RCLCPP_DEBUG_STREAM(logger_, "Joint velocity limit scaling applied by a factor of " << joint_limit_scale);
    }

    target_state.velocities *= joint_limit_scale;

    // Adjust joint position based on scaled down velocity
    target_state.positions = current_state.positions + (target_state.velocities * servo_params_.publish_period);

    // Check if any joints are going past joint position limits
    const std::vector<int> joints_to_halt =
        jointsToHalt(target_state.positions, target_state.velocities, joint_bounds, servo_params_.joint_limit_margin);

    // Apply halting if any joints need to be halted.
    if (!joints_to_halt.empty())
    {
      servo_status_ = StatusCode::JOINT_BOUND;
      target_state = haltJoints(joints_to_halt, current_state, target_state);
    }
  }

  return target_state;
}

std::optional<Eigen::Isometry3d> Servo::getPlanningToCommandFrameTransform(const std::string& command_frame,
                                                                           const std::string& planning_frame) const
{
  const moveit::core::RobotStatePtr robot_state = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  if (robot_state->knowsFrameTransform(command_frame) && (robot_state->knowsFrameTransform(planning_frame)))
  {
    return robot_state->getGlobalLinkTransform(planning_frame).inverse() *
           robot_state->getGlobalLinkTransform(command_frame);
  }
  else
  {
    try
    {
      return tf2::transformToEigen(
          planning_scene_monitor_->getTFClient()->lookupTransform(planning_frame, command_frame, rclcpp::Time(0)));
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_ERROR(logger_, "Failed to get planning to command frame transform: %s", ex.what());
      return std::nullopt;
    }
  }
}

std::optional<TwistCommand> Servo::toPlanningFrame(const TwistCommand& command, const std::string& planning_frame) const
{
  Eigen::VectorXd transformed_twist = command.velocities;

  if (command.frame_id != planning_frame)
  {
    // Look up the transform between the planning and command frames.
    const auto planning_to_command_tf_maybe = getPlanningToCommandFrameTransform(command.frame_id, planning_frame);
    if (!planning_to_command_tf_maybe.has_value())
    {
      return std::nullopt;
    }
    const auto& planning_to_command_tf = *planning_to_command_tf_maybe;

    if (servo_params_.apply_twist_commands_about_ee_frame)
    {
      // If the twist command is applied about the end effector frame, simply apply the rotation of the transform.
      const auto planning_to_command_rotation = planning_to_command_tf.linear();
      const Eigen::Vector3d translation_vector =
          planning_to_command_rotation *
          Eigen::Vector3d(command.velocities[0], command.velocities[1], command.velocities[2]);
      const Eigen::Vector3d angular_vector =
          planning_to_command_rotation *
          Eigen::Vector3d(command.velocities[3], command.velocities[4], command.velocities[5]);

      // Update the values of the original command message to reflect the change in frame
      transformed_twist.head<3>() = translation_vector;
      transformed_twist.tail<3>() = angular_vector;
    }
    else
    {
      // If the twist command is applied about the planning frame, the spatial twist is calculated
      // as shown in Equation 3.83 in http://hades.mech.northwestern.edu/images/7/7f/MR.pdf.
      // The above equation defines twist as [angular; linear], but in our convention it is
      // [linear; angular] so the adjoint matrix is also reordered accordingly.
      Eigen::MatrixXd adjoint(6, 6);

      const Eigen::Matrix3d& rotation = planning_to_command_tf.rotation();
      const Eigen::Vector3d& translation = planning_to_command_tf.translation();

      Eigen::Matrix3d skew_translation;
      skew_translation.row(0) << 0, -translation(2), translation(1);
      skew_translation.row(1) << translation(2), 0, -translation(0);
      skew_translation.row(2) << -translation(1), translation(0), 0;

      adjoint.topLeftCorner(3, 3) = skew_translation * rotation;
      adjoint.topRightCorner(3, 3) = rotation;
      adjoint.bottomLeftCorner(3, 3) = rotation;
      adjoint.bottomRightCorner(3, 3).setZero();

      transformed_twist = adjoint * transformed_twist;
    }
  }

  return TwistCommand{ planning_frame, transformed_twist };
}

std::optional<PoseCommand> Servo::toPlanningFrame(const PoseCommand& command, const std::string& planning_frame) const
{
  const auto planning_to_command_tf_maybe = getPlanningToCommandFrameTransform(command.frame_id, planning_frame);
  if (!planning_to_command_tf_maybe)
  {
    return std::nullopt;
  }

  const auto& planning_to_command_tf = *planning_to_command_tf_maybe;
  return PoseCommand{ planning_frame, planning_to_command_tf * command.pose };
}

KinematicState Servo::getCurrentRobotState() const
{
  moveit::core::RobotStatePtr robot_state = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  const moveit::core::JointModelGroup* joint_model_group =
      robot_state->getJointModelGroup(servo_params_.move_group_name);
  const auto joint_names = joint_model_group->getActiveJointModelNames();

  KinematicState current_state(joint_names.size());
  current_state.joint_names = joint_names;
  robot_state->copyJointGroupPositions(joint_model_group, current_state.positions);
  robot_state->copyJointGroupVelocities(joint_model_group, current_state.velocities);
  robot_state->copyJointGroupAccelerations(joint_model_group, current_state.accelerations);

  return current_state;
}

std::pair<bool, KinematicState> Servo::smoothHalt(const KinematicState& halt_state)
{
  bool stopped = false;
  auto target_state = halt_state;
  const KinematicState current_state = getCurrentRobotState();

  const size_t num_joints = current_state.joint_names.size();
  for (size_t i = 0; i < num_joints; ++i)
  {
    const double vel = (target_state.positions[i] - current_state.positions[i]) / servo_params_.publish_period;
    target_state.velocities[i] = (vel > STOPPED_VELOCITY_EPS) ? vel : 0.0;
    target_state.accelerations[i] =
        (target_state.velocities[i] - current_state.velocities[i]) / servo_params_.publish_period;
  }

  doSmoothing(target_state);

  // If all velocities are near zero, robot has decelerated to a stop.
  stopped =
      (std::accumulate(target_state.velocities.begin(), target_state.velocities.end(), 0.0) <= STOPPED_VELOCITY_EPS);

  return std::make_pair(stopped, target_state);
}

}  // namespace moveit_servo
