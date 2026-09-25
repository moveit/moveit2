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
 *      Created   : 3/9/2017
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 */

#include <moveit_servo/make_shared_from_pool.h>
#include <moveit_servo/servo.h>

namespace moveit_servo
{
namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.servo");
constexpr double ROBOT_STATE_WAIT_TIME = 10.0;  // seconds
}  // namespace

Servo::Servo(const rclcpp::Node::SharedPtr& node, const ServoParameters::SharedConstPtr& parameters,
             const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
  : planning_scene_monitor_{ planning_scene_monitor }
  , parameters_{ parameters }
  , servo_calcs_{ node, parameters, planning_scene_monitor_ }
  , collision_checker_{ node, parameters, planning_scene_monitor_ }
{
}

void Servo::start()
{
  if (!planning_scene_monitor_->getStateMonitor()->waitForCompleteState(parameters_->move_group_name,
                                                                        ROBOT_STATE_WAIT_TIME))
  {
    RCLCPP_ERROR(LOGGER, "Timeout waiting for current state");
    return;
  }

  setPaused(false);

  // Crunch the numbers in this timer
  servo_calcs_.start();

  // Check collisions in this timer
  if (parameters_->check_collisions)
    collision_checker_.start();
}

Servo::~Servo()
{
  setPaused(true);
}

void Servo::setPaused(bool paused)
{
  servo_calcs_.setPaused(paused);
  collision_checker_.setPaused(paused);
}

bool Servo::getCommandFrameTransform(Eigen::Isometry3d& transform)
{
  return servo_calcs_.getCommandFrameTransform(transform);
}

bool Servo::getCommandFrameTransform(geometry_msgs::msg::TransformStamped& transform)
{
  return servo_calcs_.getCommandFrameTransform(transform);
}

bool Servo::getEEFrameTransform(Eigen::Isometry3d& transform)
{
  return servo_calcs_.getEEFrameTransform(transform);
}

bool Servo::getEEFrameTransform(geometry_msgs::msg::TransformStamped& transform)
{
  return servo_calcs_.getEEFrameTransform(transform);
}

const ServoParameters::SharedConstPtr& Servo::getParameters() const
{
<<<<<<< HEAD
  return parameters_;
=======
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

KinematicState Servo::haltJoints(const std::vector<size_t>& joint_variables_to_halt,
                                 const KinematicState& current_state, const KinematicState& target_state) const
{
  KinematicState bounded_state(target_state.joint_names.size());
  bounded_state.joint_names = target_state.joint_names;

  std::stringstream halting_joint_names;
  for (const auto idx : joint_variables_to_halt)
  {
    halting_joint_names << bounded_state.joint_names[idx] + " ";
  }
  RCLCPP_WARN_STREAM(logger_, "Joint position limit reached on joints: " << halting_joint_names.str());

  const bool all_joint_halt =
      (getCommandType() == CommandType::JOINT_JOG && servo_params_.halt_all_joints_in_joint_mode) ||
      ((getCommandType() == CommandType::TWIST || getCommandType() == CommandType::POSE) &&
       servo_params_.halt_all_joints_in_cartesian_mode);

  if (all_joint_halt)
  {
    // The velocities are initialized to zero by default, so we don't need to set it here.
    bounded_state.positions = current_state.positions;
  }
  else
  {
    // Halt only the joints that are out of bounds
    bounded_state.positions = target_state.positions;
    bounded_state.velocities = target_state.velocities;
    for (const auto idx : joint_variables_to_halt)
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
      // Transform the pose command to the planning frame, which is the base frame of the active subgroup's IK solver,
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

KinematicState Servo::getNextJointState(const moveit::core::RobotStatePtr& robot_state, const ServoInput& command)
{
  // Set status to clear
  servo_status_ = StatusCode::NO_WARNING;

  // Update the parameters
  updateParams();

  // Get the joint model group info.
  const moveit::core::JointModelGroup* joint_model_group =
      robot_state->getJointModelGroup(servo_params_.move_group_name);

  // Get necessary information about joints
  const std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();
  const moveit::core::JointBoundsVector joint_bounds = joint_model_group->getActiveJointModelsBounds();
  const int num_joints = joint_names.size();

  // Extract current state from robot state
  KinematicState current_state = extractRobotState(robot_state, servo_params_.move_group_name);
  KinematicState target_state(num_joints);
  target_state.joint_names = joint_names;

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
    // Compute the next joint positions based on the joint position deltas
    target_state.positions = current_state.positions + joint_position_delta;

    // Compute the joint velocities required to reach positions
    target_state.velocities = joint_position_delta / servo_params_.publish_period;

    // Scale down the velocity based on joint velocity limit or user defined scaling if applicable.
    const double joint_velocity_limit_scale = jointLimitVelocityScalingFactor(
        target_state.velocities, joint_bounds, servo_params_.override_velocity_scaling_factor);
    if (joint_velocity_limit_scale < 1.0)  // 1.0 means no scaling.
    {
      RCLCPP_DEBUG_STREAM(logger_, "Joint velocity limit scaling applied by a factor of " << joint_velocity_limit_scale);
    }
    target_state.velocities *= joint_velocity_limit_scale;

    // Adjust joint position based on scaled down velocity
    target_state.positions = current_state.positions + (target_state.velocities * servo_params_.publish_period);

    // Apply collision scaling to the joint position delta
    target_state.positions =
        current_state.positions + collision_velocity_scale_ * (target_state.positions - current_state.positions);

    // Compute velocities based on smoothed joint positions
    target_state.velocities = (target_state.positions - current_state.positions) / servo_params_.publish_period;

    // Check if any joints are going past joint position limits.
    const std::vector<size_t> joint_variables_to_halt =
        jointVariablesToHalt(target_state.positions, target_state.velocities, joint_bounds, joint_limit_margins_);

    // Apply halting if any joints need to be halted.
    if (!joint_variables_to_halt.empty())
    {
      servo_status_ = StatusCode::JOINT_BOUND;
      target_state = haltJoints(joint_variables_to_halt, current_state, target_state);
    }
  }

  // Apply smoothing to the positions if a smoother was provided.
  doSmoothing(target_state);

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

KinematicState Servo::getCurrentRobotState(bool block_for_current_state) const
{
  if (block_for_current_state)
  {
    bool have_current_state = false;
    while (rclcpp::ok() && !have_current_state)
    {
      have_current_state =
          planning_scene_monitor_->getStateMonitor()->waitForCurrentState(node_->now(), ROBOT_STATE_WAIT_TIME /* s */);
      if (!have_current_state)
      {
        RCLCPP_WARN(logger_, "Waiting for the current state");
      }
    }
  }
  moveit::core::RobotStatePtr robot_state = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  return extractRobotState(robot_state, servo_params_.move_group_name);
}

std::pair<bool, KinematicState> Servo::smoothHalt(const KinematicState& halt_state)
{
  auto target_state = halt_state;

  // If all velocities are near zero, robot has decelerated to a stop.
  bool stopped = (target_state.velocities.cwiseAbs().array() < STOPPED_VELOCITY_EPS).all();

  if (!stopped)
  {
    // set target velocity
    target_state.velocities *= 0.0;

    // scale velocity in case of obstacle
    target_state.velocities *= collision_velocity_scale_;

    for (long i = 0; i < halt_state.positions.size(); ++i)
    {
      target_state.positions[i] = halt_state.positions[i] + target_state.velocities[i] * servo_params_.publish_period;
      const double vel = target_state.velocities[i];
      target_state.velocities[i] = (std::abs(vel) > STOPPED_VELOCITY_EPS) ? vel : 0.0;
      target_state.accelerations[i] =
          (target_state.velocities[i] - halt_state.velocities[i]) / servo_params_.publish_period;
    }
  }

  // apply smoothing: this will change target position/velocity to make slow down gradual
  doSmoothing(target_state);

  return std::make_pair(stopped, target_state);
>>>>>>> dcf9515 (Fix Servo pose joint limit halting (#3841))
}

}  // namespace moveit_servo
