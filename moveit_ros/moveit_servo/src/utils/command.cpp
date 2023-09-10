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

/*      Title     : command.cpp
 *      Project   : moveit_servo
 *      Created   : 06/04/2023
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson, V Mohammed Ibrahim
 */

#include <moveit_servo/utils/command.hpp>

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.command_processor");
}

namespace moveit_servo
{

JointDeltaResult jointDeltaFromJointJog(const JointJogCommand& command, const moveit::core::RobotStatePtr& robot_state,
                                        const servo::Params& servo_params)
{
  // Find the target joint position based on the commanded joint velocity
  StatusCode status = StatusCode::NO_WARNING;
  const auto joint_names = robot_state->getJointModelGroup(servo_params.move_group_name)->getActiveJointModelNames();
  Eigen::VectorXd joint_position_delta(joint_names.size());
  Eigen::VectorXd velocities(joint_names.size());

  velocities.setZero();
  bool names_valid = true;

  for (size_t i = 0; i < command.names.size(); i++)
  {
    auto it = std::find(joint_names.begin(), joint_names.end(), command.names[i]);
    if (it != std::end(joint_names))
    {
      velocities[std::distance(joint_names.begin(), it)] = command.velocities[i];
    }
    else
    {
      names_valid = false;
      break;
    }
  }
  const bool velocity_valid = isValidCommand(velocities);
  if (names_valid && velocity_valid)
  {
    joint_position_delta = velocities * servo_params.publish_period;
    if (servo_params.command_in_type == "unitless")
    {
      joint_position_delta *= servo_params.scale.joint;
    }
  }
  else
  {
    status = StatusCode::INVALID;
    if (!names_valid)
    {
      RCLCPP_WARN_STREAM(LOGGER, "Invalid joint names in joint jog command");
    }
    if (!velocity_valid)
    {
      RCLCPP_WARN_STREAM(LOGGER, "Invalid velocity values in joint jog command");
    }
  }
  return std::make_pair(status, joint_position_delta);
}

JointDeltaResult jointDeltaFromTwist(const TwistCommand& command, const moveit::core::RobotStatePtr& robot_state,
                                     const servo::Params& servo_params)
{
  StatusCode status = StatusCode::NO_WARNING;
  const int num_joints =
      robot_state->getJointModelGroup(servo_params.move_group_name)->getActiveJointModelNames().size();
  Eigen::VectorXd joint_position_delta(num_joints);
  Eigen::VectorXd cartesian_position_delta;

  const bool valid_command = isValidCommand(command);
  const bool is_planning_frame = (command.frame_id == servo_params.planning_frame);
  const bool is_zero = command.velocities.isZero();
  if (!is_zero && is_planning_frame && valid_command)
  {
    // Compute the Cartesian position delta based on incoming command, assumed to be in m/s
    cartesian_position_delta = command.velocities * servo_params.publish_period;
    // This scaling is supposed to be applied to the command.
    // But since it is only used here, we avoid creating a copy of the command,
    // by applying the scaling to the computed Cartesian delta instead.
    if (servo_params.command_in_type == "unitless")
    {
      cartesian_position_delta.head<3>() *= servo_params.scale.linear;
      cartesian_position_delta.tail<3>() *= servo_params.scale.rotational;
    }

    // Compute the required change in joint angles.
    const auto delta_result = jointDeltaFromIK(cartesian_position_delta, robot_state, servo_params);
    status = delta_result.first;
    if (status != StatusCode::INVALID)
    {
      joint_position_delta = delta_result.second;
      // Get velocity scaling information for singularity.
      const std::pair<double, StatusCode> singularity_scaling_info =
          velocityScalingFactorForSingularity(robot_state, cartesian_position_delta, servo_params);
      // Apply velocity scaling for singularity, if there was any scaling.
      if (singularity_scaling_info.second != StatusCode::NO_WARNING)
      {
        status = singularity_scaling_info.second;
        RCLCPP_WARN_STREAM(LOGGER, SERVO_STATUS_CODE_MAP.at(status));
        joint_position_delta *= singularity_scaling_info.first;
      }
    }
  }
  else if (is_zero)
  {
    joint_position_delta.setZero();
  }
  else
  {
    status = StatusCode::INVALID;
    if (!valid_command)
    {
      RCLCPP_WARN_STREAM(LOGGER, "Invalid twist command.");
    }
    if (!is_planning_frame)
    {
      RCLCPP_WARN_STREAM(LOGGER,
                         "Command frame is: " << command.frame_id << " expected: " << servo_params.planning_frame);
    }
  }
  return std::make_pair(status, joint_position_delta);
}

JointDeltaResult jointDeltaFromPose(const PoseCommand& command, const moveit::core::RobotStatePtr& robot_state,
                                    const servo::Params& servo_params)
{
  StatusCode status = StatusCode::NO_WARNING;
  const int num_joints =
      robot_state->getJointModelGroup(servo_params.move_group_name)->getActiveJointModelNames().size();
  Eigen::VectorXd joint_position_delta(num_joints);

  const bool valid_command = isValidCommand(command);
  const bool is_planning_frame = command.frame_id == servo_params.planning_frame;

  if (valid_command && is_planning_frame)
  {
    Eigen::Vector<double, 6> cartesian_position_delta;

    // Compute linear and angular change needed.
    const Eigen::Isometry3d ee_pose{ robot_state->getGlobalLinkTransform(servo_params.ee_frame) };
    const Eigen::Quaterniond q_current(ee_pose.rotation()), q_target(command.pose.rotation());
    const Eigen::Quaterniond q_error = q_target * q_current.inverse();
    const Eigen::AngleAxisd angle_axis_error(q_error);

    cartesian_position_delta.head<3>() = command.pose.translation() - ee_pose.translation();
    cartesian_position_delta.tail<3>() = angle_axis_error.axis() * angle_axis_error.angle();

    // Compute the required change in joint angles.
    const auto delta_result = jointDeltaFromIK(cartesian_position_delta, robot_state, servo_params);
    status = delta_result.first;
    if (status != StatusCode::INVALID)
    {
      joint_position_delta = delta_result.second;
    }
  }
  else
  {
    status = StatusCode::INVALID;
    if (!valid_command)
    {
      RCLCPP_WARN_STREAM(LOGGER, "Invalid pose command.");
    }
    if (!is_planning_frame)
    {
      RCLCPP_WARN_STREAM(LOGGER,
                         "Command frame is: " << command.frame_id << " expected: " << servo_params.planning_frame);
    }
  }
  return std::make_pair(status, joint_position_delta);
}

JointDeltaResult jointDeltaFromIK(const Eigen::VectorXd& cartesian_position_delta,
                                  const moveit::core::RobotStatePtr& robot_state, const servo::Params& servo_params)
{
  const moveit::core::JointModelGroup* joint_model_group =
      robot_state->getJointModelGroup(servo_params.move_group_name);

  std::vector<double> current_joint_positions;
  robot_state->copyJointGroupPositions(joint_model_group, current_joint_positions);

  Eigen::VectorXd delta_theta(current_joint_positions.size());
  StatusCode status = StatusCode::NO_WARNING;

  const kinematics::KinematicsBaseConstPtr ik_solver = joint_model_group->getSolverInstance();
  bool ik_solver_supports_group = true;
  if (ik_solver)
  {
    ik_solver_supports_group = ik_solver->supportsGroup(joint_model_group);
    if (!ik_solver_supports_group)
    {
      status = StatusCode::INVALID;
      RCLCPP_ERROR_STREAM(LOGGER, "Loaded IK plugin does not support group " << joint_model_group->getName());
    }
  }

  if (ik_solver && ik_solver_supports_group)
  {
    const Eigen::Isometry3d base_to_tip_frame_transform =
        robot_state->getGlobalLinkTransform(ik_solver->getBaseFrame()).inverse() *
        robot_state->getGlobalLinkTransform(ik_solver->getTipFrame());

    const geometry_msgs::msg::Pose next_pose =
        poseFromCartesianDelta(cartesian_position_delta, base_to_tip_frame_transform);

    // setup for IK call
    std::vector<double> solution;
    moveit_msgs::msg::MoveItErrorCodes err;
    kinematics::KinematicsQueryOptions opts;
    opts.return_approximate_solution = true;
    if (ik_solver->searchPositionIK(next_pose, current_joint_positions, servo_params.publish_period / 2.0, solution,
                                    err, opts))
    {
      // find the difference in joint positions that will get us to the desired pose
      for (size_t i = 0; i < current_joint_positions.size(); ++i)
      {
        delta_theta[i] = solution[i] - current_joint_positions[i];
      }
    }
    else
    {
      status = StatusCode::INVALID;
      RCLCPP_WARN_STREAM(LOGGER, "Could not find IK solution for requested motion, got error code " << err.val);
    }
  }
  else
  {
    // Robot does not have an IK solver, use inverse Jacobian to compute IK.
    const Eigen::MatrixXd jacobian = robot_state->getJacobian(joint_model_group);
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd =
        Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::MatrixXd matrix_s = svd.singularValues().asDiagonal();
    const Eigen::MatrixXd pseudo_inverse = svd.matrixV() * matrix_s.inverse() * svd.matrixU().transpose();

    delta_theta = pseudo_inverse * cartesian_position_delta;
  }

  return std::make_pair(status, delta_theta);
}

}  // namespace moveit_servo
