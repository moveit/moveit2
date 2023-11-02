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
#include <moveit/utils/logger.hpp>

namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("servo_command");
}

/**
 * @brief Helper function to create a move group deltas vector from a sub group deltas vector. A delta vector for the
 * whole move group is created and all entries zeroed. The elements of the subgroup deltas vector are copied into the
 * correct element of the bigger move group delta vector.
 * @param sub_group_deltas Set of command deltas for a subgroup of the move group actuated by servo
 * @param robot_state Current robot state
 * @param servo_params Servo params
 * @param joint_name_group_index_map Mapping between joint subgroup name and move group joint vector position.
 * @return Delta vector for the whole move group. The elements that don't belong to the actuated subgroup are zero.
 */
const Eigen::VectorXd createMoveGroupDelta(const Eigen::VectorXd& sub_group_deltas,
                                           const moveit::core::RobotStatePtr& robot_state,
                                           const servo::Params& servo_params,
                                           const moveit_servo::JointNameToMoveGroupIndexMap& joint_name_group_index_map)
{
  const auto& subgroup_joint_names =
      robot_state->getJointModelGroup(servo_params.active_subgroup)->getActiveJointModelNames();

  // Create
  Eigen::VectorXd move_group_delta_theta = Eigen::VectorXd::Zero(
      robot_state->getJointModelGroup(servo_params.move_group_name)->getActiveJointModelNames().size());
  for (size_t index = 0; index < subgroup_joint_names.size(); index++)
  {
    move_group_delta_theta[joint_name_group_index_map.at(subgroup_joint_names.at(index))] = sub_group_deltas[index];
  }
  return move_group_delta_theta;
};
}  // namespace

namespace moveit_servo
{

JointDeltaResult jointDeltaFromJointJog(const JointJogCommand& command, const moveit::core::RobotStatePtr& robot_state,
                                        const servo::Params& servo_params,
                                        const JointNameToMoveGroupIndexMap& joint_name_group_index_map)
{
  // Find the target joint position based on the commanded joint velocity
  StatusCode status = StatusCode::NO_WARNING;
  const auto& group_name =
      servo_params.active_subgroup.empty() ? servo_params.move_group_name : servo_params.active_subgroup;
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(group_name);
  const auto joint_names = joint_model_group->getActiveJointModelNames();
  Eigen::VectorXd joint_position_delta(joint_names.size());
  Eigen::VectorXd velocities(joint_names.size());

  velocities.setZero();
  bool names_valid = true;

  for (size_t i = 0; i < command.names.size(); ++i)
  {
    auto it = std::find(joint_names.begin(), joint_names.end(), command.names[i]);
    if (it != std::end(joint_names))
    {
      velocities[std::distance(joint_names.begin(), it)] = command.velocities[i];
    }
    else
    {
      RCLCPP_WARN_STREAM(getLogger(), "Invalid joint name: " << command.names[i]);

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
      RCLCPP_WARN_STREAM(getLogger(),
                         "Invalid joint names in joint jog command. Either you're sending commands for a joint "
                         "that is not part of the move group or certain joints cannot be moved because a "
                         "subgroup is active and they are not part of it.");
    }
    if (!velocity_valid)
    {
      RCLCPP_WARN_STREAM(getLogger(), "Invalid velocity values in joint jog command");
    }
  }

  if (!servo_params.active_subgroup.empty() && servo_params.active_subgroup != servo_params.move_group_name)
  {
    return std::make_pair(status, createMoveGroupDelta(joint_position_delta, robot_state, servo_params,
                                                       joint_name_group_index_map));
  }

  return std::make_pair(status, joint_position_delta);
}

JointDeltaResult jointDeltaFromTwist(const TwistCommand& command, const moveit::core::RobotStatePtr& robot_state,
                                     const servo::Params& servo_params, const std::string& planning_frame,
                                     const JointNameToMoveGroupIndexMap& joint_name_group_index_map)
{
  StatusCode status = StatusCode::NO_WARNING;
  const int num_joints =
      robot_state->getJointModelGroup(servo_params.move_group_name)->getActiveJointModelNames().size();
  Eigen::VectorXd joint_position_delta(num_joints);
  Eigen::Vector<double, 6> cartesian_position_delta;

  const bool valid_command = isValidCommand(command);
  const bool is_planning_frame = (command.frame_id == planning_frame);
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
    const auto delta_result =
        jointDeltaFromIK(cartesian_position_delta, robot_state, servo_params, joint_name_group_index_map);
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
        RCLCPP_WARN_STREAM(getLogger(), SERVO_STATUS_CODE_MAP.at(status));
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
      RCLCPP_ERROR_STREAM(getLogger(), "Invalid twist command.");
    }
    if (!is_planning_frame)
    {
      RCLCPP_ERROR_STREAM(getLogger(), "Command frame is: " << command.frame_id << ", expected: " << planning_frame);
    }
  }
  return std::make_pair(status, joint_position_delta);
}

JointDeltaResult jointDeltaFromPose(const PoseCommand& command, const moveit::core::RobotStatePtr& robot_state,
                                    const servo::Params& servo_params, const std::string& planning_frame,
                                    const std::string& ee_frame,
                                    const JointNameToMoveGroupIndexMap& joint_name_group_index_map)
{
  StatusCode status = StatusCode::NO_WARNING;
  const int num_joints =
      robot_state->getJointModelGroup(servo_params.move_group_name)->getActiveJointModelNames().size();
  Eigen::VectorXd joint_position_delta(num_joints);

  const bool valid_command = isValidCommand(command);
  const bool is_planning_frame = (command.frame_id == planning_frame);

  if (valid_command && is_planning_frame)
  {
    Eigen::Vector<double, 6> cartesian_position_delta;

    // Compute linear and angular change needed.
    const Eigen::Isometry3d ee_pose{ robot_state->getGlobalLinkTransform(ee_frame) };
    const Eigen::Quaterniond q_current(ee_pose.rotation()), q_target(command.pose.rotation());
    const Eigen::Quaterniond q_error = q_target * q_current.inverse();
    const Eigen::AngleAxisd angle_axis_error(q_error);

    cartesian_position_delta.head<3>() = command.pose.translation() - ee_pose.translation();
    cartesian_position_delta.tail<3>() = angle_axis_error.axis() * angle_axis_error.angle();

    // Compute the required change in joint angles.
    const auto delta_result =
        jointDeltaFromIK(cartesian_position_delta, robot_state, servo_params, joint_name_group_index_map);
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
      RCLCPP_WARN_STREAM(getLogger(), "Invalid pose command.");
    }
    if (!is_planning_frame)
    {
      RCLCPP_WARN_STREAM(getLogger(), "Command frame is: " << command.frame_id << " expected: " << planning_frame);
    }
  }
  return std::make_pair(status, joint_position_delta);
}

JointDeltaResult jointDeltaFromIK(const Eigen::VectorXd& cartesian_position_delta,
                                  const moveit::core::RobotStatePtr& robot_state, const servo::Params& servo_params,
                                  const JointNameToMoveGroupIndexMap& joint_name_group_index_map)
{
  const auto& group_name =
      servo_params.active_subgroup.empty() ? servo_params.move_group_name : servo_params.active_subgroup;
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(group_name);

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
      RCLCPP_ERROR_STREAM(getLogger(), "Loaded IK plugin does not support group " << joint_model_group->getName());
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
    solution.reserve(current_joint_positions.size());
    moveit_msgs::msg::MoveItErrorCodes err;
    kinematics::KinematicsQueryOptions opts;
    opts.return_approximate_solution = true;
    if (ik_solver->searchPositionIK(next_pose, current_joint_positions, servo_params.publish_period / 2.0, solution,
                                    err, opts))
    {
      // find the difference in joint positions that will get us to the desired pose
      for (size_t i = 0; i < current_joint_positions.size(); ++i)
      {
        delta_theta[i] = solution.at(i) - current_joint_positions.at(i);
      }
    }
    else
    {
      status = StatusCode::INVALID;
      RCLCPP_WARN_STREAM(getLogger(), "Could not find IK solution for requested motion, got error code " << err.val);
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

  if (!servo_params.active_subgroup.empty() && servo_params.active_subgroup != servo_params.move_group_name)
  {
    return std::make_pair(status,
                          createMoveGroupDelta(delta_theta, robot_state, servo_params, joint_name_group_index_map));
  }

  return std::make_pair(status, delta_theta);
}

}  // namespace moveit_servo
