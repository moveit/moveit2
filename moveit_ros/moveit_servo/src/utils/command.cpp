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
  return moveit::getLogger("moveit.ros.servo");
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
  const auto& group_name =
      servo_params.active_subgroup.empty() ? servo_params.move_group_name : servo_params.active_subgroup;
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(group_name);
  const auto joint_names = joint_model_group->getActiveJointModelNames();
  Eigen::VectorXd joint_position_delta(joint_names.size());
  Eigen::VectorXd velocities(joint_names.size());

  velocities.setZero();
  if (command.velocities.size() != command.names.size())
  {
    RCLCPP_WARN_STREAM(getLogger(), "Invalid joint jog command. Each joint name must have one corresponding "
                                    "velocity command. Received "
                                        << command.names.size() << " joints with " << command.velocities.size()
                                        << " commands.");
    return std::make_pair(StatusCode::INVALID, joint_position_delta);
  }

  for (size_t i = 0; i < command.names.size(); ++i)
  {
    auto it = std::find(joint_names.begin(), joint_names.end(), command.names[i]);
    if (it != std::end(joint_names))
    {
      velocities[std::distance(joint_names.begin(), it)] = command.velocities[i];
    }
    else
    {
      RCLCPP_WARN_STREAM(getLogger(), "Invalid joint name: " << command.names[i]
                                                             << "Either you're sending commands for a joint "
                                                                "that is not part of the move group or certain joints "
                                                                "cannot be moved because a "
                                                                "subgroup is active and they are not part of it.");
      return std::make_pair(StatusCode::INVALID, joint_position_delta);
    }
  }

  if (!isValidCommand(velocities))
  {
    RCLCPP_WARN_STREAM(getLogger(), "Invalid velocity values in joint jog command");
    return std::make_pair(StatusCode::INVALID, joint_position_delta);
  }

  joint_position_delta = velocities * servo_params.publish_period;
  if (servo_params.command_in_type == "unitless")
  {
    joint_position_delta *= servo_params.scale.joint;
  }

  if (!servo_params.active_subgroup.empty() && servo_params.active_subgroup != servo_params.move_group_name)
  {
    return std::make_pair(StatusCode::NO_WARNING, createMoveGroupDelta(joint_position_delta, robot_state, servo_params,
                                                                       joint_name_group_index_map));
  }

  return std::make_pair(StatusCode::NO_WARNING, joint_position_delta);
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

  if (command.frame_id != planning_frame)
  {
    RCLCPP_WARN_STREAM(getLogger(), "Command frame is: " << command.frame_id << ", expected: " << planning_frame);
    return std::make_pair(StatusCode::INVALID, joint_position_delta);
  }

  if (!isValidCommand(command))
  {
    RCLCPP_WARN_STREAM(getLogger(), "Invalid twist command.");
    return std::make_pair(StatusCode::INVALID, joint_position_delta);
  }

  if (command.velocities.isZero())
  {
    joint_position_delta.setZero();
  }
  else
  {
    // Compute the Cartesian position delta based on incoming twist command.
    cartesian_position_delta = command.velocities * servo_params.publish_period;
    // This scaling is supposed to be applied to the command.
    // But since it is only used here, we avoid creating a copy of the command,
    // by applying the scaling to the computed Cartesian delta instead.
    if (servo_params.command_in_type == "unitless")
    {
      cartesian_position_delta.head<3>() *= servo_params.scale.linear;
      cartesian_position_delta.tail<3>() *= servo_params.scale.rotational;
    }
    else if (servo_params.command_in_type == "speed_units")
    {
      if (servo_params.scale.linear > 0.0)
      {
        const auto linear_speed_scale = command.velocities.head<3>().norm() / servo_params.scale.linear;
        if (linear_speed_scale > 1.0)
        {
          cartesian_position_delta.head<3>() /= linear_speed_scale;
        }
      }
      if (servo_params.scale.rotational > 0.0)
      {
        const auto angular_speed_scale = command.velocities.tail<3>().norm() / servo_params.scale.rotational;
        if (angular_speed_scale > 1.0)
        {
          cartesian_position_delta.tail<3>() /= angular_speed_scale;
        }
      }
    }

    // Compute the required change in joint angles.
    const auto delta_result =
        jointDeltaFromIK(cartesian_position_delta, robot_state, servo_params, joint_name_group_index_map);
    status = delta_result.first;
    if (status != StatusCode::INVALID)
    {
      joint_position_delta = delta_result.second;
      // Get velocity scaling information for singularity.
      const auto singularity_scaling_info =
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

  if (!isValidCommand(command))
  {
    RCLCPP_WARN_STREAM(getLogger(), "Invalid pose command.");
    return std::make_pair(StatusCode::INVALID, joint_position_delta);
  }

  if (command.frame_id != planning_frame)
  {
    RCLCPP_WARN_STREAM(getLogger(), "Command frame is: " << command.frame_id << " expected: " << planning_frame);
    return std::make_pair(StatusCode::INVALID, joint_position_delta);
  }

  Eigen::Vector<double, 6> cartesian_position_delta;

  // Compute linear and angular change needed.
  const Eigen::Isometry3d ee_pose{ robot_state->getGlobalLinkTransform(planning_frame).inverse() *
                                   robot_state->getGlobalLinkTransform(ee_frame) };
  const Eigen::Quaterniond q_current(ee_pose.rotation());
  Eigen::Quaterniond q_target(command.pose.rotation());
  Eigen::Vector3d translation_error = command.pose.translation() - ee_pose.translation();

  // Limit the commands by the maximum linear and angular speeds provided.
  if (servo_params.scale.linear > 0.0)
  {
    const auto linear_speed_scale =
        (translation_error.norm() / servo_params.publish_period) / servo_params.scale.linear;
    if (linear_speed_scale > 1.0)
    {
      translation_error /= linear_speed_scale;
    }
  }
  if (servo_params.scale.rotational > 0.0)
  {
    const auto angular_speed_scale =
        (std::abs(q_target.angularDistance(q_current)) / servo_params.publish_period) / servo_params.scale.rotational;
    if (angular_speed_scale > 1.0)
    {
      q_target = q_current.slerp(1.0 / angular_speed_scale, q_target);
    }
  }

  // Compute the Cartesian deltas from the velocity-scaled values.
  const auto angle_axis_error = Eigen::AngleAxisd(q_target * q_current.inverse());
  cartesian_position_delta.head<3>() = translation_error;
  cartesian_position_delta.tail<3>() = angle_axis_error.axis() * angle_axis_error.angle();

  // Compute the required change in joint angles.
  const auto delta_result =
      jointDeltaFromIK(cartesian_position_delta, robot_state, servo_params, joint_name_group_index_map);
  status = delta_result.first;
  if (status != StatusCode::INVALID)
  {
    joint_position_delta = delta_result.second;
    // Get velocity scaling information for singularity.
    const auto singularity_scaling_info =
        velocityScalingFactorForSingularity(robot_state, cartesian_position_delta, servo_params);
    // Apply velocity scaling for singularity, if there was any scaling.
    if (singularity_scaling_info.second != StatusCode::NO_WARNING)
    {
      status = singularity_scaling_info.second;
      RCLCPP_WARN_STREAM(getLogger(), SERVO_STATUS_CODE_MAP.at(status));
      joint_position_delta *= singularity_scaling_info.first;
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
