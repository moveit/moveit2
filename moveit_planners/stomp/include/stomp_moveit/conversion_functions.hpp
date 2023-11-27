/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** @file
 * @author Henning Kayser
 * @brief Helper functions for converting between MoveIt types and plain Eigen types.
 */

#pragma once

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/robot_state.h>

namespace stomp_moveit
{
using Joints = std::vector<const moveit::core::JointModel*>;

/**
 * Copies the position values of a robot state filtered by the provided joints.
 *
 * @param state  The RobotState to copy the values from
 * @param joints The joints that should be considered
 *
 * @return       The vector containing the joint values
 */
std::vector<double> getPositions(const moveit::core::RobotState& state, const Joints& joints)
{
  std::vector<double> positions;
  for (const auto& joint : joints)
  {
    positions.push_back(*state.getJointPositions(joint));
  }

  return positions;
}

/**
 * Writes the provided position values into a robot state.
 *
 * This function requires the dimension of values and joints to be equal!
 *
 * @param values The joint position values to copy from
 * @param joints The joints that should be considered
 * @param state  The robot state to update with the new joint values
 */
void setJointPositions(const Eigen::VectorXd& values, const Joints& joints, moveit::core::RobotState& state)
{
  for (size_t joint_index = 0; joint_index < joints.size(); ++joint_index)
  {
    state.setJointPositions(joints[joint_index], &values[joint_index]);
  }
}

/**
 * Writes the provided position value sequence into a robot trajectory.
 *
 * @param trajectory_values The joint value sequence to copy the waypoints from
 * @param reference_state   A robot state providing default joint values and robot model
 * @param trajectory        The robot trajectory containing waypoints with updated values
 */
void fillRobotTrajectory(const Eigen::MatrixXd& trajectory_values, const moveit::core::RobotState& reference_state,
                         robot_trajectory::RobotTrajectory& trajectory)
{
  trajectory.clear();
  const auto& active_joints = trajectory.getGroup() ? trajectory.getGroup()->getActiveJointModels() :
                                                      trajectory.getRobotModel()->getActiveJointModels();
  assert(static_cast<std::size_t>(trajectory_values.rows()) == active_joints.size());

  for (int timestep = 0; timestep < trajectory_values.cols(); ++timestep)
  {
    const auto waypoint = std::make_shared<moveit::core::RobotState>(reference_state);
    setJointPositions(trajectory_values.col(timestep), active_joints, *waypoint);

    trajectory.addSuffixWayPoint(waypoint, 0.1 /* placeholder dt */);
  }
}

/**
 * Constructs a new robot trajectory with the waypoints provided in the input matrix.
 *
 * @param trajectory_values The waypoints and positions to copy
 * @param reference_state   The RobotState with default joint values and robot model
 * @param group             An optional JointModelGroup to filter for joints
 *
 * @return                  The created RobotTrajectory containing updated waypoints
 */
robot_trajectory::RobotTrajectory matrixToRobotTrajectory(const Eigen::MatrixXd& trajectory_values,
                                                          const moveit::core::RobotState& reference_state,
                                                          const moveit::core::JointModelGroup* group = nullptr)
{
  robot_trajectory::RobotTrajectory trajectory(reference_state.getRobotModel(), group);
  fillRobotTrajectory(trajectory_values, reference_state, trajectory);
  return trajectory;
}

/**
 * Copies the waypoint positions of a RobotTrajectory into an Eigen matrix.
 *
 * @param trajectory The RobotTrajectory to read the waypoint positions fromi
 *
 * @return           The matrix representing a sequence of waypoint positions
 */
Eigen::MatrixXd robotTrajectoryToMatrix(const robot_trajectory::RobotTrajectory& trajectory)
{
  const auto& active_joints = trajectory.getGroup() ? trajectory.getGroup()->getActiveJointModels() :
                                                      trajectory.getRobotModel()->getActiveJointModels();

  Eigen::MatrixXd trajectory_values(active_joints.size(), trajectory.getWayPointCount());

  for (int timestep = 0; timestep < trajectory_values.cols(); ++timestep)
  {
    const auto& waypoint = trajectory.getWayPoint(timestep);
    for (size_t joint_index = 0; joint_index < active_joints.size(); ++joint_index)
    {
      trajectory_values(joint_index, timestep) = *waypoint.getJointPositions(active_joints[joint_index]);
    }
  }

  return trajectory_values;
}

}  // namespace stomp_moveit
