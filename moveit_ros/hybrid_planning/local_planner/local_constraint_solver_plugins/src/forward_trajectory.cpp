/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
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

/* Author: Sebastian Jahr
 */

#include <moveit/local_constraint_solver_plugins/forward_trajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

namespace moveit_hybrid_planning
{
bool ForwardTrajectory::initialize(const rclcpp::Node::SharedPtr& node,
                                   const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                                   const std::string& group_name)
{
  return true;
}

moveit_msgs::action::LocalPlanner::Feedback
ForwardTrajectory::solve(const robot_trajectory::RobotTrajectory& local_trajectory,
                         const std::vector<moveit_msgs::msg::Constraints>& local_constraints,
                         trajectory_msgs::msg::JointTrajectory& local_solution)
{
  // Create controller command trajectory
  robot_trajectory::RobotTrajectory robot_command(local_trajectory.getRobotModel(), local_trajectory.getGroupName());

  // Forward next waypoint to the robot controller
  robot_command.addSuffixWayPoint(local_trajectory.getWayPoint(0), 0.0);

  // Transform into ROS 2 msg
  moveit_msgs::msg::RobotTrajectory robot_command_msg;
  robot_command.getRobotTrajectoryMsg(robot_command_msg);
  local_solution = robot_command_msg.joint_trajectory;

  // Empty feedback result
  moveit_msgs::action::LocalPlanner::Feedback feedback_result;
  return feedback_result;
}
}  // namespace moveit_hybrid_planning

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(moveit_hybrid_planning::ForwardTrajectory,
                       moveit_hybrid_planning::LocalConstraintSolverInterface);
