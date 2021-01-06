/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

#include <moveit/local_constraint_solver_plugins/decelerate_before_collision.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

namespace moveit_hybrid_planning
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("local_planner_component");
const double CYLCE_TIME = 0.01;  // TODO(sjahr) Add param and proper time handling

DecelerateBeforeCollision::DecelerateBeforeCollision() : loop_rate_(1 / CYLCE_TIME){};

bool DecelerateBeforeCollision::initialize(const rclcpp::Node::SharedPtr& node,
                                           planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor)
{
  planning_scene_monitor_ = planning_scene_monitor;
  node_handle_ = node;
  path_invalidation_event_send_ = false;

  // Initialize PID
  auto joint_names = (planning_scene_monitor_->getRobotModel())->getJointModelNames();
  for (std::size_t i = 0; i < joint_names.size(); i++)
  {
    joint_position_pids_.push_back(control_toolbox::Pid(pid_config_.k_p, pid_config_.k_i, pid_config_.k_d,
                                                        pid_config_.windup_limit, -pid_config_.windup_limit,
                                                        true));  // TODO(sjahr) Add ROS2 param
  }
  return true;
}

trajectory_msgs::msg::JointTrajectory
DecelerateBeforeCollision::solve(robot_trajectory::RobotTrajectory local_trajectory,
                                 std::vector<moveit_msgs::msg::Constraints> local_constraints,
                                 std::shared_ptr<moveit_msgs::action::LocalPlanner::Feedback> feedback)
{
  // Get current planning scene
  planning_scene_monitor_->updateFrameTransforms();

  planning_scene_monitor::LockedPlanningSceneRO locked_planning_scene(planning_scene_monitor_);

  robot_trajectory::RobotTrajectory robot_command(local_trajectory.getRobotModel(), local_trajectory.getGroupName());
  std::vector<std::size_t>* invalid_index = nullptr;

  // Get Current State
  moveit::core::RobotState current_state = locked_planning_scene->getCurrentState();

  // Check if path is valid
  if (locked_planning_scene->isPathValid(local_trajectory, local_trajectory.getGroupName(), false, invalid_index))
  {
    if (path_invalidation_event_send_)
    {
      path_invalidation_event_send_ = false;  // Reset flag
    }

    // Forward next waypoint to the robot controller
    robot_command.addSuffixWayPoint(local_trajectory.getWayPoint(0), 0.0);
  }
  else
  {
    if (!path_invalidation_event_send_)
    {  // Send feedback only once
      feedback->feedback = "collision_ahead";
      path_invalidation_event_send_ = true;  // Set feedback flag
    }

    // Keep current position
    robot_command.addSuffixWayPoint(current_state, 0.0);
  }

  // Transform into joint_trajectory
  moveit_msgs::msg::RobotTrajectory robot_command_msg;
  robot_command.getRobotTrajectoryMsg(robot_command_msg);

  trajectory_msgs::msg::JointTrajectory joint_trajectory = robot_command_msg.joint_trajectory;

  // Transform current state into msg
  moveit_msgs::msg::RobotState current_state_msg;
  robotStateToRobotStateMsg(current_state, current_state_msg);

  // Calculate PID command
  std::vector<double> delta_theta;
  for (std::size_t i = 0; i < joint_trajectory.points[0].positions.size(); i++)
  {
    double error = joint_trajectory.points[0].positions[i] - current_state_msg.joint_state.position[i];
    delta_theta.push_back(joint_position_pids_[i].computeCommand(error, loop_rate_.period().count()));
  }

  // Apply PID command
  trajectory_msgs::msg::JointTrajectoryPoint command_goal_point;
  command_goal_point.time_from_start = rclcpp::Duration(loop_rate_.period().count());
  for (std::size_t i = 0; i < joint_trajectory.points[0].positions.size(); i++)
  {
    // Increment joint
    command_goal_point.positions.push_back(current_state_msg.joint_state.position[i] += delta_theta[i]);
  }

  // Add new command point to local solution joint trajectory
  joint_trajectory.points.clear();
  joint_trajectory.points.push_back(command_goal_point);

  // Return only joint trajectory
  return joint_trajectory;
}
}  // namespace moveit_hybrid_planning

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(moveit_hybrid_planning::DecelerateBeforeCollision,
                       moveit_hybrid_planning::LocalConstraintSolverInterface);
