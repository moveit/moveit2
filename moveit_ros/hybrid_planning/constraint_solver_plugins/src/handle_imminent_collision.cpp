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

#include <moveit/constraint_solver_plugins/handle_imminent_collision.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

namespace moveit_hybrid_planning
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("local_planner_component");

bool HandleImminentCollision::initialize(const rclcpp::Node::SharedPtr& node,
                                         planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor)
{
  planning_scene_monitor_ = planning_scene_monitor;
  node_handle_ = node;
  feedback_send_ = false;
  return true;
}

trajectory_msgs::msg::JointTrajectory
HandleImminentCollision::solve(robot_trajectory::RobotTrajectory local_trajectory,
                               std::vector<moveit_msgs::msg::Constraints> local_constraints,
                               std::shared_ptr<moveit_msgs::action::LocalPlanner::Feedback> feedback)
{
  // Transform local_trajectory into msg data structure to use isPathValid()
  moveit_msgs::msg::RobotTrajectory local_trajectory_msg;
  local_trajectory.getRobotTrajectoryMsg(local_trajectory_msg);

  // Clone current planning scene
  planning_scene_monitor_->updateFrameTransforms();
  planning_scene_monitor_->lockSceneRead();  // LOCK planning scene
  planning_scene::PlanningScenePtr planning_scene = planning_scene::PlanningScene::clone(
      planning_scene_monitor_->getPlanningScene());  // TODO remove expensive planning scene cloning
  planning_scene_monitor_->unlockSceneRead();        // UNLOCK planning scene

  // Get Current State
  moveit_msgs::msg::RobotState current_state_msg;
  robotStateToRobotStateMsg(planning_scene->getCurrentState(), current_state_msg);

  robot_trajectory::RobotTrajectory robot_command(local_trajectory.getRobotModel(), local_trajectory.getGroupName());

  // Check if path is valid
  if (planning_scene->isPathValid(current_state_msg, local_trajectory_msg))
  {
    if (feedback_send_)
    {
      feedback_send_ = false;  // Reset feedback flag
    }

    // Forward next waypoint to the robot controller
    robot_command.addSuffixWayPoint(local_trajectory.getWayPoint(0), 0.0);
  }
  else
  {
    if (!feedback_send_)
    {  // Send feedback only once
      feedback->feedback = "collision_ahead";
      feedback_send_ = true;  // Set feedback flag
    }

    // Keep current position
    moveit::core::RobotState current_state = planning_scene->getCurrentState();
    robot_command.addSuffixWayPoint(current_state, 0.0);
  }

  // Transform RobotTrajectory into RobotTrajectoryMsg
  moveit_msgs::msg::RobotTrajectory robot_command_msg;
  robot_command.getRobotTrajectoryMsg(robot_command_msg);

  // Return only joint trajectory
  return robot_command_msg.joint_trajectory;
}
}  // namespace moveit_hybrid_planning

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(moveit_hybrid_planning::HandleImminentCollision,
                       moveit_hybrid_planning::ConstraintSolverInterface);
