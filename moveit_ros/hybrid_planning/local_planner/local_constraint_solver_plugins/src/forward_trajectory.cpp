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

#include <moveit/local_constraint_solver_plugins/forward_trajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("local_planner_component");
}

namespace moveit::hybrid_planning
{
bool ForwardTrajectory::initialize(const rclcpp::Node::SharedPtr& node,
                                   const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                                   const std::string& group_name)
{
  // Load parameter & initialize member variables
  if (node->has_parameter("stop_before_collision"))
    node->get_parameter<bool>("stop_before_collision", stop_before_collision_);
  else
    stop_before_collision_ = node->declare_parameter<bool>("stop_before_collision", false);
  planning_scene_monitor_ = planning_scene_monitor;
  node_handle_ = node;
  path_invalidation_event_send_ = false;
  return true;
}

bool ForwardTrajectory::reset()
{
  path_invalidation_event_send_ = false;
  return true;
};

moveit_msgs::action::LocalPlanner::Feedback
ForwardTrajectory::solve(const robot_trajectory::RobotTrajectory& local_trajectory,
                         const std::shared_ptr<const moveit_msgs::action::LocalPlanner::Goal> local_goal,
                         trajectory_msgs::msg::JointTrajectory& local_solution)
{
  // Create controller command trajectory
  robot_trajectory::RobotTrajectory robot_command(local_trajectory.getRobotModel(), local_trajectory.getGroupName());

  // Feedback
  moveit_msgs::action::LocalPlanner::Feedback feedback_result;

  if (!stop_before_collision_)
  {
    robot_command.addSuffixWayPoint(local_trajectory.getWayPoint(0), 0.0);
  }
  else
  {
    // Get current planning scene
    planning_scene_monitor_->updateFrameTransforms();

    planning_scene_monitor::LockedPlanningSceneRO locked_planning_scene(planning_scene_monitor_);

    // Get Current State
    const moveit::core::RobotState& current_state = locked_planning_scene->getCurrentState();

    // Check if path is valid
    if (locked_planning_scene->isPathValid(local_trajectory, local_trajectory.getGroupName(), false))
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
        feedback_result.feedback = "collision_ahead";
        path_invalidation_event_send_ = true;  // Set feedback flag
      }
      RCLCPP_INFO(LOGGER, "Collision ahead, hold current position");
      // Keep current position
      moveit::core::RobotState current_state_command(current_state);
      if (current_state_command.hasVelocities())
      {
        current_state_command.zeroVelocities();
      }
      if (current_state_command.hasAccelerations())
      {
        current_state_command.zeroAccelerations();
      }
      robot_command.empty();
      robot_command.addSuffixWayPoint(current_state, 0.0);
    }
  }

  // Transform robot trajectory into joint_trajectory message
  moveit_msgs::msg::RobotTrajectory robot_command_msg;
  robot_command.getRobotTrajectoryMsg(robot_command_msg);
  local_solution = robot_command_msg.joint_trajectory;

  return feedback_result;
}
}  // namespace moveit::hybrid_planning

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(moveit::hybrid_planning::ForwardTrajectory,
                       moveit::hybrid_planning::LocalConstraintSolverInterface);
