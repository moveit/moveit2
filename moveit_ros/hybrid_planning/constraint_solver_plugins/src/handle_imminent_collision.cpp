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

bool HandleImminentCollision::initialize(const rclcpp::Node::SharedPtr& node)
{
  node_handle_ = node;
  feedback_send_ = false;
  return true;
}

trajectory_msgs::msg::JointTrajectory
HandleImminentCollision::solve(std::vector<moveit_msgs::msg::Constraints> local_problem,
                               std::vector<moveit_msgs::msg::Constraints> additional_constraints,
                               planning_scene::PlanningScenePtr planning_scene,
                               std::shared_ptr<moveit_msgs::action::LocalPlanner::Feedback> feedback)
{
  trajectory_msgs::msg::JointTrajectory local_joint_trajectory;
  for (auto& joint_constraint : local_problem[0].joint_constraints)
  {
    local_joint_trajectory.joint_names.push_back(joint_constraint.joint_name);
  }
  for (auto& waypoint_constraints : local_problem)
  {
    trajectory_msgs::msg::JointTrajectoryPoint waypoint;
    for (auto& joint_constraint : waypoint_constraints.joint_constraints)
    {
      waypoint.positions.push_back(joint_constraint.position);
    }
    local_joint_trajectory.points.push_back(waypoint);
  }
  robot_trajectory::RobotTrajectory local_trajectory(planning_scene->getRobotModel(), "panda_arm");
  local_trajectory.setRobotTrajectoryMsg(planning_scene->getCurrentState(), local_joint_trajectory);
  moveit_msgs::msg::RobotTrajectory local_trajectory_msg;
  local_trajectory.getRobotTrajectoryMsg(local_trajectory_msg, local_joint_trajectory.joint_names);

  // Check if path is valid
  moveit_msgs::msg::RobotState current_state_msg;
  robotStateToRobotStateMsg(planning_scene->getCurrentState(), current_state_msg);

  trajectory_msgs::msg::JointTrajectory local_solution;
  trajectory_msgs::msg::JointTrajectoryPoint waypoint;

  // Check if path is valid
  if (planning_scene->isPathValid(current_state_msg, local_trajectory_msg))
  {
    if (feedback_send_)
    {
      feedback_send_ = false;  // Reset feedback flag
    }

    // Forward closest waypoint to the robot controller
    for (auto& joint_constraint : local_problem[0].joint_constraints)
    {
      local_solution.joint_names.push_back(joint_constraint.joint_name);
      waypoint.positions.push_back(joint_constraint.position);
    }
    local_solution.points.push_back(waypoint);
  }
  else
  {
    if (!feedback_send_)
    {  // Send feedback only once
      feedback->feedback = "collision_ahead";
      feedback_send_ = true;
    }
    // Keep current position
    moveit::core::RobotState current_state = planning_scene->getCurrentState();
    std::vector<double> joint_states;
    current_state.copyJointGroupPositions("panda_arm", joint_states);
    for (size_t i = 0; i < local_joint_trajectory.joint_names.size(); i++)
    {
      local_solution.joint_names.push_back(local_problem[0].joint_constraints[i].joint_name);
      waypoint.positions.push_back(joint_states[i]);
    }
    local_solution.points.push_back(waypoint);
  }
  return local_solution;
}
}  // namespace moveit_hybrid_planning

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(moveit_hybrid_planning::HandleImminentCollision,
                       moveit_hybrid_planning::ConstraintSolverInterface);
