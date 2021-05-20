/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik Inc.
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

#include <moveit/local_constraint_solver_plugins/servo_solver.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

namespace moveit_hybrid_planning
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("local_planner_component");

bool ServoSolver::initialize(const rclcpp::Node::SharedPtr& node,
                             const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                             const std::string& group_name)
{
  // Load parameter & initialize member variables
  if (node->has_parameter("velocity_scaling_threshold"))
    node->get_parameter<double>("velocity_scaling_threshold", velocity_scaling_threshold_);
  else
    velocity_scaling_threshold_ = node->declare_parameter<double>("velocity_scaling_threshold", 0.0);

  planning_scene_monitor_ = planning_scene_monitor;
  node_handle_ = node;
  replan_ = false;
  feedback_send_ = false;
  joint_cmd_pub_ = node_handle_->create_publisher<control_msgs::msg::JointJog>("servo_demo_node/delta_joint_cmds", 10);

  // Get Servo Parameters
  auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node_handle_, LOGGER);
  if (!servo_parameters)
  {
    RCLCPP_FATAL(LOGGER, "Failed to load the servo parameters");
    return false;
  }

  // Create Servo and start it
  servo_ = std::make_unique<moveit_servo::Servo>(node_handle_, servo_parameters, planning_scene_monitor_);
  servo_->start();

  // Create publisher to send servo commands
  joint_cmd_pub_ =
      node_handle_->create_publisher<control_msgs::msg::JointJog>(servo_parameters->joint_command_in_topic, 1);

  // Subscribe to the collision_check topic
  collision_velocity_scale_sub_ = node_handle_->create_subscription<std_msgs::msg::Float64>(
      "~/collision_velocity_scale", 2, [this](const std_msgs::msg::Float64::SharedPtr msg) {
        auto collision_velocity_scale = msg.get()->data;
        if (collision_velocity_scale < velocity_scaling_threshold_)
        {
          replan_ = true;
        }
        else
        {
          replan_ = false;
        }
      });

  return true;
}

bool ServoSolver::reset()
{
  RCLCPP_INFO(LOGGER, "Reset Servo Solver");
  return true;
};

moveit_msgs::action::LocalPlanner::Feedback
ServoSolver::solve(const robot_trajectory::RobotTrajectory& local_trajectory,
                   const std::vector<moveit_msgs::msg::Constraints>& local_constraints,
                   trajectory_msgs::msg::JointTrajectory& local_solution)
{
  // Create Feedback
  moveit_msgs::action::LocalPlanner::Feedback feedback_result;

  // Transform next robot trajectory waypoint into JointJog message
  moveit_msgs::msg::RobotTrajectory robot_command;
  local_trajectory.getRobotTrajectoryMsg(robot_command);

  // Replan if velocity scaling is below threshold
  if (replan_)
  {
    if (!feedback_send_)
    {
      feedback_result.feedback = "collision_ahead";
    }
    feedback_send_ = true;  // Give the architecture time to handle feedback
  }
  else
  {
    feedback_send_ = false;
  }

  auto msg = std::make_unique<control_msgs::msg::JointJog>();
  msg->header.stamp = node_handle_->now();
  msg->joint_names = robot_command.joint_trajectory.joint_names;
  msg->velocities = robot_command.joint_trajectory.points[0].velocities;
  joint_cmd_pub_->publish(std::move(msg));
  return feedback_result;
}
}  // namespace moveit_hybrid_planning

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(moveit_hybrid_planning::ServoSolver, moveit_hybrid_planning::LocalConstraintSolverInterface);
