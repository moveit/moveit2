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
   Description: Local solver plugin that uses moveit_servo steer the robot towards a Cartesian goal point.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <moveit/local_planner/local_constraint_solver_interface.h>

#include <moveit_servo/servo.hpp>
#include <servo_solver_parameters.hpp>

namespace moveit::hybrid_planning
{
/**
 * @brief Local solver plugin that utilizes moveit_servo as a local planner
 *
 */
class ServoSolver : public moveit::hybrid_planning::LocalConstraintSolverInterface
{
public:
  /**
   * @brief Initializes moveit servo
   *
   * @param node
   * @param planning_scene_monitor Planning scene monitor to access the current state of the system
   * @param group_name UNUSED
   * @return True if initialization was successful
   */
  bool initialize(const rclcpp::Node::SharedPtr& node,
                  const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                  const std::string& group_name) override;
  /**
   * @brief Reset rolling window
   *
   * @return Always returns true
   */
  bool reset() override;

  /**
   * @brief Solves local planning problem with servo. The first waypoint of the local trajectory is used as goal point
   * for servo. After computing FK for that waypoint, the difference between the current and goal reference frame is used
   * to calculate a twist command for servo. The status of servo is returned as feedback to the hybrid planning manager.
   *
   * @param local_trajectory Reference trajectory, only the first waypoint is used
   * @param local_goal UNUSED
   * @param local_solution Joint trajectory containing a sequence of servo commands
   * @return moveit_msgs::action::LocalPlanner::Feedback containing the servo status code
   */
  moveit_msgs::action::LocalPlanner::Feedback
  solve(const robot_trajectory::RobotTrajectory& local_trajectory,
        const std::shared_ptr<const moveit_msgs::action::LocalPlanner::Goal> local_goal,
        trajectory_msgs::msg::JointTrajectory& local_solution) override;

private:
  rclcpp::Node::SharedPtr node_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  servo_solver_parameters::Params solver_parameters_;
  servo::Params servo_parameters_;

  // Servo cpp interface
  std::unique_ptr<moveit_servo::Servo> servo_;

  // Command queue to build trajectory message and add current robot state
  std::deque<moveit_servo::KinematicState> joint_cmd_rolling_window_;
};
}  // namespace moveit::hybrid_planning
