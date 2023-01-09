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
   Description: Defines an interface for a local constraint solver plugin implementation for the local planner component node.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/constraints.hpp>

#include <moveit_msgs/action/local_planner.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace moveit::hybrid_planning
{
/**
 * Class LocalConstraintSolverInterface - Base class for a local constrain solver.
 */
class LocalConstraintSolverInterface
{
public:
  LocalConstraintSolverInterface() = default;
  LocalConstraintSolverInterface(const LocalConstraintSolverInterface&) = default;
  LocalConstraintSolverInterface(LocalConstraintSolverInterface&&) = default;
  LocalConstraintSolverInterface& operator=(const LocalConstraintSolverInterface&) = default;
  LocalConstraintSolverInterface& operator=(LocalConstraintSolverInterface&&) = default;
  virtual ~LocalConstraintSolverInterface() = default;
  /**
   * Initialize local constraint solver
   * @return True if initialization was successful
   */
  virtual bool initialize(const rclcpp::Node::SharedPtr& node,
                          const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                          const std::string& group_name) = 0;

  /**
   * Solve local planning problem for the current iteration
   * @param local_trajectory The local trajectory to pursue
   * @param local_goal Local goal constraints
   * @param local_solution solution plan in joint space
   * @return Feedback event from the current solver call i.e. "Collision detected"
   */
  virtual moveit_msgs::action::LocalPlanner::Feedback
  solve(const robot_trajectory::RobotTrajectory& local_trajectory,
        const std::shared_ptr<const moveit_msgs::action::LocalPlanner::Goal> local_goal,
        trajectory_msgs::msg::JointTrajectory& local_solution) = 0;

  /**
   * Reset local constraint solver to some user-defined initial state
   * @return True if reset was successful
   */
  virtual bool reset() = 0;
};
}  // namespace moveit::hybrid_planning
