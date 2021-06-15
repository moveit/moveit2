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
   Description: Simple local solver plugin that forwards the next waypoint of the sampled local trajectory.
   Additionally, it is possible to enable collision checking, which lets the robot stop in front of a collision object.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/local_planner/local_constraint_solver_interface.h>

namespace moveit_hybrid_planning
{
class ForwardTrajectory : public LocalConstraintSolverInterface
{
public:
  ForwardTrajectory(){};
  ~ForwardTrajectory() override{};
  bool initialize(const rclcpp::Node::SharedPtr& node,
                  const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                  const std::string& group_name) override;
  bool reset() override;

  moveit_msgs::action::LocalPlanner::Feedback
  solve(const robot_trajectory::RobotTrajectory& local_trajectory,
        const std::shared_ptr<const moveit_msgs::action::LocalPlanner::Goal> local_goal,
        trajectory_msgs::msg::JointTrajectory& local_solution) override;

private:
  rclcpp::Node::SharedPtr node_handle_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  bool path_invalidation_event_send_;  // Send path invalidation event only once
  bool stop_before_collision_;
};
}  // namespace moveit_hybrid_planning
