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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit_msgs/action/operate_local_planner.hpp>
#include <moveit_msgs/action/plan_global_trajectory.hpp>
#include <moveit_msgs/action/run_hybrid_planning.hpp>

#include <moveit_msgs/msg/move_it_error_codes.hpp>

namespace hybrid_planning_action = moveit_msgs::action;

namespace hybrid_planning
{
// Hybrid planning manager component node
class HybridPlanningManager : public rclcpp::Node
{
public:
  HybridPlanningManager(const rclcpp::NodeOptions& options);

private:
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<rclcpp_action::ServerGoalHandle<hybrid_planning_action::RunHybridPlanning>>
      hybrid_planning_goal_handle_;
  std::shared_ptr<moveit_msgs::action::RunHybridPlanning_Feedback> hybrid_planning_progess_;

  bool global_planning_started_;
  bool local_planning_started_;

  bool global_planning_executed_;
  bool local_planning_executed_;

  bool abort_;

  // Planning request action clients
  rclcpp_action::Client<hybrid_planning_action::OperateLocalPlanner>::SharedPtr local_planner_action_client_;
  rclcpp_action::Client<hybrid_planning_action::PlanGlobalTrajectory>::SharedPtr global_planner_action_client_;

  // Hybrid planning request action server
  rclcpp_action::Server<hybrid_planning_action::RunHybridPlanning>::SharedPtr hybrid_planning_request_server_;

  // Hybrid planning goal callback for hybrid planning request server
  void runHybridPlanning(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<hybrid_planning_action::RunHybridPlanning>> goal_handle);

  void hybridPlanningLoop();

  // Request global planning
  int planGlobalTrajectory();

  // Start local planning and execution
  int runLocalPlanner();
};
}  // namespace hybrid_planning