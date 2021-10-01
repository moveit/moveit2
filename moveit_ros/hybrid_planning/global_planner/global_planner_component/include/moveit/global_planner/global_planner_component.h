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
   Description: Implementation of the Global Planner Component node which solves global motion planning requests as
   defined by the Global Planner Plugin and shares the solution with the other components.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <pluginlib/class_loader.hpp>

#include <moveit/global_planner/global_planner_interface.h>

#include <moveit_msgs/action/global_planner.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>

namespace moveit_hybrid_planning
{
// Component node containing the global planner
class GlobalPlannerComponent : public rclcpp::Node
{
public:
  GlobalPlannerComponent(const rclcpp::NodeOptions& options);

private:
  rclcpp::TimerBase::SharedPtr timer_;
  bool initialized_;

  std::string global_planner_name_;

  // Global planner plugin loader
  std::unique_ptr<pluginlib::ClassLoader<moveit_hybrid_planning::GlobalPlannerInterface>> global_planner_plugin_loader_;

  // Global planner instance
  std::shared_ptr<moveit_hybrid_planning::GlobalPlannerInterface> global_planner_instance_;

  moveit_msgs::msg::MotionPlanResponse last_global_solution_;

  // Global planning request action server
  rclcpp_action::Server<moveit_msgs::action::GlobalPlanner>::SharedPtr global_planning_request_server_;

  // Global trajectory publisher
  rclcpp::Publisher<moveit_msgs::msg::MotionPlanResponse>::SharedPtr global_trajectory_pub_;

  // Goal callback for global planning request action server
  void globalPlanningRequestCallback(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::GlobalPlanner>> goal_handle);

  // Initialize planning scene monitor and load pipelines
  bool init();
};
}  // namespace moveit_hybrid_planning
