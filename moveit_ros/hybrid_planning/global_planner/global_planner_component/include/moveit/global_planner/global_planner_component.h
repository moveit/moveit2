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
   Description: A global planner component node that is customizable through plugins.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <pluginlib/class_loader.hpp>

#include <moveit/global_planner/global_planner_interface.h>

#include <moveit_msgs/action/global_planner.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>

namespace moveit::hybrid_planning
{
// Component node containing the global planner
class GlobalPlannerComponent
{
public:
  /** \brief Constructor */
  GlobalPlannerComponent(const rclcpp::NodeOptions& options);

  /** \brief Destructor */
  ~GlobalPlannerComponent()
  {
    // Join the thread used for long-running callbacks
    if (long_callback_thread_.joinable())
    {
      long_callback_thread_.join();
    }
  }

  // This function is required to make this class a valid NodeClass
  // see https://docs.ros2.org/foxy/api/rclcpp_components/register__node__macro_8hpp.html
  // Skip linting due to unconventional function naming
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()  // NOLINT
  {
    return node_->get_node_base_interface();  // NOLINT
  }

private:
  std::shared_ptr<rclcpp::Node> node_;

  std::string planner_plugin_name_;

  // Global planner plugin loader
  std::unique_ptr<pluginlib::ClassLoader<GlobalPlannerInterface>> global_planner_plugin_loader_;

  // Global planner instance
  std::shared_ptr<GlobalPlannerInterface> global_planner_instance_;

  // Global planning request action server
  rclcpp_action::Server<moveit_msgs::action::GlobalPlanner>::SharedPtr global_planning_request_server_;

  // Global trajectory publisher
  rclcpp::Publisher<moveit_msgs::msg::MotionPlanResponse>::SharedPtr global_trajectory_pub_;

  // Goal callback for global planning request action server
  void globalPlanningRequestCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::GlobalPlanner>>& goal_handle);

  // Initialize planning scene monitor and load pipelines
  bool initializeGlobalPlanner();

  // This thread is used for long-running callbacks. It's a member so they do not go out of scope.
  std::thread long_callback_thread_;

  // A unique callback group, to avoid mixing callbacks with other action servers
  rclcpp::CallbackGroup::SharedPtr cb_group_;
};

}  // namespace moveit::hybrid_planning
