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
 */

#include <moveit/global_planner/global_planner_component.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/move_it_error_codes.h>

#include <chrono>
#include <thread>

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("global_planner_component");
}

namespace moveit_hybrid_planning
{
using namespace std::chrono_literals;
const std::string UNDEFINED = "<undefined>";

GlobalPlannerComponent::GlobalPlannerComponent(const rclcpp::NodeOptions& options)
  : Node("global_planner_component", options), initialized_(false)
{
  // Initialize global planner after construction
  // TODO(sjahr) Remove once life cycle component nodes are available
  timer_ = this->create_wall_timer(1ms, [this]() {
    if (initialized_)
    {
      timer_->cancel();
    }
    else
    {
      initialized_ = this->initialize();
      if (!initialized_)
      {
        timer_->cancel();
        throw std::runtime_error("Failed to initialize global planner");
      }
    }
  });
}

bool GlobalPlannerComponent::initialize()
{
  auto node_ptr = shared_from_this();

  // Initialize global planning request action server
  global_planning_request_server_ = rclcpp_action::create_server<moveit_msgs::action::GlobalPlanner>(
      this->get_node_base_interface(), this->get_node_clock_interface(), this->get_node_logging_interface(),
      this->get_node_waitables_interface(), "global_planning_action",
      [](const rclcpp_action::GoalUUID& /*unused*/,
         std::shared_ptr<const moveit_msgs::action::GlobalPlanner::Goal> /*unused*/) {
        RCLCPP_INFO(LOGGER, "Received global planning goal request");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::GlobalPlanner>>& /*unused*/) {
        RCLCPP_INFO(LOGGER, "Received request to cancel global planning goal");
        if (!global_planner_instance_->reset())
        {
          RCLCPP_ERROR(LOGGER, "Failed to reset the global planner while aborting current global planning");
        }
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      std::bind(&GlobalPlannerComponent::globalPlanningRequestCallback, this, std::placeholders::_1));

  global_trajectory_pub_ = this->create_publisher<moveit_msgs::msg::MotionPlanResponse>("global_trajectory", 1);

  // Load global planner plugin
  global_planner_name_ = this->declare_parameter<std::string>("global_planner_name", UNDEFINED);

  try
  {
    global_planner_plugin_loader_ =
        std::make_unique<pluginlib::ClassLoader<moveit_hybrid_planning::GlobalPlannerInterface>>(
            "moveit_hybrid_planning", "moveit_hybrid_planning::GlobalPlannerInterface");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(LOGGER, "Exception while creating global planner plugin loader: '%s'", ex.what());
    return false;
  }
  try
  {
    global_planner_instance_ = global_planner_plugin_loader_->createUniqueInstance(global_planner_name_);
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(LOGGER, "Exception while loading global planner '%s': '%s'", global_planner_name_.c_str(), ex.what());
    return false;
  }

  // Initialize global planner plugin
  if (!global_planner_instance_->initialize(node_ptr))
  {
    RCLCPP_ERROR(LOGGER, "Unable to initialize global planner plugin '%s'", global_planner_name_.c_str());
    return false;
  }
  RCLCPP_INFO(LOGGER, "Using global planner plugin '%s'", global_planner_name_.c_str());
  return true;
}

void GlobalPlannerComponent::globalPlanningRequestCallback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::GlobalPlanner>> goal_handle)
{
  // Plan global trajectory
  moveit_msgs::msg::MotionPlanResponse planning_solution = global_planner_instance_->plan(goal_handle);

  // Publish global planning solution to the local planner
  global_trajectory_pub_->publish(planning_solution);

  // Send action response
  auto result = std::make_shared<moveit_msgs::action::GlobalPlanner::Result>();
  result->response = planning_solution;
  goal_handle->succeed(result);

  // Save newest planning solution
  last_global_solution_ = planning_solution;  // TODO(sjahr) Add Service to expose this

  // Reset the global planner
  if (!global_planner_instance_->reset())
  {
    RCLCPP_ERROR(LOGGER, "Failed to reset the global planner while aborting current global planning");
  }
};
}  // namespace moveit_hybrid_planning

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit_hybrid_planning::GlobalPlannerComponent)
