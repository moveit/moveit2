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

#include <moveit/local_planner/local_planner_component.h>

namespace moveit_hybrid_planning
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("local_planner_component");

LocalPlannerComponent::LocalPlannerComponent(const rclcpp::NodeOptions& options)
  : Node("local_planner_component", options)
{
  // Initialize local planning request action server
  using namespace std::placeholders;
  local_planning_request_server_ = rclcpp_action::create_server<moveit_msgs::action::LocalPlanner>(
      this->get_node_base_interface(), this->get_node_clock_interface(), this->get_node_logging_interface(),
      this->get_node_waitables_interface(), "local_planning_action",
      [](const rclcpp_action::GoalUUID& /*unused*/,
         std::shared_ptr<const moveit_msgs::action::LocalPlanner::Goal> /*unused*/) {
        RCLCPP_INFO(LOGGER, "Received local planning goal request");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::LocalPlanner>>& /*unused*/) {
        RCLCPP_INFO(LOGGER, "Received request to cancel local planning goal");
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::LocalPlanner>> goal_handle) {
        local_planning_goal_handle_ = std::move(goal_handle);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1),
                                         std::bind(&LocalPlannerComponent::localPlanningLoop, this));
      });

  // Initialize global trajectory listener
  global_trajectory_sub_ = create_subscription<moveit_msgs::msg::MotionPlanResponse>(
      "global_trajectory", 1, [this](const moveit_msgs::msg::MotionPlanResponse::SharedPtr msg) {
        RCLCPP_INFO(LOGGER, "Received global trajectory");
        global_trajectory_ = msg->trajectory;
        global_trajectory_received_ = true;
      });

  trajectory_pub_ =
      this->create_publisher<trajectory_msgs::msg::JointTrajectory>("panda_arm_controller/joint_trajectory", 1);

  state_ = moveit_hybrid_planning::LocalPlannerState::READY;
}

void LocalPlannerComponent::localPlanningLoop()
{
  auto result = std::make_shared<moveit_msgs::action::LocalPlanner::Result>();
  switch (state_)
  {
    case moveit_hybrid_planning::LocalPlannerState::READY:
      state_ = moveit_hybrid_planning::LocalPlannerState::AWAIT_GLOBAL_TRAJECTORY;
      break;
    case moveit_hybrid_planning::LocalPlannerState::AWAIT_GLOBAL_TRAJECTORY:
      if (!global_trajectory_received_)
      {
        break;
      }
      else
        state_ = moveit_hybrid_planning::LocalPlannerState::LOCAL_PLANNING_ACTIVE;
      break;
    case moveit_hybrid_planning::LocalPlannerState::LOCAL_PLANNING_ACTIVE:
      RCLCPP_INFO(LOGGER, "Forward trajectory");

      trajectory_pub_->publish(global_trajectory_.joint_trajectory);

      local_planning_goal_handle_->succeed(result);
      state_ = moveit_hybrid_planning::LocalPlannerState::READY;
      timer_->cancel();
      break;
    default:
      local_planning_goal_handle_->abort(result);
      timer_->cancel();
      RCLCPP_ERROR(LOGGER, "Local planner somehow failed :(");
      state_ = moveit_hybrid_planning::LocalPlannerState::READY;
      break;
  }
};
}  // namespace moveit_hybrid_planning

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit_hybrid_planning::LocalPlannerComponent)