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

#include <moveit/hybrid_planning/local_planner_component.h>

namespace hybrid_planning
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("local_planner_component");

LocalPlannerComponent::LocalPlannerComponent(const rclcpp::NodeOptions& options)
  : Node("local_planner_component", options)
{
  // Initialize local planning request action server
  using namespace std::placeholders;
  local_planning_request_server_ = rclcpp_action::create_server<hybrid_planning_action::OperateLocalPlanner>(
      this->get_node_base_interface(), this->get_node_clock_interface(), this->get_node_logging_interface(),
      this->get_node_waitables_interface(), "operate_local_planner",
      [](const rclcpp_action::GoalUUID& /*unused*/,
         std::shared_ptr<const hybrid_planning_action::OperateLocalPlanner::Goal> /*unused*/) {
        RCLCPP_INFO(LOGGER, "Received local planning goal request");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<hybrid_planning_action::OperateLocalPlanner>>& /*unused*/) {
        RCLCPP_INFO(LOGGER, "Received request to cancel local planning goal");
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      std::bind(&LocalPlannerComponent::localPlanningGoalCallback, this, _1));
}

void LocalPlannerComponent::localPlanningGoalCallback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<hybrid_planning_action::OperateLocalPlanner>> goal_handle)
{
  // Fake computation for dummy implementation
  auto result = std::make_shared<hybrid_planning_action::OperateLocalPlanner::Result>();
  RCLCPP_INFO(LOGGER, "Local planning dummy computation started");
  std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  RCLCPP_INFO(LOGGER, "Local planning dummy computation finished");
  goal_handle->succeed(result);
};
}  // namespace hybrid_planning

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hybrid_planning::LocalPlannerComponent)