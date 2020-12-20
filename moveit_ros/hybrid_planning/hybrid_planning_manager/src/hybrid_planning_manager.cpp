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

#include <moveit/hybrid_planning_manager/hybrid_planning_manager.h>

#include <chrono>
#include <thread>

namespace moveit
{
namespace hybrid_planning
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("hybrid_planning_manager");

HybridPlanningManager::HybridPlanningManager(const rclcpp::NodeOptions& options)
  : Node("hybrid_planning_manager", options)
{
  // Initialize local planning action client
  local_planner_action_client_ =
      rclcpp_action::create_client<moveit_msgs::action::LocalPlanner>(this, "local_planning_action");
  if (!local_planner_action_client_->wait_for_action_server(std::chrono::seconds(2)))
  {
    const std::string error = "Local planner action server not available after waiting";
    RCLCPP_FATAL(LOGGER, error);
    throw std::runtime_error(error);
  }

  // Initialize global planning action client
  global_planner_action_client_ =
      rclcpp_action::create_client<moveit_msgs::action::GlobalPlanner>(this, "global_planning_action");
  if (!global_planner_action_client_->wait_for_action_server(std::chrono::seconds(2)))
  {
    const std::string error = "Global planner action server not available after waiting";
    RCLCPP_FATAL(LOGGER, error);
    throw std::runtime_error(error);
  }

  // Initialize hybrid planning action server
  hybrid_planning_request_server_ = rclcpp_action::create_server<moveit_msgs::action::HybridPlanning>(
      this->get_node_base_interface(), this->get_node_clock_interface(), this->get_node_logging_interface(),
      this->get_node_waitables_interface(), "run_hybrid_planning",
      [this](const rclcpp_action::GoalUUID& /*unused*/,
             std::shared_ptr<const moveit_msgs::action::HybridPlanning::Goal> /*unused*/) {
        RCLCPP_INFO(LOGGER, "Received goal request");
        state_ = hybrid_planning::HybridPlanningState::REQUEST_RECEIVED;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::HybridPlanning>>& /*unused*/) {
        RCLCPP_INFO(LOGGER, "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      std::bind(&HybridPlanningManager::runHybridPlanning, this, std::placeholders::_1));
  state_ = hybrid_planning::HybridPlanningState::READY;
}

int HybridPlanningManager::planGlobalTrajectory()
{
  auto send_goal_options = rclcpp_action::Client<moveit_msgs::action::GlobalPlanner>::SendGoalOptions();

  // Add goal response callback to print whether the goal is accepted or not
  send_goal_options.goal_response_callback =
      [this](std::shared_future<rclcpp_action::ClientGoalHandle<moveit_msgs::action::GlobalPlanner>::SharedPtr>
                 future) {
        auto goal_handle = future.get();
        auto planning_progress = std::make_shared<moveit_msgs::action::HybridPlanning::Feedback>();
        auto& feedback = planning_progress->feedback;
        if (!goal_handle)
        {
          feedback = "Global goal was rejected by server";
        }
        else
        {
          feedback = "Global goal accepted by server";
        }
        hybrid_planning_goal_handle_->publish_feedback(planning_progress);
      };

  // Add result callback to print the result
  send_goal_options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::GlobalPlanner>::WrappedResult& result) {
        auto planning_progress = std::make_shared<moveit_msgs::action::HybridPlanning::Feedback>();
        auto& feedback = planning_progress->feedback;
        switch (result.code)
        {
          case rclcpp_action::ResultCode::SUCCEEDED:
            feedback = "Global result received";
            break;
          case rclcpp_action::ResultCode::ABORTED:
            feedback = "Global goal was aborted";
            break;
          case rclcpp_action::ResultCode::CANCELED:
            feedback = "Global goal was canceled";
            break;
          default:
            feedback = "Unknown global result code";
            break;
        }
        hybrid_planning_goal_handle_->publish_feedback(planning_progress);
        state_ = hybrid_planning::HybridPlanningState::GLOBAL_PLAN_READY;
      };
  // Forward global trajectory goal
  auto goal_msg = moveit_msgs::action::GlobalPlanner::Goal();
  goal_msg.request = (hybrid_planning_goal_handle_->get_goal())->request;

  // Send global planning goal and wait until it's accepted
  auto goal_handle_future = global_planner_action_client_->async_send_goal(goal_msg, send_goal_options);
  return 1;  // return always success
}

int HybridPlanningManager::runLocalPlanner()
{
  // Setup empty dummy goal
  auto goal_msg = moveit_msgs::action::LocalPlanner::Goal();
  auto send_goal_options = rclcpp_action::Client<moveit_msgs::action::LocalPlanner>::SendGoalOptions();
  rclcpp_action::ClientGoalHandle<moveit_msgs::action::LocalPlanner>::SharedPtr goal_handle;

  // Add goal response callback to print whether the goal is accepted or not
  send_goal_options.goal_response_callback =
      [this](std::shared_future<rclcpp_action::ClientGoalHandle<moveit_msgs::action::LocalPlanner>::SharedPtr>
                 future) {
        auto goal_handle = future.get();
        auto planning_progress = std::make_shared<moveit_msgs::action::HybridPlanning::Feedback>();
        auto& feedback = planning_progress->feedback;
        if (!goal_handle)
        {
          feedback = "Local goal was rejected by server";
        }
        else
        {
          feedback = "Local goal accepted by server";
        }
        hybrid_planning_goal_handle_->publish_feedback(planning_progress);
      };

  // Add result callback to print the result
  send_goal_options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::LocalPlanner>::WrappedResult& result) {
        auto planning_progress = std::make_shared<moveit_msgs::action::HybridPlanning::Feedback>();
        auto& feedback = planning_progress->feedback;
        switch (result.code)
        {
          case rclcpp_action::ResultCode::SUCCEEDED:
            feedback = "Local result received";
            break;
          case rclcpp_action::ResultCode::ABORTED:
            feedback = "Local goal was aborted";
            break;
          case rclcpp_action::ResultCode::CANCELED:
            feedback = "Local goal was canceled";
            break;
          default:
            feedback = "Unknown local result code";
            break;
        }
        hybrid_planning_goal_handle_->publish_feedback(planning_progress);
        state_ = hybrid_planning::HybridPlanningState::FINISHED;
      };
  // Send global planning goal
  auto goal_handle_future = local_planner_action_client_->async_send_goal(goal_msg, send_goal_options);
  return 1;  // return always success
}

void HybridPlanningManager::runHybridPlanning(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::HybridPlanning>> goal_handle)
{
  hybrid_planning_goal_handle_ = std::move(goal_handle);

  // Start hybrid planning loop
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1),
                                   std::bind(&HybridPlanningManager::hybridPlanningLoop, this));
}

void HybridPlanningManager::hybridPlanningLoop()
{
  auto result = std::make_shared<moveit_msgs::action::HybridPlanning::Result>();
  switch (state_)
  {
    case hybrid_planning::HybridPlanningState::REQUEST_RECEIVED:
      if (!this->planGlobalTrajectory())
      {
        state_ = hybrid_planning::HybridPlanningState::ABORT;
        break;
      }
      else
        state_ = hybrid_planning::HybridPlanningState::GLOBAL_PLANNING_ACTIVE;
      break;
    case hybrid_planning::HybridPlanningState::GLOBAL_PLANNING_ACTIVE:
      break;  // wait till global planner found a solution
    case hybrid_planning::HybridPlanningState::GLOBAL_PLAN_READY:
      if (!this->runLocalPlanner())
      {
        state_ = hybrid_planning::HybridPlanningState::ABORT;
        break;
      }
      else
        state_ = hybrid_planning::HybridPlanningState::LOCAL_PLANNING_ACTIVE;
      break;
    case hybrid_planning::HybridPlanningState::LOCAL_PLANNING_ACTIVE:
      break;  // wait till local planner executed the trajectory
    case hybrid_planning::HybridPlanningState::FINISHED:
      result->error_code.val = 1;
      hybrid_planning_goal_handle_->succeed(result);
      timer_->cancel();
      state_ = hybrid_planning::HybridPlanningState::READY;
      break;
    case hybrid_planning::HybridPlanningState::ABORT:
      hybrid_planning_goal_handle_->abort(result);
      timer_->cancel();
      state_ = hybrid_planning::HybridPlanningState::READY;
      break;
    default:
      auto planning_progress = std::make_shared<moveit_msgs::action::HybridPlanning::Feedback>();
      auto& feedback = planning_progress->feedback;
      feedback = "Unknown hybrid planning manager state";
      hybrid_planning_goal_handle_->publish_feedback(planning_progress);
      state_ = hybrid_planning::HybridPlanningState::ABORT;
      break;
  }
}

}  // namespace hybrid_planning
}  // namespace moveit

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit::hybrid_planning::HybridPlanningManager)