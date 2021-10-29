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

#include <moveit/hybrid_planning_manager/hybrid_planning_interface.h>
#include <moveit/hybrid_planning_manager/hybrid_planning_events.h>

using namespace std::chrono_literals;

namespace moveit::hybrid_planning
{
namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("hybrid_planning_manager");
}

HybridPlanningInterface::HybridPlanningInterface(const rclcpp::Node::SharedPtr& node,
                                                 const std::shared_ptr<PlannerLogicInterface>& planning_logic_)
{
  node_ = node;

  // Initialize local planning action client
  local_planner_action_client_ =
      rclcpp_action::create_client<moveit_msgs::action::LocalPlanner>(node_, "local_planning_action");
  if (!local_planner_action_client_->wait_for_action_server(2s))
  {
    throw std::runtime_error("Local planner action server not available after waiting");
  }

  // Initialize global planning action client
  global_planner_action_client_ =
      rclcpp_action::create_client<moveit_msgs::action::GlobalPlanner>(node_, "global_planning_action");
  if (!global_planner_action_client_->wait_for_action_server(2s))
  {
    throw std::runtime_error("Global planner action server not available after waiting");
  }

  // Initialize hybrid planning action server
  hybrid_planning_request_server_ = rclcpp_action::create_server<moveit_msgs::action::HybridPlanning>(
      node_, "run_hybrid_planning",
      [](const rclcpp_action::GoalUUID& /*unused*/,
         std::shared_ptr<const moveit_msgs::action::HybridPlanning::Goal> /*unused*/) {
        RCLCPP_INFO(LOGGER, "Received goal request");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::HybridPlanning>>& /*unused*/) {
        RCLCPP_INFO(LOGGER, "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      std::bind(&HybridPlanningInterface::hybridPlanningRequestCallback, this, std::placeholders::_1));

  // Initialize global solution subscriber
  global_solution_sub_ = node_->create_subscription<moveit_msgs::msg::MotionPlanResponse>(
      "global_trajectory", 1, [this](const moveit_msgs::msg::MotionPlanResponse::SharedPtr msg) {
        // Reaction result from the latest event
        ReactionResult reaction_result = planner_logic_->processEvent(HybridPlanningEvent::GLOBAL_SOLUTION_AVAILABLE);
        if (reaction_result.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
          auto result = std::make_shared<moveit_msgs::action::HybridPlanning::Result>();
          result->error_code.val = reaction_result.error_code.val;
          result->error_message = reaction_result.error_message;
          hybrid_planning_goal_handle_->abort(result);
          RCLCPP_ERROR(LOGGER, "Hybrid Planning Manager failed to react to  '%s'", reaction_result.event.c_str());
        }
      });
}

bool HybridPlanningInterface::sendGlobalPlannerAction()
{
  auto global_goal_options = rclcpp_action::Client<moveit_msgs::action::GlobalPlanner>::SendGoalOptions();

  // Add goal response callback
  global_goal_options.goal_response_callback =
      [this](std::shared_future<rclcpp_action::ClientGoalHandle<moveit_msgs::action::GlobalPlanner>::SharedPtr> future) {
        auto const& goal_handle = future.get();
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
  // Add result callback
  global_goal_options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::GlobalPlanner>::WrappedResult& global_result) {
        // Reaction result from the latest event
        ReactionResult reaction_result =
            planner_logic_->processEvent(HybridPlanningEvent::GLOBAL_PLANNING_ACTION_FINISHED);
        auto result = std::make_shared<moveit_msgs::action::HybridPlanning::Result>();
        if (reaction_result.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
          auto result = std::make_shared<moveit_msgs::action::HybridPlanning::Result>();
          result->error_code.val = reaction_result.error_code.val;
          result->error_message = reaction_result.error_message;
          hybrid_planning_goal_handle_->abort(result);
          RCLCPP_ERROR(LOGGER, "Hybrid Planning Manager failed to react to  '%s'", reaction_result.event.c_str());
        }
      };

  // Forward global trajectory goal from hybrid planning request TODO(sjahr) pass goal as function argument
  auto global_goal_msg = moveit_msgs::action::GlobalPlanner::Goal();
  global_goal_msg.motion_sequence =
      (hybrid_planning_goal_handle_->get_goal())->motion_sequence;  // latest desired motion sequence
  global_goal_msg.planning_group = (hybrid_planning_goal_handle_->get_goal())->planning_group;  // planning_group_;
  // Send global planning goal and wait until it's accepted
  auto goal_handle_future = global_planner_action_client_->async_send_goal(global_goal_msg, global_goal_options);
  return true;  // return always success TODO(sjahr) add more error checking
};

bool HybridPlanningInterface::sendLocalPlannerAction()
{
  // Setup empty dummy goal (Global trajectory is subscribed by the local planner) TODO(sjahr) pass goal as function argument
  auto local_goal_msg = moveit_msgs::action::LocalPlanner::Goal();
  auto local_goal_options = rclcpp_action::Client<moveit_msgs::action::LocalPlanner>::SendGoalOptions();
  rclcpp_action::ClientGoalHandle<moveit_msgs::action::LocalPlanner>::SharedPtr goal_handle;

  // Add goal response callback
  local_goal_options.goal_response_callback =
      [this](std::shared_future<rclcpp_action::ClientGoalHandle<moveit_msgs::action::LocalPlanner>::SharedPtr> future) {
        auto const& goal_handle = future.get();
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

  // Add feedback callback
  local_goal_options.feedback_callback =
      [this](rclcpp_action::ClientGoalHandle<moveit_msgs::action::LocalPlanner>::SharedPtr /*unused*/,
             const std::shared_ptr<const moveit_msgs::action::LocalPlanner::Feedback> local_planner_feedback) {
        // Reaction result from the latest event
        ReactionResult reaction_result = planner_logic_->processEvent(local_planner_feedback->feedback);
        if (reaction_result.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
          auto result = std::make_shared<moveit_msgs::action::HybridPlanning::Result>();
          result->error_code.val = reaction_result.error_code.val;
          result->error_message = reaction_result.error_message;
          hybrid_planning_goal_handle_->abort(result);
          RCLCPP_ERROR(LOGGER, "Hybrid Planning Manager failed to react to  '%s'", reaction_result.event.c_str());
        }
      };

  // Add result callback to print the result
  local_goal_options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::LocalPlanner>::WrappedResult& result) {
        // Reaction result from the latest event
        ReactionResult reaction_result =
            planner_logic_->processEvent(HybridPlanningEvent::LOCAL_PLANNING_ACTION_FINISHED);
        if (reaction_result.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
          auto result = std::make_shared<moveit_msgs::action::HybridPlanning::Result>();
          result->error_code.val = reaction_result.error_code.val;
          result->error_message = reaction_result.error_message;
          hybrid_planning_goal_handle_->abort(result);
          RCLCPP_ERROR(LOGGER, "Hybrid Planning Manager failed to react to  '%s'", reaction_result.event.c_str());
        }
      };

  // Send global planning goal
  auto goal_handle_future = local_planner_action_client_->async_send_goal(local_goal_msg, local_goal_options);
  return true;  // return always success TODO(sjahr) add more error checking
}

void HybridPlanningInterface::hybridPlanningRequestCallback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::HybridPlanning>> goal_handle)
{
  // Pass goal handle to class member
  hybrid_planning_goal_handle_ = std::move(goal_handle);

  // React on incoming planning request
  ReactionResult reaction_result = planner_logic_->processEvent(HybridPlanningEvent::HYBRID_PLANNING_REQUEST_RECEIVED);
  if (reaction_result.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    auto result = std::make_shared<moveit_msgs::action::HybridPlanning::Result>();
    result->error_code.val = reaction_result.error_code.val;
    result->error_message = reaction_result.error_message;
    hybrid_planning_goal_handle_->abort(result);
    RCLCPP_ERROR(LOGGER, "Hybrid Planning Manager failed to react to  '%s'", reaction_result.event.c_str());
  }
}

void HybridPlanningInterface::sendHybridPlanningResponse(bool success)
{
  // Return hybrid planning action result dependend on the function's argument
  auto result = std::make_shared<moveit_msgs::action::HybridPlanning::Result>();
  if (success)
  {
    result->error_code.val = 1;
    hybrid_planning_goal_handle_->succeed(result);
  }
  else
  {
    hybrid_planning_goal_handle_->abort(result);
  }
}
}  // namespace moveit::hybrid_planning
