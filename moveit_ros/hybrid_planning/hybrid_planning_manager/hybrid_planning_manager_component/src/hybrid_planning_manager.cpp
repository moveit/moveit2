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

#include <moveit/hybrid_planning_manager/hybrid_planning_manager.h>
#include <moveit/hybrid_planning_manager/hybrid_planning_events.h>
#include <hp_manager_parameters.hpp>
#include <moveit/utils/logger.hpp>

namespace moveit::hybrid_planning
{
using namespace std::chrono_literals;

namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.hybrid_planning.manager");
}
}  // namespace

HybridPlanningManager::HybridPlanningManager(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("hybrid_planning_manager", options) }, stop_hybrid_planning_(false)
{
  // Load planning logic plugin
  try
  {
    planner_logic_plugin_loader_ = std::make_unique<pluginlib::ClassLoader<PlannerLogicInterface>>(
        "moveit_hybrid_planning", "moveit::hybrid_planning::PlannerLogicInterface");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(getLogger(), "Exception while creating planner logic plugin loader '%s'", ex.what());
  }

  auto param_listener = hp_manager_parameters::ParamListener(node_, "");
  const auto params = param_listener.get_params();
  try
  {
    planner_logic_instance_ = planner_logic_plugin_loader_->createUniqueInstance(params.planner_logic_plugin_name);
    if (!planner_logic_instance_->initialize())
    {
      throw std::runtime_error("Unable to initialize planner logic plugin");
    }
    RCLCPP_INFO(getLogger(), "Using planner logic interface '%s'", params.planner_logic_plugin_name.c_str());
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(getLogger(), "Exception while loading planner logic '%s': '%s'",
                 params.planner_logic_plugin_name.c_str(), ex.what());
    return;
  }

  // Initialize local planning action client
  local_planner_action_client_ =
      rclcpp_action::create_client<moveit_msgs::action::LocalPlanner>(node_, "local_planning_action");
  if (!local_planner_action_client_->wait_for_action_server(2s))
  {
    RCLCPP_ERROR(getLogger(), "Local planner action server not available after waiting");
    return;
  }

  // Initialize global planning action client
  global_planner_action_client_ =
      rclcpp_action::create_client<moveit_msgs::action::GlobalPlanner>(node_, "global_planning_action");
  if (!global_planner_action_client_->wait_for_action_server(2s))
  {
    RCLCPP_ERROR(getLogger(), "Global planner action server not available after waiting");
    return;
  }

  // Initialize hybrid planning action server
  cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  hybrid_planning_request_server_ = rclcpp_action::create_server<moveit_msgs::action::HybridPlanner>(
      node_, "run_hybrid_planning",
      // Goal callback
      [](const rclcpp_action::GoalUUID& /*unused*/,
         const std::shared_ptr<const moveit_msgs::action::HybridPlanner::Goal>& /*unused*/) {
        RCLCPP_INFO(getLogger(), "Received goal request");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      // Cancel callback
      [&](const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::HybridPlanner>>& /*unused*/) {
        cancelHybridManagerGoals();
        RCLCPP_INFO(getLogger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      // Execution callback
      [&](const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::HybridPlanner>>& goal_handle) {
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        if (long_callback_thread_.joinable())
        {
          cancelHybridManagerGoals();
        }
        long_callback_thread_ = std::thread(&HybridPlanningManager::executeHybridPlannerGoal, this, goal_handle);
      },
      rcl_action_server_get_default_options(), cb_group_);

  // Initialize global solution subscriber
  global_solution_sub_ = node_->create_subscription<moveit_msgs::msg::MotionPlanResponse>(
      "global_trajectory", rclcpp::SystemDefaultsQoS(),
      [&](const moveit_msgs::msg::MotionPlanResponse::ConstSharedPtr& /* unused */) {
        // react is defined in a hybrid_planning_manager plugin
        processReactionResult(planner_logic_instance_->react(HybridPlanningEvent::GLOBAL_SOLUTION_AVAILABLE));
      });
}

void HybridPlanningManager::cancelHybridManagerGoals() noexcept
{
  // Prevent any new global or local requests from going out
  stop_hybrid_planning_ = true;
  // Cancel local action
  local_planner_action_client_->async_cancel_all_goals();
  // Cancel global action
  global_planner_action_client_->async_cancel_all_goals();
  if (long_callback_thread_.joinable())
  {
    long_callback_thread_.join();
  }
}

void HybridPlanningManager::executeHybridPlannerGoal(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::HybridPlanner>> goal_handle)
{
  // Reset the "stop" flag if it was set previously
  stop_hybrid_planning_ = false;

  // Pass goal handle to class member
  hybrid_planning_goal_handle_ = std::move(goal_handle);

  // react is defined in a hybrid_planning_manager plugin
  processReactionResult(planner_logic_instance_->react(HybridPlanningEvent::HYBRID_PLANNING_REQUEST_RECEIVED));
}

bool HybridPlanningManager::sendGlobalPlannerAction()
{
  auto global_goal_options = rclcpp_action::Client<moveit_msgs::action::GlobalPlanner>::SendGoalOptions();

  // Add goal response callback
  global_goal_options.goal_response_callback =
      [&](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::GlobalPlanner>::SharedPtr& goal_handle) {
        auto planning_progress = std::make_shared<moveit_msgs::action::HybridPlanner::Feedback>();
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
      [&](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::GlobalPlanner>::WrappedResult& global_result) {
        // Reaction result from the latest event
        ReactionResult reaction_result =
            ReactionResult(HybridPlanningEvent::UNDEFINED, "", moveit_msgs::msg::MoveItErrorCodes::FAILURE);
        switch (global_result.code)
        {
          case rclcpp_action::ResultCode::SUCCEEDED:
            reaction_result = planner_logic_instance_->react(HybridPlanningEvent::GLOBAL_PLANNING_ACTION_SUCCESSFUL);
            break;
          case rclcpp_action::ResultCode::CANCELED:
            reaction_result = planner_logic_instance_->react(HybridPlanningEvent::GLOBAL_PLANNING_ACTION_CANCELED);
            break;
          case rclcpp_action::ResultCode::ABORTED:
            reaction_result = planner_logic_instance_->react(HybridPlanningEvent::GLOBAL_PLANNING_ACTION_ABORTED);
            break;
          default:
            break;
        }
        // Abort hybrid planning if reaction fails
        processReactionResult(reaction_result);
      };

  // Forward global trajectory goal from hybrid planning request TODO(sjahr) pass goal as function argument
  auto global_goal_msg = moveit_msgs::action::GlobalPlanner::Goal();
  global_goal_msg.motion_sequence =
      (hybrid_planning_goal_handle_->get_goal())->motion_sequence;  // latest desired motion sequence;
  global_goal_msg.planning_group = (hybrid_planning_goal_handle_->get_goal())->planning_group;  // planning_group_;

  if (stop_hybrid_planning_)
  {
    return false;
  }

  // Send global planning goal and wait until it's accepted
  auto goal_handle_future = global_planner_action_client_->async_send_goal(global_goal_msg, global_goal_options);
  return true;  // return always success TODO(sjahr) add more error checking
};

bool HybridPlanningManager::sendLocalPlannerAction()
{
  // Setup empty dummy goal (Global trajectory is subscribed by the local planner) TODO(sjahr) pass goal as function argument
  auto local_goal_msg = moveit_msgs::action::LocalPlanner::Goal();
  auto local_goal_options = rclcpp_action::Client<moveit_msgs::action::LocalPlanner>::SendGoalOptions();
  rclcpp_action::ClientGoalHandle<moveit_msgs::action::LocalPlanner>::SharedPtr goal_handle;

  // Add goal response callback
  local_goal_options.goal_response_callback =
      [&](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::LocalPlanner>::SharedPtr& goal_handle) {
        auto planning_progress = std::make_shared<moveit_msgs::action::HybridPlanner::Feedback>();
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
      [&](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::LocalPlanner>::SharedPtr& /*unused*/,
          const std::shared_ptr<const moveit_msgs::action::LocalPlanner::Feedback>& local_planner_feedback) {
        processReactionResult(planner_logic_instance_->react(local_planner_feedback->feedback));
      };

  // Add result callback to print the result
  local_goal_options.result_callback =
      [&](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::LocalPlanner>::WrappedResult& local_result) {
        // Reaction result from the latest event
        ReactionResult reaction_result =
            ReactionResult(HybridPlanningEvent::UNDEFINED, "", moveit_msgs::msg::MoveItErrorCodes::FAILURE);
        switch (local_result.code)
        {
          case rclcpp_action::ResultCode::SUCCEEDED:
            reaction_result = planner_logic_instance_->react(HybridPlanningEvent::LOCAL_PLANNING_ACTION_SUCCESSFUL);
            break;
          case rclcpp_action::ResultCode::CANCELED:
            reaction_result = planner_logic_instance_->react(HybridPlanningEvent::LOCAL_PLANNING_ACTION_CANCELED);
            break;
          case rclcpp_action::ResultCode::ABORTED:
            reaction_result = planner_logic_instance_->react(HybridPlanningEvent::LOCAL_PLANNING_ACTION_ABORTED);
            break;
          default:
            break;
        }
        processReactionResult(reaction_result);
      };

  if (stop_hybrid_planning_)
  {
    return false;
  }

  // Send global planning goal
  auto goal_handle_future = local_planner_action_client_->async_send_goal(local_goal_msg, local_goal_options);
  return true;  // return always success TODO(sjahr) add more error checking
}

void HybridPlanningManager::sendHybridPlanningResponse(bool success)
{
  // Return hybrid planning action result dependent on the function's argument
  auto result = std::make_shared<moveit_msgs::action::HybridPlanner::Result>();
  if (success)
  {
    result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    hybrid_planning_goal_handle_->succeed(result);
  }
  else
  {
    result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    hybrid_planning_goal_handle_->abort(result);
  }
}

void HybridPlanningManager::processReactionResult(const ReactionResult& result)
{
  if (result.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    auto hp_result = std::make_shared<moveit_msgs::action::HybridPlanner::Result>();
    hp_result->error_code.val = result.error_code.val;
    hp_result->error_message = result.error_message;
    hybrid_planning_goal_handle_->abort(hp_result);
    RCLCPP_ERROR(getLogger(), "Hybrid Planning Manager failed to react to  '%s'", result.event.c_str());
    return;
  }

  switch (result.action)
  {
    case HybridPlanningAction::DO_NOTHING:
      // Do nothing
      break;
    case HybridPlanningAction::RETURN_HP_SUCCESS:
      sendHybridPlanningResponse(true);
      break;
    case HybridPlanningAction::RETURN_HP_FAILURE:
      sendHybridPlanningResponse(false);
      break;
    case HybridPlanningAction::SEND_GLOBAL_SOLVER_REQUEST:
      sendGlobalPlannerAction();
      break;
    case HybridPlanningAction::SEND_LOCAL_SOLVER_REQUEST:
      sendLocalPlannerAction();
      break;
    default:
      RCLCPP_ERROR(getLogger(), "Unknown reaction result code. No actions taken by the hybrid planning manager.");
  }
}
}  // namespace moveit::hybrid_planning

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit::hybrid_planning::HybridPlanningManager)
