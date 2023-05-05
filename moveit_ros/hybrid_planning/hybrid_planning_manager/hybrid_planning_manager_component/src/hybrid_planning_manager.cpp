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

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("hybrid_planning_manager");
}  // namespace

namespace moveit::hybrid_planning
{
using namespace std::chrono_literals;

HybridPlanningManager::HybridPlanningManager(const rclcpp::NodeOptions& options)
  : Node("hybrid_planning_manager", options), initialized_(false), stop_hybrid_planning_(false)
{
  // Initialize hybrid planning component after construction
  // TODO(sjahr) Remove once life cycle component nodes are available
  timer_ = create_wall_timer(1ms, [this]() {
    if (initialized_)
    {
      timer_->cancel();
    }
    else
    {
      if (!initialize())
      {
        const std::string error = "Failed to initialize global planner";
        timer_->cancel();
        throw std::runtime_error(error);
      }
      initialized_ = true;
    }
  });
}

bool HybridPlanningManager::initialize()
{
  // Load planning logic plugin
  try
  {
    planner_logic_plugin_loader_ = std::make_unique<pluginlib::ClassLoader<PlannerLogicInterface>>(
        "moveit_hybrid_planning", "moveit::hybrid_planning::PlannerLogicInterface");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(LOGGER, "Exception while creating planner logic plugin loader '%s'", ex.what());
  }
  // TODO(sjahr) Refactor parameter declaration and use repository wide solution
  std::string logic_plugin_name = "";
  if (has_parameter("planner_logic_plugin_name"))
  {
    get_parameter<std::string>("planner_logic_plugin_name", logic_plugin_name);
  }
  else
  {
    logic_plugin_name = declare_parameter<std::string>("planner_logic_plugin_name",
                                                       "moveit::hybrid_planning/ReplanInvalidatedTrajectory");
  }
  try
  {
    planner_logic_instance_ = planner_logic_plugin_loader_->createUniqueInstance(logic_plugin_name);
    if (!planner_logic_instance_->initialize(HybridPlanningManager::shared_from_this()))
    {
      throw std::runtime_error("Unable to initialize planner logic plugin");
    }
    RCLCPP_INFO(LOGGER, "Using planner logic interface '%s'", logic_plugin_name.c_str());
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(LOGGER, "Exception while loading planner logic '%s': '%s'", logic_plugin_name.c_str(), ex.what());
  }

  // Initialize local planning action client
  std::string local_planning_action_name = declare_parameter<std::string>("local_planning_action_name", "");
  get_parameter<std::string>("local_planning_action_name", local_planning_action_name);
  if (local_planning_action_name.empty())
  {
    RCLCPP_ERROR(LOGGER, "local_planning_action_name parameter was not defined");
    return false;
  }
  local_planner_action_client_ =
      rclcpp_action::create_client<moveit_msgs::action::LocalPlanner>(this, local_planning_action_name);
  if (!local_planner_action_client_->wait_for_action_server(2s))
  {
    RCLCPP_ERROR(LOGGER, "Local planner action server not available after waiting");
    return false;
  }

  // Initialize global planning action client
  std::string global_planning_action_name = declare_parameter<std::string>("global_planning_action_name", "");
  get_parameter<std::string>("global_planning_action_name", global_planning_action_name);
  if (global_planning_action_name.empty())
  {
    RCLCPP_ERROR(LOGGER, "global_planning_action_name parameter was not defined");
    return false;
  }
  global_planner_action_client_ =
      rclcpp_action::create_client<moveit_msgs::action::GlobalPlanner>(this, global_planning_action_name);
  if (!global_planner_action_client_->wait_for_action_server(2s))
  {
    RCLCPP_ERROR(LOGGER, "Global planner action server not available after waiting");
    return false;
  }

  // Initialize hybrid planning action server
  std::string hybrid_planning_action_name = declare_parameter<std::string>("hybrid_planning_action_name", "");
  get_parameter<std::string>("hybrid_planning_action_name", hybrid_planning_action_name);
  if (hybrid_planning_action_name.empty())
  {
    RCLCPP_ERROR(LOGGER, "hybrid_planning_action_name parameter was not defined");
    return false;
  }
  cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  hybrid_planning_request_server_ = rclcpp_action::create_server<moveit_msgs::action::HybridPlanner>(
      get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(),
      get_node_waitables_interface(), hybrid_planning_action_name,
      // Goal callback
      [](const rclcpp_action::GoalUUID& /*unused*/,
         const std::shared_ptr<const moveit_msgs::action::HybridPlanner::Goal>& /*unused*/) {
        RCLCPP_INFO(LOGGER, "Received goal request");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      // Cancel callback
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::HybridPlanner>>& /*unused*/) {
        cancelHybridManagerGoals();
        RCLCPP_INFO(LOGGER, "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      // Execution callback
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::HybridPlanner>> goal_handle) {
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        if (long_callback_thread_.joinable())
        {
          cancelHybridManagerGoals();
        }
        long_callback_thread_ = std::thread(&HybridPlanningManager::executeHybridPlannerGoal, this, goal_handle);
      },
      rcl_action_server_get_default_options(), cb_group_);

  // Initialize global solution subscriber
  global_solution_sub_ = create_subscription<moveit_msgs::msg::MotionPlanResponse>(
      "global_trajectory", rclcpp::SystemDefaultsQoS(),
      [this](const moveit_msgs::msg::MotionPlanResponse::ConstSharedPtr& /* unused */) {
        // react is defined in a hybrid_planning_manager plugin
        ReactionResult reaction_result = planner_logic_instance_->react(HybridPlanningEvent::GLOBAL_SOLUTION_AVAILABLE);
        if (reaction_result.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
          auto result = std::make_shared<moveit_msgs::action::HybridPlanner::Result>();
          result->error_code.val = reaction_result.error_code.val;
          result->error_message = reaction_result.error_message;
          hybrid_planning_goal_handle_->abort(result);
          RCLCPP_ERROR(LOGGER, "Hybrid Planning Manager failed to react to  '%s'", reaction_result.event.c_str());
        }
      });
  return true;
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
  ReactionResult reaction_result =
      planner_logic_instance_->react(HybridPlanningEvent::HYBRID_PLANNING_REQUEST_RECEIVED);
  if (reaction_result.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    auto result = std::make_shared<moveit_msgs::action::HybridPlanner::Result>();
    result->error_code.val = reaction_result.error_code.val;
    result->error_message = reaction_result.error_message;
    hybrid_planning_goal_handle_->abort(result);
    RCLCPP_ERROR_STREAM(LOGGER, "Hybrid Planning Manager failed to react to  " << reaction_result.event);
  }
}

bool HybridPlanningManager::sendGlobalPlannerAction()
{
  auto global_goal_options = rclcpp_action::Client<moveit_msgs::action::GlobalPlanner>::SendGoalOptions();

  // Add goal response callback
  global_goal_options.goal_response_callback =
      [this](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::GlobalPlanner>::SharedPtr& goal_handle) {
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
      [this](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::GlobalPlanner>::WrappedResult& global_result) {
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
        if (reaction_result.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
          auto result = std::make_shared<moveit_msgs::action::HybridPlanner::Result>();
          result->error_code.val = reaction_result.error_code.val;
          result->error_message = reaction_result.error_message;

          hybrid_planning_goal_handle_->abort(result);
          RCLCPP_ERROR(LOGGER, "Hybrid Planning Manager failed to react to  '%s'", reaction_result.event.c_str());
        }
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
      [this](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::LocalPlanner>::SharedPtr& goal_handle) {
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
      [this](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::LocalPlanner>::SharedPtr& /*unused*/,
             const std::shared_ptr<const moveit_msgs::action::LocalPlanner::Feedback>& local_planner_feedback) {
        // react is defined in a hybrid_planning_manager plugin
        ReactionResult reaction_result = planner_logic_instance_->react(local_planner_feedback->feedback);
        if (reaction_result.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
          auto result = std::make_shared<moveit_msgs::action::HybridPlanner::Result>();
          result->error_code.val = reaction_result.error_code.val;
          result->error_message = reaction_result.error_message;
          hybrid_planning_goal_handle_->abort(result);
          RCLCPP_ERROR(LOGGER, "Hybrid Planning Manager failed to react to  '%s'", reaction_result.event.c_str());
        }
      };

  // Add result callback to print the result
  local_goal_options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::LocalPlanner>::WrappedResult& local_result) {
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
        // Abort hybrid planning if reaction fails
        if (reaction_result.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
          const moveit_msgs::action::HybridPlanner::Result result([reaction_result]() {
            moveit_msgs::action::HybridPlanner::Result result;
            result.error_code.val = reaction_result.error_code.val;
            result.error_message = reaction_result.error_message;
            return result;
          }());
          hybrid_planning_goal_handle_->abort(std::make_shared<moveit_msgs::action::HybridPlanner::Result>(result));
          RCLCPP_ERROR_STREAM(LOGGER, "Hybrid Planning Manager failed to react to " << reaction_result.event);
        }
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
}  // namespace moveit::hybrid_planning

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit::hybrid_planning::HybridPlanningManager)
