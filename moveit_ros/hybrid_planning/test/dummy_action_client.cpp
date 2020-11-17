#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <moveit/hybrid_planning/global_planner_component.h>
#include <moveit/hybrid_planning/hybrid_planning_manager.h>
#include <moveit/hybrid_planning/local_planner_component.h>

#include <moveit_msgs/action/run_hybrid_planning.hpp>
#include <moveit_msgs/action/plan_global_trajectory.hpp>

namespace hybrid_planning_action = moveit_msgs::action;

const rclcpp::Logger LOGGER = rclcpp::get_logger("dummy_hybrid_planning_client");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto node = rclcpp::Node::make_shared("dummy_hybrid_planning_client");
  auto hp_action_client =
      rclcpp_action::create_client<hybrid_planning_action::RunHybridPlanning>(node, "hybrid_planning_request");

  if (!hp_action_client->wait_for_action_server(std::chrono::seconds(20)))
  {
    RCLCPP_ERROR(LOGGER, "Hybrid planning action server not available after waiting");
    return 1;
  }

  // Setup dummy goal
  auto goal_msg = hybrid_planning_action::RunHybridPlanning::Goal();
  auto send_goal_options = rclcpp_action::Client<hybrid_planning_action::RunHybridPlanning>::SendGoalOptions();
  send_goal_options.feedback_callback =
      [](rclcpp_action::ClientGoalHandle<hybrid_planning_action::RunHybridPlanning>::SharedPtr,
         const std::shared_ptr<const hybrid_planning_action::RunHybridPlanning::Feedback> feedback) {
        RCLCPP_INFO(LOGGER, feedback->feedback);
      };

  RCLCPP_INFO(LOGGER, "Sending hybrid planning goal");
  // Ask server to achieve some goal and wait until it's accepted
  auto goal_handle_future = hp_action_client->async_send_goal(goal_msg, send_goal_options);
  if (rclcpp::spin_until_future_complete(node, goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "send goal call failed :(");
    return 1;
  }

  rclcpp_action::ClientGoalHandle<hybrid_planning_action::RunHybridPlanning>::SharedPtr goal_handle =
      goal_handle_future.get();
  if (!goal_handle)
  {
    RCLCPP_ERROR(LOGGER, "Goal was rejected by server");
    return 1;
  }

  // Wait for the server to be done with the goal
  auto result_future = hp_action_client->async_get_result(goal_handle);

  RCLCPP_INFO(LOGGER, "Waiting for hybrid planning result");
  if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "get result call failed :(");
    return 1;
  }

  rclcpp_action::ClientGoalHandle<hybrid_planning_action::RunHybridPlanning>::WrappedResult wrapped_result =
      result_future.get();

  switch (wrapped_result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(LOGGER, "Hybrid planning goal succeded");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(LOGGER, "Hybrid planning goal was aborted");
      return 1;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(LOGGER, "Hybrid planning goal was canceled");
      return 1;
    default:
      RCLCPP_ERROR(LOGGER, "Unknown hybrid planning result code");
      return 1;
  }

  RCLCPP_INFO(LOGGER, "Hybrid planning result received");

  rclcpp::shutdown();
  return 0;
}