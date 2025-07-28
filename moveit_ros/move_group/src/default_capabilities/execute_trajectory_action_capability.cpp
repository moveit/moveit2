/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Kentaro Wada.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Kentaro Wada */

#include "execute_trajectory_action_capability.hpp"

#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <moveit/plan_execution/plan_execution.hpp>
#include <moveit/trajectory_processing/trajectory_tools.hpp>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit/move_group/capability_names.hpp>
#include <moveit/utils/logger.hpp>

namespace move_group
{
namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.ros.move_group.clear_octomap_service");
}
}  // namespace

MoveGroupExecuteTrajectoryAction::MoveGroupExecuteTrajectoryAction() : MoveGroupCapability("execute_trajectory_action")
{
}

MoveGroupExecuteTrajectoryAction::~MoveGroupExecuteTrajectoryAction()
{
  callback_executor_.cancel();

  if (callback_thread_.joinable())
    callback_thread_.join();
}

void MoveGroupExecuteTrajectoryAction::initialize()
{
  auto node = context_->moveit_cpp_->getNode();
  callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,
                                                false /* don't spin with node executor */);
  callback_executor_.add_callback_group(callback_group_, node->get_node_base_interface());
  callback_thread_ = std::thread([this]() { callback_executor_.spin(); });
  // start the move action server
  execute_action_server_ = rclcpp_action::create_server<ExecTrajectory>(
      node->get_node_base_interface(), node->get_node_clock_interface(), node->get_node_logging_interface(),
      node->get_node_waitables_interface(), EXECUTE_ACTION_NAME,
      [](const rclcpp_action::GoalUUID& /*unused*/, const std::shared_ptr<const ExecTrajectory::Goal>& /*unused*/) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<ExecTrajectoryGoal>& /* unused */) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](const auto& goal) { executePathCallback(goal); }, rcl_action_server_get_default_options(),
      callback_group_);
}

void MoveGroupExecuteTrajectoryAction::executePathCallback(const std::shared_ptr<ExecTrajectoryGoal>& goal)
{
  auto action_res = std::make_shared<ExecTrajectory::Result>();
  if (!context_->trajectory_execution_manager_)
  {
    const std::string response = "Cannot execute trajectory since ~allow_trajectory_execution was set to false";
    action_res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
    goal->abort(action_res);
    return;
  }

  executePath(goal, action_res);

  const std::string response = getActionResultString(action_res->error_code, false, false);
  auto fb = std::make_shared<ExecTrajectory::Feedback>();
  fb->state = response;
  if (action_res->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    goal->publish_feedback(fb);
    goal->succeed(action_res);
  }
  else
  {
    goal->publish_feedback(fb);
    goal->abort(action_res);
  }

  setExecuteTrajectoryState(IDLE, goal);
}

void MoveGroupExecuteTrajectoryAction::executePath(const std::shared_ptr<ExecTrajectoryGoal>& goal,
                                                   std::shared_ptr<ExecTrajectory::Result>& action_res)
{
  RCLCPP_INFO(getLogger(), "Execution request received");

  if (context_->trajectory_execution_manager_->push(goal->get_goal()->trajectory, goal->get_goal()->controller_names))
  {
    setExecuteTrajectoryState(MONITOR, goal);
    context_->trajectory_execution_manager_->execute();
    moveit_controller_manager::ExecutionStatus status = context_->trajectory_execution_manager_->waitForExecution();
    if (status == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
    {
      action_res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    }
    else if (status == moveit_controller_manager::ExecutionStatus::PREEMPTED)
    {
      action_res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
    }
    else if (status == moveit_controller_manager::ExecutionStatus::TIMED_OUT)
    {
      action_res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT;
    }
    else
    {
      action_res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
    }
    RCLCPP_INFO_STREAM(getLogger(), "Execution completed: " << status.asString());
  }
  else
  {
    action_res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
  }
}

void MoveGroupExecuteTrajectoryAction::preemptExecuteTrajectoryCallback()
{
  context_->trajectory_execution_manager_->stopExecution(true);
}

void MoveGroupExecuteTrajectoryAction::setExecuteTrajectoryState(MoveGroupState state,
                                                                 const std::shared_ptr<ExecTrajectoryGoal>& goal)
{
  auto execute_feedback = std::make_shared<ExecTrajectory::Feedback>();
  execute_feedback->state = stateToStr(state);
  goal->publish_feedback(execute_feedback);
}

}  // namespace move_group

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(move_group::MoveGroupExecuteTrajectoryAction, move_group::MoveGroupCapability)
