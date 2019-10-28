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

#include "execute_trajectory_action_capability.h"

#include <moveit/plan_execution/plan_execution.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group/capability_names.h>

namespace move_group
{
MoveGroupExecuteTrajectoryAction::MoveGroupExecuteTrajectoryAction() : MoveGroupCapability("ExecuteTrajectoryAction")
{
}

void MoveGroupExecuteTrajectoryAction::initialize(std::shared_ptr<rclcpp::Node>& node)
{
  this->node_ = node;
  // start the move action server
  execute_action_server_.reset();
  execute_action_server_ = rclcpp_action::create_server<moveit_msgs::action::ExecuteTrajectory>(
      node_, EXECUTE_ACTION_NAME, std::bind(&move_group::MoveGroupExecuteTrajectoryAction::handle_trajectory_goal, this,
                                            std::placeholders::_1, std::placeholders::_2),
      std::bind(&move_group::MoveGroupExecuteTrajectoryAction::handle_trajectory_cancel, this, std::placeholders::_1),
      std::bind(&move_group::MoveGroupExecuteTrajectoryAction::handle_trajectory_accept, this, std::placeholders::_1));
  // TODO(anasarrak): Prempt for ros2 actions?
  // execute_action_server_->registerPreemptCallback(
  //     boost::bind(&MoveGroupExecuteTrajectoryAction::preemptExecuteTrajectoryCallback, this));
  // execute_action_server_->start();
}

rclcpp_action::GoalResponse move_group::MoveGroupExecuteTrajectoryAction::handle_trajectory_goal(
    const std::array<uint8_t, 16>& uuid, std::shared_ptr<const moveit_msgs::action::ExecuteTrajectory::Goal> goal)
{
  (void)uuid;
  // TODO (anasarrak): Add a REJECT
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse move_group::MoveGroupExecuteTrajectoryAction::handle_trajectory_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::ExecuteTrajectory>> goal_handle)
{
  RCLCPP_INFO(rclcpp::get_logger("execute_trajectory_action_capability"), "Got request to cancel goal");
  (void)goal_handle;

  return rclcpp_action::CancelResponse::ACCEPT;
}

void move_group::MoveGroupExecuteTrajectoryAction::handle_trajectory_accept(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::ExecuteTrajectory>> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread(&move_group::MoveGroupExecuteTrajectoryAction::executePathCallback, this, goal_handle).detach();
}

void MoveGroupExecuteTrajectoryAction::executePathCallback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::ExecuteTrajectory>> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto action_res = std::make_shared<moveit_msgs::action::ExecuteTrajectory::Result>();
  if (!context_->trajectory_execution_manager_)
  {
    const std::string response = "Cannot execute trajectory since ~allow_trajectory_execution was set to false";
    action_res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
    goal_handle->abort(action_res);
    return;
  }
  executePath(goal_handle, action_res);

  const std::string response = getActionResultString(action_res->error_code, false, false);
  if (action_res->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    goal_handle->succeed(action_res);
  }
  // TODO(anasarrak) not prempt for ros2 actions yet
  // else if (action_res->error_code.val == moveit_msgs::msg::MoveItErrorCodes::PREEMPTED)
  // {
  //   goal_handle->setPreempted(action_res, response);
  // }
  else
  {
    goal_handle->abort(action_res);
  }

  setExecuteTrajectoryState(IDLE, goal_handle);
}

void MoveGroupExecuteTrajectoryAction::executePath(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::ExecuteTrajectory>> goal_handle,
    std::shared_ptr<moveit_msgs::action::ExecuteTrajectory::Result> action_res)
{
  RCLCPP_INFO(node_->get_logger(), "Execution request received");
  const auto goal = goal_handle->get_goal();
  context_->trajectory_execution_manager_->clear();
  if (context_->trajectory_execution_manager_->push(goal->trajectory))
  {
    setExecuteTrajectoryState(MONITOR, goal_handle);
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
    RCLCPP_INFO(node_->get_logger(), "Execution completed: %s", status.asString().c_str());
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

void MoveGroupExecuteTrajectoryAction::setExecuteTrajectoryState(
    MoveGroupState state,
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::ExecuteTrajectory>> goal_handle)
{
  auto execute_feedback = std::make_shared<moveit_msgs::action::ExecuteTrajectory::Feedback>();
  execute_feedback->state = stateToStr(state);
  goal_handle->publish_feedback(execute_feedback);
}

}  // namespace move_group

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupExecuteTrajectoryAction, move_group::MoveGroupCapability)
