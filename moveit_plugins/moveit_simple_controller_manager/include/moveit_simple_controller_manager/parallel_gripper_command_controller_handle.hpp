/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, Marq Rasmussen
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
 *   * Neither the name of the Marq Rasmussen nor the names of its
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
/* Design inspired by gripper_command_controller_handle.hpp */
/* Author: Marq Rasmussen */

#pragma once

#include <moveit_simple_controller_manager/action_based_controller_handle.hpp>
#include <control_msgs/action/parallel_gripper_command.hpp>
#include <set>

namespace moveit_simple_controller_manager
{
/*
 * This is an interface for a gripper using the control_msgs/ParallelGripperCommand action interface.
 */
class ParallelGripperCommandControllerHandle
  : public ActionBasedControllerHandle<control_msgs::action::ParallelGripperCommand>
{
public:
  /* Topics will map to name/ns/goal, name/ns/result, etc */
  ParallelGripperCommandControllerHandle(const rclcpp::Node::SharedPtr& node, const std::string& name,
                                         const std::string& ns, const double max_effort = 0.0,
                                         const double max_velocity = 0.0)
    : ActionBasedControllerHandle<control_msgs::action::ParallelGripperCommand>(
          node, name, ns, "moveit.simple_controller_manager.parallel_gripper_controller_handle")
    , max_effort_(max_effort)
    , max_velocity_(max_velocity)
  {
  }

  bool sendTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory) override
  {
    RCLCPP_DEBUG_STREAM(logger_, "Received new trajectory for " << name_);

    if (!controller_action_client_)
      return false;

    if (!isConnected())
    {
      RCLCPP_ERROR_STREAM(logger_, "Action client not connected to action server: " << getActionName());
      return false;
    }

    if (!trajectory.multi_dof_joint_trajectory.points.empty())
    {
      RCLCPP_ERROR(logger_, "%s cannot execute multi-dof trajectories.", name_.c_str());
      return false;
    }

    if (trajectory.joint_trajectory.points.empty())
    {
      RCLCPP_ERROR(logger_, "%s requires at least one joint trajectory point, but none received.", name_.c_str());
      return false;
    }

    if (trajectory.joint_trajectory.joint_names.empty())
    {
      RCLCPP_ERROR(logger_, "%s received a trajectory with no joint names specified.", name_.c_str());
      return false;
    }

    // goal to be sent
    control_msgs::action::ParallelGripperCommand::Goal goal;
    auto& cmd_state = goal.command;

    auto& joint_names = trajectory.joint_trajectory.joint_names;
    auto it = std::find(joint_names.begin(), joint_names.end(), command_joint_);
    if (it != joint_names.end())
    {
      cmd_state.name.push_back(command_joint_);
      std::size_t gripper_joint_index = std::distance(joint_names.begin(), it);
      // send last trajectory point
      if (trajectory.joint_trajectory.points.back().positions.size() <= gripper_joint_index)
      {
        RCLCPP_ERROR(logger_, "ParallelGripperCommand expects a joint trajectory with one \
                               point that specifies at least the position of joint \
                               '%s', but insufficient positions provided.",
                     trajectory.joint_trajectory.joint_names[gripper_joint_index].c_str());
        return false;
      }
      cmd_state.position.push_back(trajectory.joint_trajectory.points.back().positions[gripper_joint_index]);
      // only set the velocity or effort if the user has specified a positive non-zero max value
      if (max_velocity_ > 0.0)
      {
        cmd_state.velocity.push_back(max_velocity_);
      }
      if (max_effort_ > 0.0)
      {
        cmd_state.effort.push_back(max_effort_);
      }
    }
    else
    {
      RCLCPP_ERROR(logger_, "Received trajectory does not include a command for joint name %s.", command_joint_.c_str());
      return false;
    }

    rclcpp_action::Client<control_msgs::action::ParallelGripperCommand>::SendGoalOptions send_goal_options;
    // Active callback
    send_goal_options.goal_response_callback =
        [this](const rclcpp_action::Client<control_msgs::action::ParallelGripperCommand>::GoalHandle::SharedPtr&
               /* unused-arg */) { RCLCPP_DEBUG_STREAM(logger_, name_ << " started execution."); };
    // Send goal
    auto current_goal_future = controller_action_client_->async_send_goal(goal, send_goal_options);
    current_goal_ = current_goal_future.get();
    if (!current_goal_)
    {
      RCLCPP_ERROR(logger_, "%s goal was rejected by server.", name_.c_str());
      return false;
    }

    done_ = false;
    last_exec_ = moveit_controller_manager::ExecutionStatus::RUNNING;
    return true;
  }

  void setCommandJoint(const std::string& name)
  {
    command_joint_ = name;
  }

private:
  void controllerDoneCallback(
      const rclcpp_action::ClientGoalHandle<control_msgs::action::ParallelGripperCommand>::WrappedResult& wrapped_result)
      override
  {
    finishControllerExecution(wrapped_result.code);
  }

  /*
   * The ``max_effort`` used in the ParallelGripperCommand message.
   */
  double max_effort_ = 0.0;

  /*
   * The ``max_velocity_`` used in the ParallelGripperCommand message.
   */
  double max_velocity_ = 0.0;

  /*
   * The joint to command in the ParallelGripperCommand message
   */
  std::string command_joint_;
};  // namespace moveit_simple_controller_manager

}  // end namespace moveit_simple_controller_manager
