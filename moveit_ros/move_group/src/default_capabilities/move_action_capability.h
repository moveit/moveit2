/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#pragma once

#include <moveit/move_group/move_group_capability.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <memory>

namespace move_group
{
using MGAction = moveit_msgs::action::MoveGroup;
using MGActionGoal = rclcpp_action::ServerGoalHandle<MGAction>;

class MoveGroupMoveAction : public MoveGroupCapability
{
public:
  MoveGroupMoveAction();

  void initialize() override;

private:
  void executeMoveCallback(std::shared_ptr<MGActionGoal> goal);
  void executeMoveCallbackPlanAndExecute(const std::shared_ptr<MGActionGoal>& goal,
                                         std::shared_ptr<MGAction::Result>& action_res);
  void executeMoveCallbackPlanOnly(const std::shared_ptr<MGActionGoal>& goal,
                                   std::shared_ptr<MGAction::Result>& action_res);

  void startMoveExecutionCallback();
  void startMoveLookCallback();
  void preemptMoveCallback();
  void setMoveState(MoveGroupState state, const std::shared_ptr<MGActionGoal>& goal);

  bool planUsingPlanningPipeline(const planning_interface::MotionPlanRequest& req,
                                 plan_execution::ExecutableMotionPlan& plan);

  std::shared_ptr<rclcpp_action::Server<MGAction>> execute_action_server_;

  MoveGroupState move_state_;
  bool preempt_requested_;
};
}  // namespace move_group
