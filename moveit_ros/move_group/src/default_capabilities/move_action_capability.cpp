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

#include "move_action_capability.h"

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/utils/message_checks.h>
#include <moveit/move_group/capability_names.h>

namespace move_group
{
static const rclcpp::Logger LOGGER =
    rclcpp::get_logger("moveit_move_group_default_capabilities.move_action_capability");

MoveGroupMoveAction::MoveGroupMoveAction()
  : MoveGroupCapability("MoveAction"), move_state_(IDLE), preempt_requested_{ false }
{
}

void MoveGroupMoveAction::initialize()
{
  // start the move action server
  auto node = context_->moveit_cpp_->getNode();
  execute_action_server_ = rclcpp_action::create_server<MGAction>(
      node, MOVE_ACTION,
      [](const rclcpp_action::GoalUUID& /*unused*/, std::shared_ptr<const MGAction::Goal> /*unused*/) {
        RCLCPP_INFO(LOGGER, "Received request");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<MGActionGoal>& /*unused*/) {
        RCLCPP_INFO(LOGGER, "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<MGActionGoal> goal) { return executeMoveCallback(goal); });
}

void MoveGroupMoveAction::executeMoveCallback(std::shared_ptr<MGActionGoal> goal)
{
  RCLCPP_INFO(LOGGER, "executing..");
  setMoveState(PLANNING, goal);
  // before we start planning, ensure that we have the latest robot state received...
  auto node = context_->moveit_cpp_->getNode();
  context_->planning_scene_monitor_->waitForCurrentRobotState(node->get_clock()->now());
  context_->planning_scene_monitor_->updateFrameTransforms();

  auto action_res = std::make_shared<MGAction::Result>();
  if (goal->get_goal()->planning_options.plan_only || !context_->allow_trajectory_execution_)
  {
    if (!goal->get_goal()->planning_options.plan_only)
      RCLCPP_WARN(LOGGER, "This instance of MoveGroup is not allowed to execute trajectories "
                          "but the goal request has plan_only set to false. "
                          "Only a motion plan will be computed anyway.");
    executeMoveCallbackPlanOnly(goal, action_res);
  }
  else
    executeMoveCallbackPlanAndExecute(goal, action_res);

  bool planned_trajectory_empty = trajectory_processing::isTrajectoryEmpty(action_res->planned_trajectory);
  // @todo: Response messages
  RCLCPP_INFO_STREAM(LOGGER, getActionResultString(action_res->error_code, planned_trajectory_empty,
                                                   goal->get_goal()->planning_options.plan_only));
  if (action_res->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    goal->succeed(action_res);
  else if (action_res->error_code.val == moveit_msgs::msg::MoveItErrorCodes::PREEMPTED)
    goal->canceled(action_res);
  else
    goal->abort(action_res);

  setMoveState(IDLE, goal);
  preempt_requested_ = false;
}

void MoveGroupMoveAction::executeMoveCallbackPlanAndExecute(const std::shared_ptr<MGActionGoal>& goal,
                                                            std::shared_ptr<MGAction::Result>& action_res)
{
  RCLCPP_INFO(LOGGER, "Combined planning and execution request received for MoveGroup action. "
                      "Forwarding to planning and execution pipeline.");

  if (moveit::core::isEmpty(goal->get_goal()->planning_options.planning_scene_diff))
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(context_->planning_scene_monitor_);
    const moveit::core::RobotState& current_state = lscene->getCurrentState();

    // check to see if the desired constraints are already met
    for (std::size_t i = 0; i < goal->get_goal()->request.goal_constraints.size(); ++i)
      if (lscene->isStateConstrained(
              current_state, kinematic_constraints::mergeConstraints(goal->get_goal()->request.goal_constraints[i],
                                                                     goal->get_goal()->request.path_constraints)))
      {
        RCLCPP_INFO(LOGGER, "Goal constraints are already satisfied. No need to plan or execute any motions");
        action_res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
        return;
      }
  }

  plan_execution::PlanExecution::Options opt;

  const moveit_msgs::msg::MotionPlanRequest& motion_plan_request =
      moveit::core::isEmpty(goal->get_goal()->request.start_state) ? goal->get_goal()->request :
                                                                     clearRequestStartState(goal->get_goal()->request);
  const moveit_msgs::msg::PlanningScene& planning_scene_diff =
      moveit::core::isEmpty(goal->get_goal()->planning_options.planning_scene_diff.robot_state) ?
          goal->get_goal()->planning_options.planning_scene_diff :
          clearSceneRobotState(goal->get_goal()->planning_options.planning_scene_diff);

  opt.replan_ = goal->get_goal()->planning_options.replan;
  opt.replan_attempts_ = goal->get_goal()->planning_options.replan_attempts;
  opt.replan_delay_ = goal->get_goal()->planning_options.replan_delay;
  opt.before_execution_callback_ = [this] { startMoveExecutionCallback(); };

  opt.plan_callback_ = [this, &motion_plan_request](plan_execution::ExecutableMotionPlan& plan) {
    return planUsingPlanningPipeline(motion_plan_request, plan);
  };

  plan_execution::ExecutableMotionPlan plan;
  if (preempt_requested_)
  {
    RCLCPP_INFO(LOGGER, "Preempt requested before the goal is planned and executed.");
    action_res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
    return;
  }

  context_->plan_execution_->planAndExecute(plan, planning_scene_diff, opt);

  convertToMsg(plan.plan_components_, action_res->trajectory_start, action_res->planned_trajectory);
  if (plan.executed_trajectory_)
    plan.executed_trajectory_->getRobotTrajectoryMsg(action_res->executed_trajectory);
  action_res->error_code = plan.error_code_;
}

void MoveGroupMoveAction::executeMoveCallbackPlanOnly(const std::shared_ptr<MGActionGoal>& goal,
                                                      std::shared_ptr<MGAction::Result>& action_res)
{
  RCLCPP_INFO(LOGGER, "Planning request received for MoveGroup action. Forwarding to planning pipeline.");

  // lock the scene so that it does not modify the world representation while diff() is called
  planning_scene_monitor::LockedPlanningSceneRO lscene(context_->planning_scene_monitor_);
  const planning_scene::PlanningSceneConstPtr& the_scene =
      (moveit::core::isEmpty(goal->get_goal()->planning_options.planning_scene_diff)) ?
          static_cast<const planning_scene::PlanningSceneConstPtr&>(lscene) :
          lscene->diff(goal->get_goal()->planning_options.planning_scene_diff);
  planning_interface::MotionPlanResponse res;

  if (preempt_requested_)
  {
    RCLCPP_INFO(LOGGER, "Preempt requested before the goal is planned.");
    action_res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
    return;
  }

  // Select planning_pipeline to handle request
  const planning_pipeline::PlanningPipelinePtr planning_pipeline =
      resolvePlanningPipeline(goal->get_goal()->request.pipeline_id);
  if (!planning_pipeline)
  {
    action_res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return;
  }

  try
  {
    planning_pipeline->generatePlan(the_scene, goal->get_goal()->request, res);
  }
  catch (std::exception& ex)
  {
    RCLCPP_ERROR(LOGGER, "Planning pipeline threw an exception: %s", ex.what());
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  }

  convertToMsg(res.trajectory_, action_res->trajectory_start, action_res->planned_trajectory);
  action_res->error_code = res.error_code_;
  action_res->planning_time = res.planning_time_;
}

bool MoveGroupMoveAction::planUsingPlanningPipeline(const planning_interface::MotionPlanRequest& req,
                                                    plan_execution::ExecutableMotionPlan& plan)
{
  setMoveState(PLANNING, nullptr);

  bool solved = false;
  planning_interface::MotionPlanResponse res;

  // Select planning_pipeline to handle request
  const planning_pipeline::PlanningPipelinePtr planning_pipeline = resolvePlanningPipeline(req.pipeline_id);
  if (!planning_pipeline)
  {
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return solved;
  }

  planning_scene_monitor::LockedPlanningSceneRO lscene(plan.planning_scene_monitor_);
  try
  {
    solved = planning_pipeline->generatePlan(plan.planning_scene_, req, res);
  }
  catch (std::exception& ex)
  {
    RCLCPP_ERROR(LOGGER, "Planning pipeline threw an exception: %s", ex.what());
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  }
  if (res.trajectory_)
  {
    plan.plan_components_.resize(1);
    plan.plan_components_[0].trajectory_ = res.trajectory_;
    plan.plan_components_[0].description_ = "plan";
  }
  plan.error_code_ = res.error_code_;

  return solved;
}

void MoveGroupMoveAction::startMoveExecutionCallback()
{
  setMoveState(MONITOR, nullptr);
}

void MoveGroupMoveAction::startMoveLookCallback()
{
  setMoveState(LOOK, nullptr);
}

void MoveGroupMoveAction::preemptMoveCallback()
{
  preempt_requested_ = true;
  context_->plan_execution_->stop();
}

void MoveGroupMoveAction::setMoveState(MoveGroupState state, const std::shared_ptr<MGActionGoal>& goal)
{
  move_state_ = state;

  if (goal)
  {
    auto move_feedback = std::make_shared<MGAction::Feedback>();
    move_feedback->state = stateToStr(state);
    goal->publish_feedback(move_feedback);
  }
}
}  // namespace move_group

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(move_group::MoveGroupMoveAction, move_group::MoveGroupCapability)
