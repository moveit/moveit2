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

#include "move_action_capability.hpp"

#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <moveit/planning_pipeline/planning_pipeline.hpp>
#include <moveit/plan_execution/plan_execution.hpp>
#include <moveit/trajectory_processing/trajectory_tools.hpp>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit/utils/message_checks.hpp>
#include <moveit/move_group/capability_names.hpp>
#include <moveit/utils/logger.hpp>

namespace move_group
{

namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.ros.move_group.move_action");
}
}  // namespace

MoveGroupMoveAction::MoveGroupMoveAction()
  : MoveGroupCapability("move_action"), move_state_(IDLE), preempt_requested_{ false }
{
}

MoveGroupMoveAction::~MoveGroupMoveAction()
{
  // A goal that is still being planned or executed keeps a pointer to this
  // capability, so stop it and wait for its worker before the members below are
  // destroyed. Workers that have not started yet notice shutting_down_ and
  // return without touching anything else.
  shutting_down_ = true;
  preemptMoveCallback();

  std::thread worker;
  {
    std::lock_guard<std::mutex> lock(goal_worker_mutex_);
    worker = std::move(goal_worker_);
  }

  if (worker.joinable())
  {
    worker.join();
  }
}

void MoveGroupMoveAction::initialize()
{
  // start the move action server
  auto node = context_->moveit_cpp_->getNode();
  execute_action_server_ = rclcpp_action::create_server<MGAction>(
      node, MOVE_ACTION,
      [](const rclcpp_action::GoalUUID& /*unused*/, const std::shared_ptr<const MGAction::Goal>& /*unused*/) {
        RCLCPP_INFO(getLogger(), "MoveGroupMoveAction: Received request");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<MGActionGoal>& /*unused*/) {
        RCLCPP_INFO(getLogger(), "MoveGroupMoveAction: Received request to cancel goal");
        preemptMoveCallback();
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<MGActionGoal>& goal) {
        // Runs in the executor thread. Hand the goal to a new worker and let that
        // worker wait for the goal that is still being planned or executed, so
        // that this callback does not keep the executor busy.
        std::lock_guard<std::mutex> lock(goal_worker_mutex_);
        if (shutting_down_)
        {
          // The destructor has already moved the previous worker out of
          // goal_worker_, so storing a new joinable thread here would make the
          // member destructor call std::terminate, and the new worker would
          // capture a this that is being destroyed. Reject the goal instead of
          // starting a worker that cannot be joined.
          auto result = std::make_shared<MGAction::Result>();
          result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
          goal->abort(result);
          return;
        }

        std::thread previous = std::move(goal_worker_);

        goal_worker_ = std::thread{ [this, previous = std::move(previous)](
                                        const std::shared_ptr<move_group::MGActionGoal>& goal) mutable {
                                     if (previous.joinable())
                                     {
                                       previous.join();
                                     }

                                     if (shutting_down_)
                                     {
                                       return;
                                     }

                                     // The cancellation callback ignores which goal it
                                     // refers to and stops the plan execution of the goal
                                     // that is running, so a goal that was canceled while it
                                     // waited here would otherwise plan and execute robot
                                     // motion after its client asked it to stop.
                                     if (goal->is_canceling())
                                     {
                                       auto result = std::make_shared<MGAction::Result>();
                                       result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
                                       goal->canceled(result);
                                       return;
                                     }

                                     executeMoveCallback(goal);
                                   },
                                    goal };
      });
}

void MoveGroupMoveAction::executeMoveCallback(const std::shared_ptr<MGActionGoal>& goal)
{
  goal_ = goal;
  RCLCPP_INFO(getLogger(), "executing..");
  setMoveState(PLANNING, goal_);
  // before we start planning, ensure that we have the latest robot state received...
  auto node = context_->moveit_cpp_->getNode();
  context_->planning_scene_monitor_->waitForCurrentRobotState(node->get_clock()->now());
  context_->planning_scene_monitor_->updateFrameTransforms();

  auto action_res = std::make_shared<MGAction::Result>();
  if (goal->get_goal()->planning_options.plan_only || !context_->allow_trajectory_execution_)
  {
    if (!goal->get_goal()->planning_options.plan_only)
    {
      RCLCPP_WARN(getLogger(), "This instance of MoveGroup is not allowed to execute trajectories "
                               "but the goal request has plan_only set to false. "
                               "Only a motion plan will be computed anyway.");
    }
    executeMoveCallbackPlanOnly(goal, action_res);
  }
  else
  {
    executeMoveCallbackPlanAndExecute(goal, action_res);
  }

  bool planned_trajectory_empty = trajectory_processing::isTrajectoryEmpty(action_res->planned_trajectory);
  // @todo: Response messages
  RCLCPP_INFO_STREAM(getLogger(), getActionResultString(action_res->error_code, planned_trajectory_empty,
                                                        goal->get_goal()->planning_options.plan_only));
  if (action_res->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    goal->succeed(action_res);
  }
  else if (action_res->error_code.val == moveit_msgs::msg::MoveItErrorCodes::PREEMPTED)
  {
    // A PREEMPTED result does not imply that the client asked to cancel this goal: the
    // destructor and preemptMoveCallback() also set preempt_requested_, and the goal is
    // then still in the EXECUTING state. canceled() only accepts a goal that is
    // canceling and throws otherwise, so abort such a goal instead.
    if (goal->is_canceling())
    {
      goal->canceled(action_res);
    }
    else
    {
      goal->abort(action_res);
    }
  }
  else
  {
    goal->abort(action_res);
  }

  setMoveState(IDLE, goal_);
  preempt_requested_ = false;
  goal_.reset();
}

void MoveGroupMoveAction::executeMoveCallbackPlanAndExecute(const std::shared_ptr<MGActionGoal>& goal,
                                                            std::shared_ptr<MGAction::Result>& action_res)
{
  RCLCPP_INFO(getLogger(), "Combined planning and execution request received for MoveGroup action. "
                           "Forwarding to planning and execution pipeline.");

  if (moveit::core::isEmpty(goal->get_goal()->planning_options.planning_scene_diff))
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(context_->planning_scene_monitor_);
    const moveit::core::RobotState& current_state = lscene->getCurrentState();

    // check to see if the desired constraints are already met
    for (std::size_t i = 0; i < goal->get_goal()->request.goal_constraints.size(); ++i)
    {
      if (lscene->isStateConstrained(
              current_state, kinematic_constraints::mergeConstraints(goal->get_goal()->request.goal_constraints[i],
                                                                     goal->get_goal()->request.path_constraints)))
      {
        RCLCPP_INFO(getLogger(), "Goal constraints are already satisfied. No need to plan or execute any motions");
        action_res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
        return;
      }
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

  opt.replan = goal->get_goal()->planning_options.replan;
  opt.replan_attemps = goal->get_goal()->planning_options.replan_attempts;
  opt.replan_delay = goal->get_goal()->planning_options.replan_delay;
  opt.before_execution_callback_ = [this] { startMoveExecutionCallback(); };

  opt.plan_callback = [this, &motion_plan_request](plan_execution::ExecutableMotionPlan& plan) {
    return planUsingPlanningPipeline(motion_plan_request, plan);
  };

  plan_execution::ExecutableMotionPlan plan;
  if (preempt_requested_)
  {
    RCLCPP_INFO(getLogger(), "Preempt requested before the goal is planned and executed.");
    action_res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
    return;
  }

  context_->plan_execution_->planAndExecute(plan, planning_scene_diff, opt);

  convertToMsg(plan.plan_components, action_res->trajectory_start, action_res->planned_trajectory);
  if (plan.executed_trajectory)
    plan.executed_trajectory->getRobotTrajectoryMsg(action_res->executed_trajectory);
  action_res->error_code = plan.error_code;
}

void MoveGroupMoveAction::executeMoveCallbackPlanOnly(const std::shared_ptr<MGActionGoal>& goal,
                                                      std::shared_ptr<MGAction::Result>& action_res)
{
  RCLCPP_INFO(getLogger(), "Planning request received for MoveGroup action. Forwarding to planning pipeline.");

  planning_interface::MotionPlanResponse res;

  if (preempt_requested_)
  {
    RCLCPP_INFO(getLogger(), "Preempt requested before the goal is planned.");
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
    auto scene =
        context_->planning_scene_monitor_->copyPlanningScene(goal->get_goal()->planning_options.planning_scene_diff);
    if (!planning_pipeline->generatePlan(scene, goal->get_goal()->request, res, context_->debug_))
    {
      RCLCPP_ERROR(getLogger(), "Generating a plan with planning pipeline failed.");
      res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    }
  }
  catch (std::exception& ex)
  {
    RCLCPP_ERROR(getLogger(), "Planning pipeline threw an exception: %s", ex.what());
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  }

  convertToMsg(res.trajectory, action_res->trajectory_start, action_res->planned_trajectory);
  action_res->error_code = res.error_code;
  action_res->planning_time = res.planning_time;
}

bool MoveGroupMoveAction::planUsingPlanningPipeline(const planning_interface::MotionPlanRequest& req,
                                                    plan_execution::ExecutableMotionPlan& plan)
{
  setMoveState(PLANNING, goal_);

  bool solved = false;
  planning_interface::MotionPlanResponse res;

  // Select planning_pipeline to handle request
  const planning_pipeline::PlanningPipelinePtr planning_pipeline = resolvePlanningPipeline(req.pipeline_id);
  if (!planning_pipeline)
  {
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return solved;
  }

  try
  {
    solved = planning_pipeline->generatePlan(plan.copyPlanningScene(), req, res, context_->debug_);
  }
  catch (std::exception& ex)
  {
    RCLCPP_ERROR(getLogger(), "Planning pipeline threw an exception: %s", ex.what());
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  }
  if (res.trajectory)
  {
    plan.plan_components.resize(1);
    plan.plan_components[0].trajectory = res.trajectory;
    plan.plan_components[0].description = "plan";
  }
  plan.error_code = res.error_code;

  return solved;
}

void MoveGroupMoveAction::startMoveExecutionCallback()
{
  setMoveState(MONITOR, goal_);
}

void MoveGroupMoveAction::startMoveLookCallback()
{
  setMoveState(LOOK, goal_);
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
