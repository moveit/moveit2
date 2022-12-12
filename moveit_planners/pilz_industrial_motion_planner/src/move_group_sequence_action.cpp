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

// Modified by Pilz GmbH & Co. KG

#include <pilz_industrial_motion_planner/move_group_sequence_action.h>

#include <time.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/utils/message_checks.h>
#include <moveit/moveit_cpp/moveit_cpp.h>

#include <pilz_industrial_motion_planner/command_list_manager.h>
#include <pilz_industrial_motion_planner/trajectory_generation_exceptions.h>

namespace pilz_industrial_motion_planner
{
static const rclcpp::Logger LOGGER =
    rclcpp::get_logger("moveit.pilz_industrial_motion_planner.move_group_sequence_action");

MoveGroupSequenceAction::MoveGroupSequenceAction()
  : MoveGroupCapability("SequenceAction")
  , move_feedback_(std::make_shared<moveit_msgs::action::MoveGroupSequence::Feedback>())
{
}

void MoveGroupSequenceAction::initialize()
{
  // start the move action server
  RCLCPP_INFO_STREAM(LOGGER, "initialize move group sequence action");
  action_callback_group_ =
      context_->moveit_cpp_->getNode()->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  move_action_server_ = rclcpp_action::create_server<moveit_msgs::action::MoveGroupSequence>(
      context_->moveit_cpp_->getNode(), "sequence_move_group",
      [](const rclcpp_action::GoalUUID& /* unused */,
         const std::shared_ptr<const moveit_msgs::action::MoveGroupSequence::Goal>& /* unused */) {
        RCLCPP_DEBUG(LOGGER, "Received action goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<MoveGroupSequenceGoalHandle>& /* unused goal_handle */) {
        RCLCPP_DEBUG(LOGGER, "Canceling action goal");
        preemptMoveCallback();
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<MoveGroupSequenceGoalHandle>& goal_handle) {
        RCLCPP_DEBUG(LOGGER, "Accepting new action goal");
        executeSequenceCallback(goal_handle);
      },
      rcl_action_server_get_default_options(), action_callback_group_);

  command_list_manager_ = std::make_unique<pilz_industrial_motion_planner::CommandListManager>(
      context_->moveit_cpp_->getNode(), context_->planning_scene_monitor_->getRobotModel());
}

void MoveGroupSequenceAction::executeSequenceCallback(const std::shared_ptr<MoveGroupSequenceGoalHandle>& goal_handle)
{
  // Notify that goal is being executed
  goal_handle_ = goal_handle;
  const auto goal = goal_handle->get_goal();

  setMoveState(move_group::PLANNING);

  // Handle empty requests
  if (goal->request.items.empty())
  {
    RCLCPP_WARN(LOGGER, "Received empty request. That's ok but maybe not what you intended.");
    setMoveState(move_group::IDLE);
    const auto action_res = std::make_shared<moveit_msgs::action::MoveGroupSequence::Result>();
    action_res->response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    goal_handle->succeed(action_res);
    return;
  }

  // before we start planning, ensure that we have the latest robot state
  // received...
  auto node = context_->moveit_cpp_->getNode();
  context_->planning_scene_monitor_->waitForCurrentRobotState(node->get_clock()->now());
  context_->planning_scene_monitor_->updateFrameTransforms();

  const auto action_res = std::make_shared<moveit_msgs::action::MoveGroupSequence::Result>();
  if (goal->planning_options.plan_only || !context_->allow_trajectory_execution_)
  {
    if (!goal->planning_options.plan_only)
    {
      RCLCPP_WARN(LOGGER, "Only plan will be calculated, although plan_only == false.");  // LCOV_EXCL_LINE
    }
    executeMoveCallbackPlanOnly(goal, action_res);
  }
  else
  {
    executeSequenceCallbackPlanAndExecute(goal, action_res);
  }

  switch (action_res->response.error_code.val)
  {
    case moveit_msgs::msg::MoveItErrorCodes::SUCCESS:
      goal_handle->succeed(action_res);
      break;
    case moveit_msgs::msg::MoveItErrorCodes::PREEMPTED:
      goal_handle->canceled(action_res);
      break;
    default:
      goal_handle->abort(action_res);
      break;
  }

  setMoveState(move_group::IDLE);
  goal_handle_.reset();
}

void MoveGroupSequenceAction::executeSequenceCallbackPlanAndExecute(
    const moveit_msgs::action::MoveGroupSequence::Goal::ConstSharedPtr& goal,
    const moveit_msgs::action::MoveGroupSequence::Result::SharedPtr& action_res)
{
  RCLCPP_INFO(LOGGER, "Combined planning and execution request received for MoveGroupSequenceAction.");

  plan_execution::PlanExecution::Options opt;
  const moveit_msgs::msg::PlanningScene& planning_scene_diff =
      moveit::core::isEmpty(goal->planning_options.planning_scene_diff.robot_state) ?
          goal->planning_options.planning_scene_diff :
          clearSceneRobotState(goal->planning_options.planning_scene_diff);

  opt.replan_ = goal->planning_options.replan;
  opt.replan_attempts_ = goal->planning_options.replan_attempts;
  opt.replan_delay_ = goal->planning_options.replan_delay;
  opt.before_execution_callback_ = [this] { startMoveExecutionCallback(); };

  opt.plan_callback_ = [this, &request = goal->request](plan_execution::ExecutableMotionPlan& plan) {
    return planUsingSequenceManager(request, plan);
  };

  plan_execution::ExecutableMotionPlan plan;
  context_->plan_execution_->planAndExecute(plan, planning_scene_diff, opt);

  StartStatesMsg start_states_msg;
  convertToMsg(plan.plan_components_, start_states_msg, action_res->response.planned_trajectories);
  try
  {
    action_res->response.sequence_start = start_states_msg.at(0);
  }
  catch (std::out_of_range&)
  {
    RCLCPP_WARN(LOGGER, "Can not determine start state from empty sequence.");
  }
  action_res->response.error_code = plan.error_code_;
}

void MoveGroupSequenceAction::convertToMsg(const ExecutableTrajs& trajs, StartStatesMsg& start_states_msg,
                                           PlannedTrajMsgs& planned_trajs_msgs)
{
  start_states_msg.resize(trajs.size());
  planned_trajs_msgs.resize(trajs.size());
  for (size_t i = 0; i < trajs.size(); ++i)
  {
    moveit::core::robotStateToRobotStateMsg(trajs.at(i).trajectory_->getFirstWayPoint(), start_states_msg.at(i));
    trajs.at(i).trajectory_->getRobotTrajectoryMsg(planned_trajs_msgs.at(i));
  }
}

void MoveGroupSequenceAction::executeMoveCallbackPlanOnly(
    const moveit_msgs::action::MoveGroupSequence::Goal::ConstSharedPtr& goal,
    const moveit_msgs::action::MoveGroupSequence::Result::SharedPtr& action_res)
{
  RCLCPP_INFO(LOGGER, "Planning request received for MoveGroupSequenceAction action.");

  // lock the scene so that it does not modify the world representation while
  // diff() is called
  planning_scene_monitor::LockedPlanningSceneRO lscene(context_->planning_scene_monitor_);

  const planning_scene::PlanningSceneConstPtr& the_scene =
      (moveit::core::isEmpty(goal->planning_options.planning_scene_diff)) ?
          static_cast<const planning_scene::PlanningSceneConstPtr&>(lscene) :
          lscene->diff(goal->planning_options.planning_scene_diff);

  rclcpp::Time planning_start = context_->moveit_cpp_->getNode()->now();
  RobotTrajCont traj_vec;
  try
  {
    // Select planning_pipeline to handle request
    // All motions in the SequenceRequest need to use the same planning pipeline (but can use different planners)
    const planning_pipeline::PlanningPipelinePtr planning_pipeline =
        resolvePlanningPipeline(goal->request.items[0].req.pipeline_id);
    if (!planning_pipeline)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Could not load planning pipeline " << goal->request.items[0].req.pipeline_id);
      action_res->response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      return;
    }

    traj_vec = command_list_manager_->solve(the_scene, planning_pipeline, goal->request);
  }
  catch (const MoveItErrorCodeException& ex)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "> Planning pipeline threw an exception (error code: " << ex.getErrorCode()
                                                                                       << "): " << ex.what());
    action_res->response.error_code.val = ex.getErrorCode();
    return;
  }
  // LCOV_EXCL_START // Keep moveit up even if lower parts throw
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(LOGGER, "Planning pipeline threw an exception: %s", ex.what());
    action_res->response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return;
  }
  // LCOV_EXCL_STOP

  StartStatesMsg start_states_msg;
  start_states_msg.resize(traj_vec.size());
  action_res->response.planned_trajectories.resize(traj_vec.size());
  for (RobotTrajCont::size_type i = 0; i < traj_vec.size(); ++i)
  {
    move_group::MoveGroupCapability::convertToMsg(traj_vec.at(i), start_states_msg.at(i),
                                                  action_res->response.planned_trajectories.at(i));
  }
  try
  {
    action_res->response.sequence_start = start_states_msg.at(0);
  }
  catch (std::out_of_range&)
  {
    RCLCPP_WARN(LOGGER, "Can not determine start state from empty sequence.");
  }

  action_res->response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  action_res->response.planning_time = (context_->moveit_cpp_->getNode()->now() - planning_start).seconds();
}

bool MoveGroupSequenceAction::planUsingSequenceManager(const moveit_msgs::msg::MotionSequenceRequest& req,
                                                       plan_execution::ExecutableMotionPlan& plan)
{
  setMoveState(move_group::PLANNING);

  planning_scene_monitor::LockedPlanningSceneRO lscene(plan.planning_scene_monitor_);
  RobotTrajCont traj_vec;
  try
  {
    // Select planning_pipeline to handle request
    // All motions in the SequenceRequest need to use the same planning pipeline (but can use different planners)
    const planning_pipeline::PlanningPipelinePtr planning_pipeline =
        resolvePlanningPipeline(req.items[0].req.pipeline_id);
    if (!planning_pipeline)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Could not load planning pipeline " << req.items[0].req.pipeline_id);
      return false;
    }

    traj_vec = command_list_manager_->solve(plan.planning_scene_, planning_pipeline, req);
  }
  catch (const MoveItErrorCodeException& ex)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Planning pipeline threw an exception (error code: " << ex.getErrorCode()
                                                                                     << "): " << ex.what());
    plan.error_code_.val = ex.getErrorCode();
    return false;
  }
  // LCOV_EXCL_START // Keep MoveIt up even if lower parts throw
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Planning pipeline threw an exception: " << ex.what());
    plan.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return false;
  }
  // LCOV_EXCL_STOP

  if (!traj_vec.empty())
  {
    plan.plan_components_.resize(traj_vec.size());
    for (size_t i = 0; i < traj_vec.size(); ++i)
    {
      plan.plan_components_.at(i).trajectory_ = traj_vec.at(i);
      plan.plan_components_.at(i).description_ = "plan";
    }
  }
  plan.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return true;
}

void MoveGroupSequenceAction::startMoveExecutionCallback()
{
  setMoveState(move_group::MONITOR);
}

void MoveGroupSequenceAction::preemptMoveCallback()
{
  context_->plan_execution_->stop();
}

void MoveGroupSequenceAction::setMoveState(move_group::MoveGroupState state)
{
  move_state_ = state;
  move_feedback_->state = stateToStr(state);
  goal_handle_->publish_feedback(move_feedback_);
}

}  // namespace pilz_industrial_motion_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pilz_industrial_motion_planner::MoveGroupSequenceAction, move_group::MoveGroupCapability)
