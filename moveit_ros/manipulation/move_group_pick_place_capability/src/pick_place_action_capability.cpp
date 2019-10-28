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

#include "pick_place_action_capability.h"
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/plan_execution/plan_with_sensing.h>
#include <moveit/move_group_pick_place_capability/capability_names.h>

move_group::MoveGroupPickPlaceAction::MoveGroupPickPlaceAction()
  : MoveGroupCapability("PickPlaceAction"), pickup_state_(IDLE)
{
}

void move_group::MoveGroupPickPlaceAction::initialize(rclcpp::Node::SharedPtr& node)
{
  this->node_ = node;
  pick_place_.reset(new pick_place::PickPlace(context_->planning_pipeline_, node_));
  pick_place_->displayComputedMotionPlans(true);

  if (context_->debug_)
    pick_place_->displayProcessedGrasps(true);

  // start the pickup action server
  pickup_action_server_.reset();
  pickup_action_server_ = rclcpp_action::create_server<moveit_msgs::action::Pickup>(
      node_, PICKUP_ACTION, std::bind(&move_group::MoveGroupPickPlaceAction::pickup_handle_goal, this,
                                      std::placeholders::_1, std::placeholders::_2),
      std::bind(&move_group::MoveGroupPickPlaceAction::pickup_handle_cancel, this, std::placeholders::_1),
      std::bind(&move_group::MoveGroupPickPlaceAction::pickup_handle_accepted, this, std::placeholders::_1));

  // start the place action server
  place_action_server_.reset();
  place_action_server_ = rclcpp_action::create_server<moveit_msgs::action::Place>(
      node_, PLACE_ACTION, std::bind(&move_group::MoveGroupPickPlaceAction::place_handle_goal, this,
                                     std::placeholders::_1, std::placeholders::_2),
      std::bind(&move_group::MoveGroupPickPlaceAction::place_handle_cancel, this, std::placeholders::_1),
      std::bind(&move_group::MoveGroupPickPlaceAction::place_handle_accepted, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse move_group::MoveGroupPickPlaceAction::place_handle_goal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const moveit_msgs::action::Place::Goal> goal)
{
  (void)uuid;

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
move_group::MoveGroupPickPlaceAction::place_handle_cancel(const std::shared_ptr<GoalHandlePlace> goal_handle)
{
  RCLCPP_INFO(rclcpp::get_logger("pick_place_action_capability"), "Got request to cancel Place goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void move_group::MoveGroupPickPlaceAction::place_handle_accepted(const std::shared_ptr<GoalHandlePlace> goal_handle)
{
  std::thread(&move_group::MoveGroupPickPlaceAction::executePlaceCallback, this, goal_handle).detach();
}

rclcpp_action::GoalResponse move_group::MoveGroupPickPlaceAction::pickup_handle_goal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const moveit_msgs::action::Pickup::Goal> goal)
{
  (void)uuid;

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
move_group::MoveGroupPickPlaceAction::pickup_handle_cancel(const std::shared_ptr<GoalHandlePickup> goal_handle)
{
  RCLCPP_INFO(rclcpp::get_logger("pick_place_action_capability"), "Got request to cancel Pickup goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void move_group::MoveGroupPickPlaceAction::pickup_handle_accepted(const std::shared_ptr<GoalHandlePickup> goal_handle)
{
  std::thread(&move_group::MoveGroupPickPlaceAction::executePickupCallback, this, goal_handle).detach();
}

void move_group::MoveGroupPickPlaceAction::startPickupExecutionCallback(
    const std::shared_ptr<GoalHandlePickup> goal_handle)
{
  setPickupState(MONITOR, goal_handle);
}

void move_group::MoveGroupPickPlaceAction::startPickupLookCallback(const std::shared_ptr<GoalHandlePickup> goal_handle)
{
  setPickupState(LOOK, goal_handle);
}

void move_group::MoveGroupPickPlaceAction::startPlaceExecutionCallback(
    const std::shared_ptr<GoalHandlePlace> goal_handle)
{
  setPlaceState(MONITOR, goal_handle);
}

void move_group::MoveGroupPickPlaceAction::startPlaceLookCallback(const std::shared_ptr<GoalHandlePlace> goal_handle)
{
  setPlaceState(LOOK, goal_handle);
}

void move_group::MoveGroupPickPlaceAction::executePickupCallbackPlanOnly(
    const std::shared_ptr<GoalHandlePickup> goal_handle,
    std::shared_ptr<moveit_msgs::action::Pickup::Result>& action_res)
{
  pick_place::PickPlanPtr plan;
  const auto goal = goal_handle->get_goal();
  try
  {
    planning_scene_monitor::LockedPlanningSceneRO ps(context_->planning_scene_monitor_);
    plan = pick_place_->planPick(ps, *goal);
  }
  catch (std::exception& ex)
  {
    RCLCPP_ERROR(node_->get_logger(), "Pick&place threw an exception: %s", ex.what());
  }

  if (plan)
  {
    const std::vector<pick_place::ManipulationPlanPtr>& success = plan->getSuccessfulManipulationPlans();
    if (success.empty())
    {
      action_res->error_code = plan->getErrorCode();
    }
    else
    {
      const pick_place::ManipulationPlanPtr& result = success.back();
      convertToMsg(result->trajectories_, action_res->trajectory_start, action_res->trajectory_stages);
      action_res->trajectory_descriptions.resize(result->trajectories_.size());
      for (std::size_t i = 0; i < result->trajectories_.size(); ++i)
        action_res->trajectory_descriptions[i] = result->trajectories_[i].description_;
      if (result->id_ < goal->possible_grasps.size())
        action_res->grasp = goal->possible_grasps[result->id_];
      action_res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      action_res->planning_time = plan->getLastPlanTime();
    }
  }
  else
  {
    action_res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  }
}

void move_group::MoveGroupPickPlaceAction::executePlaceCallbackPlanOnly(
    const std::shared_ptr<const moveit_msgs::action::Place::Goal>& goal,
    std::shared_ptr<moveit_msgs::action::Place::Result>& action_res)
{
  pick_place::PlacePlanPtr plan;
  try
  {
    planning_scene_monitor::LockedPlanningSceneRO ps(context_->planning_scene_monitor_);
    plan = pick_place_->planPlace(ps, *goal);
  }
  catch (std::exception& ex)
  {
    RCLCPP_ERROR(node_->get_logger(), "Pick&place threw an exception: %s", ex.what());
  }

  if (plan)
  {
    const std::vector<pick_place::ManipulationPlanPtr>& success = plan->getSuccessfulManipulationPlans();
    if (success.empty())
    {
      action_res->error_code = plan->getErrorCode();
    }
    else
    {
      const pick_place::ManipulationPlanPtr& result = success.back();
      convertToMsg(result->trajectories_, action_res->trajectory_start, action_res->trajectory_stages);
      action_res->trajectory_descriptions.resize(result->trajectories_.size());
      for (std::size_t i = 0; i < result->trajectories_.size(); ++i)
        action_res->trajectory_descriptions[i] = result->trajectories_[i].description_;
      if (result->id_ < goal->place_locations.size())
        action_res->place_location = goal->place_locations[result->id_];
      action_res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      action_res->planning_time = plan->getLastPlanTime();
    }
  }
  else
  {
    action_res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  }
}

bool move_group::MoveGroupPickPlaceAction::planUsingPickPlacePickup(
    const std::shared_ptr<GoalHandlePickup> goal_handle,
    std::shared_ptr<moveit_msgs::action::Pickup::Result>& action_res, plan_execution::ExecutableMotionPlan& plan)
{
  setPickupState(PLANNING, goal_handle);
  const auto goal = goal_handle->get_goal();
  planning_scene_monitor::LockedPlanningSceneRO ps(plan.planning_scene_monitor_);

  pick_place::PickPlanPtr pick_plan;
  try
  {
    pick_plan = pick_place_->planPick(plan.planning_scene_, *goal.get());
  }
  catch (std::exception& ex)
  {
    RCLCPP_ERROR(node_->get_logger(), "Pick&place threw an exception: %s", ex.what());
  }

  if (pick_plan)
  {
    const std::vector<pick_place::ManipulationPlanPtr>& success = pick_plan->getSuccessfulManipulationPlans();
    if (success.empty())
    {
      plan.error_code_ = pick_plan->getErrorCode();
    }
    else
    {
      const pick_place::ManipulationPlanPtr& result = success.back();
      plan.plan_components_ = result->trajectories_;
      if (result->id_ < goal->possible_grasps.size())
        action_res->grasp = goal->possible_grasps[result->id_];
      plan.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      action_res->planning_time = pick_plan->getLastPlanTime();
    }
  }
  else
  {
    plan.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  }

  return plan.error_code_.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
}

bool move_group::MoveGroupPickPlaceAction::planUsingPickPlacePlace(
    const std::shared_ptr<GoalHandlePlace> goal_handle, std::shared_ptr<moveit_msgs::action::Place::Result>& action_res,
    plan_execution::ExecutableMotionPlan& plan)
{
  setPlaceState(PLANNING, goal_handle);
  const auto goal = goal_handle->get_goal();
  planning_scene_monitor::LockedPlanningSceneRO ps(plan.planning_scene_monitor_);

  pick_place::PlacePlanPtr place_plan;
  try
  {
    place_plan = pick_place_->planPlace(plan.planning_scene_, *goal.get());
  }
  catch (std::exception& ex)
  {
    RCLCPP_ERROR(node_->get_logger(), "Pick&place threw an exception: %s", ex.what());
  }

  if (place_plan)
  {
    const std::vector<pick_place::ManipulationPlanPtr>& success = place_plan->getSuccessfulManipulationPlans();
    if (success.empty())
    {
      plan.error_code_ = place_plan->getErrorCode();
    }
    else
    {
      const pick_place::ManipulationPlanPtr& result = success.back();
      plan.plan_components_ = result->trajectories_;
      if (result->id_ < goal->place_locations.size())
        action_res->place_location = goal->place_locations[result->id_];
      plan.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      action_res->planning_time = place_plan->getLastPlanTime();
    }
  }
  else
  {
    plan.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  }

  return plan.error_code_.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
}

void move_group::MoveGroupPickPlaceAction::executePickupCallbackPlanAndExecute(
    const std::shared_ptr<GoalHandlePickup> goal_handle,
    std::shared_ptr<moveit_msgs::action::Pickup::Result>& action_res)
{
  plan_execution::PlanExecution::Options opt;
  const auto goal = goal_handle->get_goal();
  opt.replan_ = goal->planning_options.replan;
  opt.replan_attempts_ = goal->planning_options.replan_attempts;
  opt.replan_delay_ = goal->planning_options.replan_delay;
  opt.before_execution_callback_ =
      std::bind(&MoveGroupPickPlaceAction::startPickupExecutionCallback, this, goal_handle);
  opt.plan_callback_ = std::bind(&MoveGroupPickPlaceAction::planUsingPickPlacePickup, this, std::cref(goal_handle),
                                 action_res, std::placeholders::_1);
  if (goal->planning_options.look_around && context_->plan_with_sensing_)
  {
    opt.plan_callback_ =
        std::bind(&plan_execution::PlanWithSensing::computePlan, context_->plan_with_sensing_.get(),
                  std::placeholders::_1, opt.plan_callback_, goal->planning_options.look_around_attempts,
                  goal->planning_options.max_safe_execution_cost);
    context_->plan_with_sensing_->setBeforeLookCallback(
        std::bind(&MoveGroupPickPlaceAction::startPickupLookCallback, this, goal_handle));
  }

  plan_execution::ExecutableMotionPlan plan;
  context_->plan_execution_->planAndExecute(plan, goal->planning_options.planning_scene_diff, opt);

  convertToMsg(plan.plan_components_, action_res->trajectory_start, action_res->trajectory_stages);
  action_res->trajectory_descriptions.resize(plan.plan_components_.size());
  for (std::size_t i = 0; i < plan.plan_components_.size(); ++i)
    action_res->trajectory_descriptions[i] = plan.plan_components_[i].description_;
  action_res->error_code = plan.error_code_;
}

void move_group::MoveGroupPickPlaceAction::executePlaceCallbackPlanAndExecute(
    const std::shared_ptr<GoalHandlePlace> goal_handle, std::shared_ptr<moveit_msgs::action::Place::Result>& action_res)
{
  plan_execution::PlanExecution::Options opt;
  const auto goal = goal_handle->get_goal();
  opt.replan_ = goal->planning_options.replan;
  opt.replan_attempts_ = goal->planning_options.replan_attempts;
  opt.replan_delay_ = goal->planning_options.replan_delay;
  opt.before_execution_callback_ = std::bind(&MoveGroupPickPlaceAction::startPlaceExecutionCallback, this, goal_handle);
  opt.plan_callback_ = std::bind(&MoveGroupPickPlaceAction::planUsingPickPlacePlace, this, std::cref(goal_handle),
                                 action_res, std::placeholders::_1);
  if (goal->planning_options.look_around && context_->plan_with_sensing_)
  {
    opt.plan_callback_ =
        std::bind(&plan_execution::PlanWithSensing::computePlan, context_->plan_with_sensing_.get(),
                  std::placeholders::_1, opt.plan_callback_, goal->planning_options.look_around_attempts,
                  goal->planning_options.max_safe_execution_cost);
    context_->plan_with_sensing_->setBeforeLookCallback(
        std::bind(&MoveGroupPickPlaceAction::startPlaceLookCallback, this, goal_handle));
  }

  plan_execution::ExecutableMotionPlan plan;
  context_->plan_execution_->planAndExecute(plan, goal->planning_options.planning_scene_diff, opt);

  convertToMsg(plan.plan_components_, action_res->trajectory_start, action_res->trajectory_stages);
  action_res->trajectory_descriptions.resize(plan.plan_components_.size());
  for (std::size_t i = 0; i < plan.plan_components_.size(); ++i)
    action_res->trajectory_descriptions[i] = plan.plan_components_[i].description_;
  action_res->error_code = plan.error_code_;
}

void move_group::MoveGroupPickPlaceAction::executePickupCallback(const std::shared_ptr<GoalHandlePickup> goal_handle)
{
  setPickupState(PLANNING, goal_handle);
  auto input_goal = goal_handle->get_goal();
  // before we start planning, ensure that we have the latest robot state received...
  context_->planning_scene_monitor_->waitForCurrentRobotState(rclcpp::Clock().now());
  context_->planning_scene_monitor_->updateFrameTransforms();

  std::shared_ptr<const moveit_msgs::action::Pickup::Goal> goal;
  if (input_goal->possible_grasps.empty())
  {
    moveit_msgs::action::Pickup::Goal* copy(new moveit_msgs::action::Pickup::Goal(*input_goal));
    goal.reset(copy);
    fillGrasps(*copy);
  }
  else
    goal = input_goal;

  auto action_res = std::make_shared<moveit_msgs::action::Pickup::Result>();

  if (goal->planning_options.plan_only || !context_->allow_trajectory_execution_)
  {
    if (!goal->planning_options.plan_only)
      RCLCPP_WARN(node_->get_logger(), "This instance of MoveGroup is not allowed to execute trajectories but the pick "
                                       "goal request has plan_only set to false. Only a motion plan will be computed "
                                       "anyway.");
    executePickupCallbackPlanOnly(goal_handle, action_res);
  }
  else
    executePickupCallbackPlanAndExecute(goal_handle, action_res);

  bool planned_trajectory_empty = action_res->trajectory_stages.empty();
  // std::string response =
  //     getActionResultString(action_res->error_code, planned_trajectory_empty, goal->planning_options.plan_only);
  if (action_res->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    goal_handle->succeed(action_res);
  else
  {  // TODO(anasarrak): Preempt for ros2?
     // if (action_res->error_code.val == moveit_msgs::msg::MoveItErrorCodes::PREEMPTED)
     //   pickup_action_server_->setPreempted(action_res, response);
     // else
    goal_handle->abort(action_res);
  }

  setPickupState(IDLE, goal_handle);
}

void move_group::MoveGroupPickPlaceAction::executePlaceCallback(const std::shared_ptr<GoalHandlePlace> goal_handle)
{
  setPlaceState(PLANNING, goal_handle);
  auto goal = goal_handle->get_goal();

  // before we start planning, ensure that we have the latest robot state received...
  context_->planning_scene_monitor_->waitForCurrentRobotState(rclcpp::Clock().now());
  context_->planning_scene_monitor_->updateFrameTransforms();

  auto action_res = std::make_shared<moveit_msgs::action::Place::Result>();

  if (goal->planning_options.plan_only || !context_->allow_trajectory_execution_)
  {
    if (!goal->planning_options.plan_only)
      RCLCPP_WARN(node_->get_logger(),
                  "This instance of MoveGroup is not allowed to execute trajectories but the place "
                  "goal request has plan_only set to false. Only a motion plan will be computed "
                  "anyway.");
    executePlaceCallbackPlanOnly(goal, action_res);
  }
  else
    executePlaceCallbackPlanAndExecute(goal_handle, action_res);

  bool planned_trajectory_empty = action_res->trajectory_stages.empty();
  // std::string response =
  //     getActionResultString(action_res->error_code, planned_trajectory_empty, goal->planning_options.plan_only);
  if (action_res->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    goal_handle->succeed(action_res);
  else
  {  // TODO(anasarrak): Preempt for ros2?
     // if (action_res->error_code.val == moveit_msgs::msg::MoveItErrorCodes::PREEMPTED)
     //   goal_handle->setPreempted(action_res);
     // else
    goal_handle->abort(action_res);
  }

  setPlaceState(IDLE, goal_handle);
}

void move_group::MoveGroupPickPlaceAction::preemptPickupCallback()
{
}

void move_group::MoveGroupPickPlaceAction::preemptPlaceCallback()
{
}

void move_group::MoveGroupPickPlaceAction::setPickupState(MoveGroupState state,
                                                          const std::shared_ptr<GoalHandlePickup> goal_handle)
{
  pickup_state_ = state;
  pickup_feedback_.state = stateToStr(state);
  auto pickup_feedback = std::make_shared<moveit_msgs::action::Pickup::Feedback>();
  goal_handle->publish_feedback(pickup_feedback);
}

void move_group::MoveGroupPickPlaceAction::setPlaceState(MoveGroupState state,
                                                         const std::shared_ptr<GoalHandlePlace> goal_handle)
{
  place_state_ = state;
  place_feedback_.state = stateToStr(state);
  auto place_feedback = std::make_shared<moveit_msgs::action::Place::Feedback>();
  goal_handle->publish_feedback(place_feedback);
}

void move_group::MoveGroupPickPlaceAction::fillGrasps(moveit_msgs::action::Pickup::Goal& goal)
{
  planning_scene_monitor::LockedPlanningSceneRO lscene(context_->planning_scene_monitor_);

  RCLCPP_DEBUG(node_->get_logger(), "Using default grasp poses");
  goal.minimize_object_distance = true;

  // add a number of default grasp points
  // \todo add more!
  moveit_msgs::msg::Grasp g;
  g.grasp_pose.header.frame_id = goal.target_name;
  g.grasp_pose.pose.position.x = -0.2;
  g.grasp_pose.pose.position.y = 0.0;
  g.grasp_pose.pose.position.z = 0.0;
  g.grasp_pose.pose.orientation.x = 0.0;
  g.grasp_pose.pose.orientation.y = 0.0;
  g.grasp_pose.pose.orientation.z = 0.0;
  g.grasp_pose.pose.orientation.w = 1.0;

  g.pre_grasp_approach.direction.header.frame_id = lscene->getPlanningFrame();
  g.pre_grasp_approach.direction.vector.x = 1.0;
  g.pre_grasp_approach.min_distance = 0.1;
  g.pre_grasp_approach.desired_distance = 0.2;

  g.post_grasp_retreat.direction.header.frame_id = lscene->getPlanningFrame();
  g.post_grasp_retreat.direction.vector.z = 1.0;
  g.post_grasp_retreat.min_distance = 0.1;
  g.post_grasp_retreat.desired_distance = 0.2;

  if (lscene->getRobotModel()->hasEndEffector(goal.end_effector))
  {
    g.pre_grasp_posture.joint_names = lscene->getRobotModel()->getEndEffector(goal.end_effector)->getJointModelNames();
    g.pre_grasp_posture.points.resize(1);
    g.pre_grasp_posture.points[0].positions.resize(g.pre_grasp_posture.joint_names.size(),
                                                   std::numeric_limits<double>::max());

    g.grasp_posture.joint_names = g.pre_grasp_posture.joint_names;
    g.grasp_posture.points.resize(1);
    g.grasp_posture.points[0].positions.resize(g.grasp_posture.joint_names.size(), -std::numeric_limits<double>::max());
  }
  goal.possible_grasps.push_back(g);
}

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupPickPlaceAction, move_group::MoveGroupCapability)
