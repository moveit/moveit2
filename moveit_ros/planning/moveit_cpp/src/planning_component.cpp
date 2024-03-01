/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik Inc.
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

/* Author: Henning Kayser */

#include <stdexcept>

#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <thread>
#include <moveit/utils/logger.hpp>

namespace moveit_cpp
{

PlanningComponent::PlanningComponent(const std::string& group_name, const MoveItCppPtr& moveit_cpp)
  : node_(moveit_cpp->getNode())
  , moveit_cpp_(moveit_cpp)
  , group_name_(group_name)
  , logger_(moveit::getLogger("moveit.ros.planning_component"))
{
  joint_model_group_ = moveit_cpp_->getRobotModel()->getJointModelGroup(group_name);
  if (!joint_model_group_)
  {
    std::string error = "Could not find joint model group '" + group_name + "'.";
    RCLCPP_FATAL_STREAM(logger_, error);
    throw std::runtime_error(error);
  }
}

PlanningComponent::PlanningComponent(const std::string& group_name, const rclcpp::Node::SharedPtr& node)
  : PlanningComponent(group_name, std::make_shared<MoveItCpp>(node))
{
  joint_model_group_ = moveit_cpp_->getRobotModel()->getJointModelGroup(group_name);
  if (!joint_model_group_)
  {
    std::string error = "Could not find joint model group '" + group_name + "'.";
    RCLCPP_FATAL_STREAM(logger_, error);
    throw std::runtime_error(error);
  }
}

const std::vector<std::string> PlanningComponent::getNamedTargetStates()
{
  if (joint_model_group_)
  {
    return joint_model_group_->getDefaultStateNames();
  }
  else
  {
    RCLCPP_WARN(logger_, "Unable to find joint group with name '%s'.", group_name_.c_str());
  }

  std::vector<std::string> empty;
  return empty;
}

const std::string& PlanningComponent::getPlanningGroupName() const
{
  return group_name_;
}

bool PlanningComponent::setPathConstraints(const moveit_msgs::msg::Constraints& path_constraints)
{
  current_path_constraints_ = path_constraints;
  return true;
}

bool PlanningComponent::setTrajectoryConstraints(const moveit_msgs::msg::TrajectoryConstraints& trajectory_constraints)
{
  current_trajectory_constraints_ = trajectory_constraints;
  return true;
}

planning_interface::MotionPlanResponse PlanningComponent::plan(const PlanRequestParameters& parameters,
                                                               planning_scene::PlanningScenePtr planning_scene)
{
  auto plan_solution = planning_interface::MotionPlanResponse();

  // check if joint_model_group exists
  if (!joint_model_group_)
  {
    RCLCPP_ERROR(logger_, "Failed to retrieve joint model group for name '%s'.", group_name_.c_str());
    plan_solution.error_code = moveit::core::MoveItErrorCode::INVALID_GROUP_NAME;
    return plan_solution;
  }

  // Check if goal constraints exist
  if (current_goal_constraints_.empty())
  {
    RCLCPP_ERROR(logger_, "No goal constraints set for planning request");
    plan_solution.error_code = moveit::core::MoveItErrorCode::INVALID_GOAL_CONSTRAINTS;
    return plan_solution;
  }

  if (!planning_scene)
  {  // Clone current planning scene
    auto planning_scene_monitor = moveit_cpp_->getPlanningSceneMonitorNonConst();
    planning_scene_monitor->updateFrameTransforms();
    planning_scene = [planning_scene_monitor] {
      planning_scene_monitor::LockedPlanningSceneRO ls(planning_scene_monitor);
      return planning_scene::PlanningScene::clone(ls);
    }();
    planning_scene_monitor.reset();  // release this pointer}
  }
  // Init MotionPlanRequest
  ::planning_interface::MotionPlanRequest request = getMotionPlanRequest(parameters);

  // Set start state
  planning_scene->setCurrentState(request.start_state);

  // Run planning attempt
  return moveit::planning_pipeline_interfaces::planWithSinglePipeline(request, planning_scene,
                                                                      moveit_cpp_->getPlanningPipelines());
}

planning_interface::MotionPlanResponse PlanningComponent::plan(
    const MultiPipelinePlanRequestParameters& parameters,
    const moveit::planning_pipeline_interfaces::SolutionSelectionFunction& solution_selection_function,
    const moveit::planning_pipeline_interfaces::StoppingCriterionFunction& stopping_criterion_callback,
    planning_scene::PlanningScenePtr planning_scene)
{
  auto plan_solution = planning_interface::MotionPlanResponse();

  // check if joint_model_group exists
  if (!joint_model_group_)
  {
    RCLCPP_ERROR(logger_, "Failed to retrieve joint model group for name '%s'.", group_name_.c_str());
    plan_solution.error_code = moveit::core::MoveItErrorCode::INVALID_GROUP_NAME;
    return plan_solution;
  }

  // Check if goal constraints exist
  if (current_goal_constraints_.empty())
  {
    RCLCPP_ERROR(logger_, "No goal constraints set for planning request");
    plan_solution.error_code = moveit::core::MoveItErrorCode::INVALID_GOAL_CONSTRAINTS;
    return plan_solution;
  }

  if (!planning_scene)
  {  // Clone current planning scene
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor =
        moveit_cpp_->getPlanningSceneMonitorNonConst();
    planning_scene_monitor->updateFrameTransforms();
    planning_scene = [planning_scene_monitor] {
      planning_scene_monitor::LockedPlanningSceneRO ls(planning_scene_monitor);
      return planning_scene::PlanningScene::clone(ls);
    }();
    planning_scene_monitor.reset();  // release this pointer}
  }
  // Init MotionPlanRequest
  std::vector<::planning_interface::MotionPlanRequest> requests = getMotionPlanRequestVector(parameters);

  // Set start state
  for (const auto& request : requests)
  {
    planning_scene->setCurrentState(request.start_state);
  }

  const auto motion_plan_response_vector = moveit::planning_pipeline_interfaces::planWithParallelPipelines(
      requests, planning_scene, moveit_cpp_->getPlanningPipelines(), stopping_criterion_callback,
      solution_selection_function);

  try
  {
    // If a solution_selection function is passed to the parallel pipeline interface, the returned vector contains only
    // the selected solution
    plan_solution = motion_plan_response_vector.at(0);
  }
  catch (std::out_of_range&)
  {
    RCLCPP_ERROR(logger_, "MotionPlanResponse vector was empty after parallel planning");
    plan_solution.error_code = moveit::core::MoveItErrorCode::INVALID_GOAL_CONSTRAINTS;
  }
  // Run planning attempt
  return plan_solution;
}

planning_interface::MotionPlanResponse PlanningComponent::plan()
{
  PlanRequestParameters plan_request_parameters;
  plan_request_parameters.load(node_);
  RCLCPP_DEBUG_STREAM(
      logger_, "Default plan request parameters loaded with --"
                   << " planning_pipeline: " << plan_request_parameters.planning_pipeline << ','
                   << " planner_id: " << plan_request_parameters.planner_id << ','
                   << " planning_time: " << plan_request_parameters.planning_time << ','
                   << " planning_attempts: " << plan_request_parameters.planning_attempts << ','
                   << " max_velocity_scaling_factor: " << plan_request_parameters.max_velocity_scaling_factor << ','
                   << " max_acceleration_scaling_factor: " << plan_request_parameters.max_acceleration_scaling_factor);
  return plan(plan_request_parameters);
}

bool PlanningComponent::setStartState(const moveit::core::RobotState& start_state)
{
  considered_start_state_ = std::make_shared<moveit::core::RobotState>(start_state);
  return true;
}

moveit::core::RobotStatePtr PlanningComponent::getStartState()
{
  if (considered_start_state_)
  {
    return considered_start_state_;
  }
  else
  {
    moveit::core::RobotStatePtr s;
    moveit_cpp_->getCurrentState(s, 1.0);
    return s;
  }
}

bool PlanningComponent::setStartState(const std::string& start_state_name)
{
  const auto& named_targets = getNamedTargetStates();
  if (std::find(named_targets.begin(), named_targets.end(), start_state_name) == named_targets.end())
  {
    RCLCPP_ERROR(logger_, "No predefined joint state found for target name '%s'", start_state_name.c_str());
    return false;
  }
  moveit::core::RobotState start_state(moveit_cpp_->getRobotModel());
  start_state.setToDefaultValues();  // required to ensure all joints are initialized
  start_state.setToDefaultValues(joint_model_group_, start_state_name);
  return setStartState(start_state);
}

void PlanningComponent::setStartStateToCurrentState()
{
  considered_start_state_.reset();
}

std::map<std::string, double> PlanningComponent::getNamedTargetStateValues(const std::string& name)
{
  // TODO(henningkayser): verify result
  std::map<std::string, double> positions;
  joint_model_group_->getVariableDefaultPositions(name, positions);
  return positions;
}

void PlanningComponent::setWorkspace(double minx, double miny, double minz, double maxx, double maxy, double maxz)
{
  workspace_parameters_.header.frame_id = moveit_cpp_->getRobotModel()->getModelFrame();
  workspace_parameters_.header.stamp = node_->now();
  workspace_parameters_.min_corner.x = minx;
  workspace_parameters_.min_corner.y = miny;
  workspace_parameters_.min_corner.z = minz;
  workspace_parameters_.max_corner.x = maxx;
  workspace_parameters_.max_corner.y = maxy;
  workspace_parameters_.max_corner.z = maxz;
  workspace_parameters_set_ = true;
}

void PlanningComponent::unsetWorkspace()
{
  workspace_parameters_set_ = false;
}

bool PlanningComponent::setGoal(const std::vector<moveit_msgs::msg::Constraints>& goal_constraints)
{
  current_goal_constraints_ = goal_constraints;
  return true;
}

bool PlanningComponent::setGoal(const moveit::core::RobotState& goal_state)
{
  current_goal_constraints_ = { kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group_) };
  return true;
}

bool PlanningComponent::setGoal(const geometry_msgs::msg::PoseStamped& goal_pose, const std::string& link_name)
{
  current_goal_constraints_ = { kinematic_constraints::constructGoalConstraints(link_name, goal_pose) };
  return true;
}

bool PlanningComponent::setGoal(const std::string& goal_state_name)
{
  const auto& named_targets = getNamedTargetStates();
  if (std::find(named_targets.begin(), named_targets.end(), goal_state_name) == named_targets.end())
  {
    RCLCPP_ERROR(logger_, "No predefined joint state found for target name '%s'", goal_state_name.c_str());
    return false;
  }
  moveit::core::RobotState goal_state(moveit_cpp_->getRobotModel());
  goal_state.setToDefaultValues(joint_model_group_, goal_state_name);
  return setGoal(goal_state);
}

::planning_interface::MotionPlanRequest
PlanningComponent::getMotionPlanRequest(const PlanRequestParameters& plan_request_parameters)
{
  ::planning_interface::MotionPlanRequest request;
  request.group_name = group_name_;
  request.pipeline_id = plan_request_parameters.planning_pipeline;
  request.planner_id = plan_request_parameters.planner_id;
  request.num_planning_attempts = std::max(1, plan_request_parameters.planning_attempts);
  request.allowed_planning_time = plan_request_parameters.planning_time;
  request.max_velocity_scaling_factor = plan_request_parameters.max_velocity_scaling_factor;
  request.max_acceleration_scaling_factor = plan_request_parameters.max_acceleration_scaling_factor;
  if (workspace_parameters_set_)
  {
    request.workspace_parameters = workspace_parameters_;
  }
  request.goal_constraints = current_goal_constraints_;
  request.path_constraints = current_path_constraints_;
  request.trajectory_constraints = current_trajectory_constraints_;

  // Set start state
  moveit::core::RobotStatePtr start_state = considered_start_state_;
  if (!start_state)
  {
    start_state = moveit_cpp_->getCurrentState();
  }
  start_state->update();
  moveit::core::robotStateToRobotStateMsg(*start_state, request.start_state);
  return request;
}

std::vector<::planning_interface::MotionPlanRequest> PlanningComponent::getMotionPlanRequestVector(
    const MultiPipelinePlanRequestParameters& multi_pipeline_plan_request_parameters)
{
  std::vector<::planning_interface::MotionPlanRequest> motion_plan_requests;
  motion_plan_requests.reserve(multi_pipeline_plan_request_parameters.plan_request_parameter_vector.size());
  for (const auto& plan_request_parameters : multi_pipeline_plan_request_parameters.plan_request_parameter_vector)
  {
    motion_plan_requests.push_back(getMotionPlanRequest(plan_request_parameters));
  }
  return motion_plan_requests;
}
}  // namespace moveit_cpp
