/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

/* Author: Sebastian Jahr
 */

#include <moveit/global_planner/global_planner_component.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/move_it_error_codes.h>

#include <chrono>
#include <thread>

namespace moveit_hybrid_planning
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("global_planner_component");
constexpr char PLANNING_PLUGIN_PARAM[] = "planning_plugin";

GlobalPlannerComponent::GlobalPlannerComponent(const rclcpp::NodeOptions& options)
  : Node("global_planner_component", options)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  config_.publish_planning_scene_topic =
      this->declare_parameter<std::string>("publish_planning_scene_topic", "/publish_planning_scene");

  std::string ns = "planning_pipelines";
  config_.pipeline_names =
      this->declare_parameter<std::vector<std::string>>(ns + ".pipeline_names", std::vector<std::string>({ "ompl" }));

  // Initialize global planning request action server
  global_planning_request_server_ = rclcpp_action::create_server<moveit_msgs::action::GlobalPlanner>(
      this->get_node_base_interface(), this->get_node_clock_interface(), this->get_node_logging_interface(),
      this->get_node_waitables_interface(), "global_planning_action",
      [](const rclcpp_action::GoalUUID& /*unused*/,
         std::shared_ptr<const moveit_msgs::action::GlobalPlanner::Goal> /*unused*/) {
        RCLCPP_INFO(LOGGER, "Received global planning goal request");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::GlobalPlanner>>& /*unused*/) {
        RCLCPP_INFO(LOGGER, "Received request to cancel global planning goal");
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      std::bind(&GlobalPlannerComponent::globalPlanningRequestCallback, this, std::placeholders::_1));

  global_trajectory_pub_ = this->create_publisher<moveit_msgs::msg::MotionPlanResponse>("global_trajectory", 1);

  // Initialize global planner after construction
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1), [this]() {
    if (initialized_)
    {
      timer_->cancel();
    }
    else
    {
      initialized_ = this->init();
      if (!initialized_)
      {
        const std::string error = "Failed to initialize global planner";
        timer_->cancel();
        throw std::runtime_error(error);
      }
    }
  });
}

bool GlobalPlannerComponent::init()
{
  auto node_ptr = shared_from_this();
  // Configure planning scene monitor
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(
      node_ptr, "robot_description", tf_buffer_, "global_planner/planning_scene_monitor"));
  // Allows us to sycronize to Rviz and also publish collision objects to ourselves
  RCLCPP_DEBUG(LOGGER, "Configuring Planning Scene Monitor");
  if (planning_scene_monitor_->getPlanningScene())
  {
    // Start state and scene monitors
    RCLCPP_INFO(LOGGER, "Listening to '/publish_planning_scene' for planning scene updates");
    planning_scene_monitor_->startSceneMonitor();
  }
  else
  {
    const std::string error = "Unable to configure planning scene monitor";
    RCLCPP_FATAL(LOGGER, error);
    throw std::runtime_error(error);
  }

  robot_model_ = planning_scene_monitor_->getRobotModel();
  if (!robot_model_)
  {
    const std::string error = "Unable to construct robot model. Please make sure all needed information is on the "
                              "parameter server.";
    RCLCPP_FATAL(LOGGER, error);
    throw std::runtime_error(error);
  }

  // Load planning pipelines
  // TODO Use refactored MoveItCpp instance (load only planning pipeline and planning scene monitor) to reduce redundancy
  for (const auto& planning_pipeline_name : config_.pipeline_names)
  {
    if (planning_pipelines_.count(planning_pipeline_name) > 0)
    {
      RCLCPP_WARN(LOGGER, "Skipping duplicate entry for planning pipeline '%s'.", planning_pipeline_name.c_str());
      continue;
    }
    RCLCPP_INFO(LOGGER, "Loading planning pipeline '%s'", planning_pipeline_name.c_str());
    planning_pipeline::PlanningPipelinePtr pipeline;
    pipeline.reset(
        new planning_pipeline::PlanningPipeline(robot_model_, node_ptr, planning_pipeline_name, PLANNING_PLUGIN_PARAM));
    if (!pipeline->getPlannerManager())
    {
      RCLCPP_ERROR(LOGGER, "Failed to initialize planning pipeline '%s'.", planning_pipeline_name.c_str());
      continue;
    }
    planning_pipelines_[planning_pipeline_name] = pipeline;
  }
  if (planning_pipelines_.empty())
  {
    RCLCPP_ERROR(LOGGER, "Failed to load any planning pipelines.");
  }
  return true;
}

void GlobalPlannerComponent::globalPlanningRequestCallback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::GlobalPlanner>> goal_handle)
{
  // TODO: Add feedback
  const auto goal = goal_handle->get_goal();

  // Plan global trajectory
  RCLCPP_INFO(LOGGER, "Start global planning");
  moveit_msgs::msg::MotionPlanResponse planning_solution = plan(goal->request);

  // Publish global planning solution to the local planner
  this->global_trajectory_pub_->publish(planning_solution);

  // Send action response
  auto result = std::make_shared<moveit_msgs::action::GlobalPlanner::Result>();
  result->response = planning_solution;
  goal_handle->succeed(result);

  // Save newest planning solution
  last_global_solution_ = planning_solution;  // TODO Add Service to expose this
};

moveit_msgs::msg::MotionPlanResponse GlobalPlannerComponent::plan(moveit_msgs::msg::MotionPlanRequest planning_problem)
{
  // Clone current planning scene
  planning_scene_monitor_->updateFrameTransforms();
  planning_scene_monitor_->lockSceneRead();  // LOCK planning scene
  ::planning_scene::PlanningScenePtr planning_scene =
      ::planning_scene::PlanningScene::clone(planning_scene_monitor_->getPlanningScene());
  planning_scene_monitor_->unlockSceneRead();  // UNLOCK planning scene

  // TODO implement start/current robot state considerations
  // TODO refactor get current state function --> see planning_context for example

  // Set start state
  moveit_msgs::msg::MotionPlanResponse planning_solution;
  moveit::core::RobotState start_state(robot_model_);
  moveit::core::robotStateMsgToRobotState(planning_problem.start_state,
                                          start_state);  // Use start state from planning problem
  planning_scene->setCurrentState(start_state);

  // Set goal constraints
  ::planning_interface::MotionPlanResponse response;
  if (planning_problem.goal_constraints.empty())
  {
    RCLCPP_ERROR(LOGGER, "No goal constraints set for planning request");
    planning_solution.error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return planning_solution;
  }

  // Run planning attempt
  const planning_pipeline::PlanningPipelinePtr pipeline = planning_pipelines_.at(
      config_.pipeline_names[0]);  // Assume, there is only one planning pipeline TODO expand Request
  pipeline->generatePlan(planning_scene, planning_problem, response);
  if (response.error_code_.val != response.error_code_.SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
  }
  response.getMessage(planning_solution);
  return planning_solution;
}
}  // namespace moveit_hybrid_planning

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit_hybrid_planning::GlobalPlannerComponent)
