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

/* Author: Ioan Sucan, Sebastian Jahr */

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <fmt/format.h>
#include <moveit/utils/logger.hpp>

namespace
{
namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.ros.planning_pipeline");
}
}  // namespace

/**
 * @brief Transform a joint trajectory into a vector of joint constraints
 *
 * @param trajectory Reference trajectory from which the joint constraints are created
 * @return A vector of joint constraints each corresponding to a waypoint of the reference trajectory.
 */
[[nodiscard]] std::vector<moveit_msgs::msg::Constraints>
getTrajectoryConstraints(const robot_trajectory::RobotTrajectoryPtr& trajectory)
{
  const std::vector<std::string> joint_names =
      trajectory->getFirstWayPoint().getJointModelGroup(trajectory->getGroupName())->getActiveJointModelNames();

  std::vector<moveit_msgs::msg::Constraints> trajectory_constraints;
  // Iterate through waypoints and create a joint constraint for each
  for (size_t waypoint_index = 0; waypoint_index < trajectory->getWayPointCount(); ++waypoint_index)
  {
    moveit_msgs::msg::Constraints new_waypoint_constraint;
    // Iterate through joints and copy waypoint data to joint constraint
    for (const auto& joint_name : joint_names)
    {
      moveit_msgs::msg::JointConstraint new_joint_constraint;
      new_joint_constraint.joint_name = joint_name;
      new_joint_constraint.position = trajectory->getWayPoint(waypoint_index).getVariablePosition(joint_name);
      new_waypoint_constraint.joint_constraints.push_back(new_joint_constraint);
    }
    trajectory_constraints.push_back(new_waypoint_constraint);
  }
  return trajectory_constraints;
}
}  // namespace

namespace planning_pipeline
{
PlanningPipeline::PlanningPipeline(const moveit::core::RobotModelConstPtr& model,
                                   const std::shared_ptr<rclcpp::Node>& node, const std::string& parameter_namespace)
  : active_{ false }
  , node_(node)
  , parameter_namespace_(parameter_namespace)
  , robot_model_(model)
  , logger_(moveit::getLogger("moveit.ros.planning_pipeline"))
{
  auto param_listener = planning_pipeline_parameters::ParamListener(node, parameter_namespace);
  pipeline_parameters_ = param_listener.get_params();

  configure();
}

PlanningPipeline::PlanningPipeline(const moveit::core::RobotModelConstPtr& model,
                                   const std::shared_ptr<rclcpp::Node>& node, const std::string& parameter_namespace,
                                   const std::vector<std::string>& planner_plugin_names,
                                   const std::vector<std::string>& request_adapter_plugin_names,
                                   const std::vector<std::string>& response_adapter_plugin_names)
  : active_{ false }
  , node_(node)
  , parameter_namespace_(parameter_namespace)
  , robot_model_(model)
  , logger_(moveit::getLogger("moveit.ros.planning_pipeline"))
{
  pipeline_parameters_.planning_plugins = planner_plugin_names;
  pipeline_parameters_.request_adapters = request_adapter_plugin_names;
  pipeline_parameters_.response_adapters = response_adapter_plugin_names;
  configure();
}

void PlanningPipeline::configure()
{
  // If progress topic parameter is not empty, initialize publisher
  if (!pipeline_parameters_.progress_topic.empty())
  {
    progress_publisher_ = node_->create_publisher<moveit_msgs::msg::PipelineState>(pipeline_parameters_.progress_topic,
                                                                                   rclcpp::SystemDefaultsQoS());
  }

  // Create planner plugin loader
  try
  {
    planner_plugin_loader_ = std::make_unique<pluginlib::ClassLoader<planning_interface::PlannerManager>>(
        "moveit_core", "planning_interface::PlannerManager");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_FATAL(logger_, "Exception while creating planning plugin loader %s", ex.what());
    throw;
  }

  if (pipeline_parameters_.planning_plugins.empty() || pipeline_parameters_.planning_plugins.at(0) == "UNKNOWN")
  {
    const std::string classes_str = fmt::format("{}", fmt::join(planner_plugin_loader_->getDeclaredClasses(), ", "));
    throw std::runtime_error("Planning plugin name is empty or not defined in namespace '" + parameter_namespace_ +
                             "'. Please choose one of the available plugins: " + classes_str);
  }

  for (const auto& planner_name : pipeline_parameters_.planning_plugins)
  {
    planning_interface::PlannerManagerPtr planner_instance;

    // Load plugin
    try
    {
      planner_instance = planner_plugin_loader_->createUniqueInstance(planner_name);
    }
    catch (pluginlib::PluginlibException& ex)
    {
      std::string classes_str = fmt::format("{}", fmt::join(planner_plugin_loader_->getDeclaredClasses(), ", "));
      RCLCPP_FATAL(getLogger(),
                   "Exception while loading planner '%s': %s"
                   "Available plugins: %s",
                   planner_name.c_str(), ex.what(), classes_str.c_str());
      throw;
    }

    // Check if planner is not NULL
    if (!planner_instance)
    {
      throw std::runtime_error("Unable to initialize planning plugin " + planner_name +
                               ". Planner instance is nullptr.");
    }

    // Initialize planner
    if (!planner_instance->initialize(robot_model_, node_, parameter_namespace_))
    {
      throw std::runtime_error("Unable to initialize planning plugin " + planner_name);
    }
    RCLCPP_INFO(getLogger(), "Successfully loaded planner '%s'", planner_instance->getDescription().c_str());
    planner_map_.insert(std::make_pair(planner_name, planner_instance));
  }

  // Load the planner request adapters
  if (!pipeline_parameters_.request_adapters.empty())
  {
    try
    {
      request_adapter_plugin_loader_ =
          std::make_unique<pluginlib::ClassLoader<planning_interface::PlanningRequestAdapter>>(
              "moveit_core", "planning_interface::PlanningRequestAdapter");
    }
    catch (pluginlib::PluginlibException& ex)
    {
      RCLCPP_FATAL(logger_, "Exception while creating planning plugin loader %s", ex.what());
      throw;
    }

    if (request_adapter_plugin_loader_)
    {
      loadPluginVector<planning_interface::PlanningRequestAdapter>(node_, request_adapter_plugin_loader_,
                                                                   planning_request_adapter_vector_,
                                                                   pipeline_parameters_.request_adapters,
                                                                   parameter_namespace_);
    }
    else
    {
      RCLCPP_WARN(logger_, "No planning request adapter names specified.");
    }
  }
  else
  {
    RCLCPP_WARN(logger_, "No planning request adapter names specified.");
  }
  // Load the planner response adapters
  if (!pipeline_parameters_.response_adapters.empty())
  {
    try
    {
      response_adapter_plugin_loader_ =
          std::make_unique<pluginlib::ClassLoader<planning_interface::PlanningResponseAdapter>>(
              "moveit_core", "planning_interface::PlanningResponseAdapter");
    }
    catch (pluginlib::PluginlibException& ex)
    {
      RCLCPP_FATAL(logger_, "Exception while creating planning plugin loader %s", ex.what());
      throw;
    }
    if (response_adapter_plugin_loader_)
    {
      loadPluginVector<planning_interface::PlanningResponseAdapter>(node_, response_adapter_plugin_loader_,
                                                                    planning_response_adapter_vector_,
                                                                    pipeline_parameters_.response_adapters,
                                                                    parameter_namespace_);
    }
  }
  else
  {
    RCLCPP_WARN(logger_, "No planning response adapter names specified.");
  }
}

void PlanningPipeline::publishPipelineState(moveit_msgs::msg::MotionPlanRequest req,
                                            const planning_interface::MotionPlanResponse& res,
                                            const std::string& pipeline_stage) const
{
  if (progress_publisher_)
  {
    moveit_msgs::msg::PipelineState progress;
    progress.request = std::move(req);
    res.getMessage(progress.response);
    progress.pipeline_stage = pipeline_stage;
    progress_publisher_->publish(progress);
  }
}

bool PlanningPipeline::generatePlan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                    const planning_interface::MotionPlanRequest& req,
                                    planning_interface::MotionPlanResponse& res,
                                    const bool publish_received_requests) const
{
  assert(!planner_map_.empty());

  // Set planning pipeline active
  active_ = true;

  // broadcast the request we are about to work on, if needed
  if (publish_received_requests)
  {
    publishPipelineState(req, res, std::string(""));
  }

  // ---------------------------------
  // Solve the motion planning problem
  // ---------------------------------

  planning_interface::MotionPlanRequest mutable_request = req;
  try
  {
    // Call plan request adapter chain
    for (const auto& req_adapter : planning_request_adapter_vector_)
    {
      assert(req_adapter);
      RCLCPP_INFO(node_->get_logger(), "Calling PlanningRequestAdapter '%s'", req_adapter->getDescription().c_str());
      const auto status = req_adapter->adapt(planning_scene, mutable_request);
      res.error_code = status.val;
      // Publish progress
      publishPipelineState(mutable_request, res, req_adapter->getDescription());
      // If adapter does not succeed, break chain and return false
      if (!res.error_code)
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "PlanningRequestAdapter '%s' failed, because '%s'. Aborting planning pipeline.",
                     req_adapter->getDescription().c_str(), status.message.c_str());
        active_ = false;
        return false;
      }
    }

    // Call planners
    for (const auto& planner_name : pipeline_parameters_.planning_plugins)
    {
      const auto& planner = planner_map_.at(planner_name);
      // Update reference trajectory with latest solution (if available)
      if (res.trajectory)
      {
        mutable_request.trajectory_constraints.constraints = getTrajectoryConstraints(res.trajectory);
      }

      // Try creating a planning context
      planning_interface::PlanningContextPtr context =
          planner->getPlanningContext(planning_scene, mutable_request, res.error_code);
      if (!context)
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "Failed to create PlanningContext for planner '%s'. Aborting planning pipeline.",
                     planner->getDescription().c_str());
        res.error_code = moveit::core::MoveItErrorCode::PLANNING_FAILED;
        active_ = false;
        return false;
      }

      // Run planner
      RCLCPP_INFO(node_->get_logger(), "Calling Planner '%s'", planner->getDescription().c_str());
      context->solve(res);
      publishPipelineState(mutable_request, res, planner->getDescription());

      // If planner does not succeed, break chain and return false
      if (!res.error_code)
      {
        RCLCPP_ERROR(node_->get_logger(), "Planner '%s' failed with error code %s", planner->getDescription().c_str(),
                     errorCodeToString(res.error_code).c_str());
        active_ = false;
        return false;
      }
    }

    // Call plan response adapter chain
    if (res.error_code)
    {
      // Call plan request adapter chain
      for (const auto& res_adapter : planning_response_adapter_vector_)
      {
        assert(res_adapter);
        RCLCPP_INFO(node_->get_logger(), "Calling PlanningResponseAdapter '%s'", res_adapter->getDescription().c_str());
        res_adapter->adapt(planning_scene, mutable_request, res);
        publishPipelineState(mutable_request, res, res_adapter->getDescription());
        // If adapter does not succeed, break chain and return false
        if (!res.error_code)
        {
          RCLCPP_ERROR(node_->get_logger(), "PlanningResponseAdapter '%s' failed with error code %s",
                       res_adapter->getDescription().c_str(), errorCodeToString(res.error_code).c_str());
          active_ = false;
          return false;
        }
      }
    }
  }
  catch (std::exception& ex)
  {
    RCLCPP_ERROR(logger_, "Exception caught: '%s'", ex.what());
    // Set planning pipeline to inactive
    active_ = false;
    return false;
  }

  // Make sure that planner id is set in the response
  if (res.planner_id.empty())
  {
    RCLCPP_WARN(logger_,
                "The planner plugin did not fill out the 'planner_id' field of the MotionPlanResponse. Setting "
                "it to the planner ID name of the MotionPlanRequest assuming that the planner plugin does warn "
                "you if it does not use the requested planner.");
    res.planner_id = req.planner_id;
  }

  // Set planning pipeline to inactive
  active_ = false;
  return static_cast<bool>(res);
}

void PlanningPipeline::terminate() const
{
  for (const auto& planner_pair : planner_map_)
  {
    if (planner_pair.second)
    {
      planner_pair.second->terminate();
    }
  }
}
}  // namespace planning_pipeline
