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

/* Author: Ioan Sucan, Sebastian Jahr
   Description: Implementation of a MoveIt planning pipeline composed of a planner plugin and request and response
   adapter plugins.
*/

#pragma once

#include <atomic>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_interface/planning_request_adapter.h>
#include <moveit/planning_interface/planning_response_adapter.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/pipeline_state.hpp>
#include <memory>
#include <moveit_planning_pipeline_export.h>
#include <planning_pipeline_parameters.hpp>

namespace planning_pipeline
{
/**
 * \brief Helper function template to load a vector of plugins
 *
 * \tparam TPluginClass Plugin class type, PlanningRequestAdapter or PlanningResponseAdapter
 * \param plugin_loader Loader to create the requested plugin
 * \param plugin_vector Vector to add the loaded plugin to
 * \param plugin_names Names of the plugins to be loaded
 */
template <class TPluginClass>
void loadPluginVector(const std::shared_ptr<rclcpp::Node>& node,
                      const std::unique_ptr<pluginlib::ClassLoader<TPluginClass>>& plugin_loader,
                      std::vector<std::shared_ptr<const TPluginClass>>& plugin_vector,
                      const std::vector<std::string>& plugin_names, const std::string& parameter_namespace)
{
  // Try loading a plugin for each plugin name
  for (const std::string& plugin_name : plugin_names)
  {
    RCLCPP_INFO(node->get_logger(), "Try loading adapter '%s'", plugin_name.c_str());
    std::shared_ptr<TPluginClass> adapter;
    try
    {
      adapter = plugin_loader->createUniqueInstance(plugin_name);
    }
    catch (pluginlib::PluginlibException& ex)
    {
      RCLCPP_FATAL(node->get_logger(), "Exception while loading planning adapter plugin '%s': %s", plugin_name.c_str(),
                   ex.what());
      throw;
    }
    // If loading was successful, initialize plugin and add to vector
    if (adapter)
    {
      adapter->initialize(node, parameter_namespace);
      plugin_vector.push_back(std::move(adapter));
      RCLCPP_INFO(node->get_logger(), "Loaded adapter '%s'", plugin_name.c_str());
    }
    else
    {
      RCLCPP_WARN(node->get_logger(), "Failed to initialize adapter '%s'. Not adding it to the chain.",
                  plugin_name.c_str());
    }
  }
};

/** \brief This class facilitates loading planning plugins and planning adapter plugins. It implements functionality to
 * use the loaded plugins in a motion planning pipeline consisting of PlanningRequestAdapters (Pre-processing), a
 * Planner plugin and PlanningResponseAdapters (Post-processing).*/
class MOVEIT_PLANNING_PIPELINE_EXPORT PlanningPipeline
{
public:
  /** \brief Given a robot model, a ROS node, and a parameter namespace, initialize the planning pipeline. The planner plugin
     and adapters are provided in form of ROS parameters. The order of the elements in the adapter vectors corresponds
     to the order in the motion planning pipeline.
     \param model The robot model for which this pipeline is initialized.
     \param node The ROS node that should be used for reading parameters needed for configuration
     \param parameter_namespace parameter namespace where the planner configurations are stored
  */
  PlanningPipeline(const moveit::core::RobotModelConstPtr& model, const std::shared_ptr<rclcpp::Node>& node,
                   const std::string& parameter_namespace);

  /** \brief Given a robot model, a ROS node, and a parameter namespace, initialize the planning pipeline with the given
     planner plugin and adapters. The order of the elements in the adapter vectors corresponds to the order in the
     motion planning pipeline.
     \param model The robot model for which this pipeline is initialized.
     \param node The ROS node that should be used for reading parameters needed for configuration
     \param parameter_namespace Parameter namespace where the planner configurations are stored
     \param planner_plugin_names Names of the planner plugins to use
     \param request_adapter_plugin_names Optional vector of RequestAdapter plugin names
     \param response_adapter_plugin_names Optional vector of ResponseAdapter plugin names
  */
  PlanningPipeline(const moveit::core::RobotModelConstPtr& model, const std::shared_ptr<rclcpp::Node>& node,
                   const std::string& parameter_namespace, const std::vector<std::string>& planner_plugin_names,
                   const std::vector<std::string>& request_adapter_plugin_names = std::vector<std::string>(),
                   const std::vector<std::string>& response_adapter_plugin_names = std::vector<std::string>());

  /*
  BEGIN BLOCK OF DEPRECATED FUNCTIONS: TODO(sjahr): Remove after 04/2024 (6 months from merge)
  */
  [[deprecated("Use generatePlan or ROS parameter API instead.")]] void displayComputedMotionPlans(bool /*flag*/){};
  [[deprecated("Use generatePlan or ROS parameter API instead.")]] void publishReceivedRequests(bool /*flag*/){};
  [[deprecated("Use generatePlan or ROS parameter API instead.")]] void checkSolutionPaths(bool /*flag*/){};
  [[deprecated("Use generatePlan or ROS parameter API instead.")]] bool getDisplayComputedMotionPlans() const
  {
    return false;
  }
  [[deprecated("Use generatePlan or ROS parameter API instead.")]] bool getPublishReceivedRequests() const
  {
    return false;
  }
  [[deprecated("Use generatePlan or ROS parameter API instead.")]] bool getCheckSolutionPaths() const
  {
    return false;
  }
  [[deprecated(
      "Please use getResponseAdapterPluginNames() or getRequestAdapterPluginNames().")]] const std::vector<std::string>&
  getAdapterPluginNames() const
  {
    return pipeline_parameters_.request_adapters;
  }
  [[deprecated("`check_solution_paths` and `display_computed_motion_plans` are deprecated. To validate the solution "
               "please add the ValidatePath response adapter and to publish the path use the DisplayMotionPath "
               "response adapter to your pipeline.")]] bool
  generatePlan(const planning_scene::PlanningSceneConstPtr& /*planning_scene*/,
               const planning_interface::MotionPlanRequest& /*req*/, planning_interface::MotionPlanResponse& /*res*/,
               const bool /*publish_received_requests*/, const bool /*check_solution_paths*/,
               const bool /*display_computed_motion_plans*/) const
  {
    return false;
  }
  [[deprecated("Please use getPlannerPluginNames().")]] const std::string& getPlannerPluginName() const
  {
    return pipeline_parameters_.planning_plugins.at(0);
  }
  [[deprecated(
      "Please use 'getPlannerManager(const std::string& planner_name)'.")]] const planning_interface::PlannerManagerPtr&
  getPlannerManager()
  {
    return planner_map_.at(pipeline_parameters_.planning_plugins.at(0));
  }
  /*
  END BLOCK OF DEPRECATED FUNCTIONS
  */

  /** \brief Call the chain of planning request adapters, motion planner plugin, and planning response adapters in
     sequence. \param planning_scene The planning scene where motion planning is to be done \param req The request for
     motion planning \param res The motion planning response \param publish_received_requests Flag indicating whether
     received requests should be published just before beginning processing (useful for debugging)
      */
  [[nodiscard]] bool generatePlan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                  const planning_interface::MotionPlanRequest& req,
                                  planning_interface::MotionPlanResponse& res,
                                  const bool publish_received_requests = false) const;

  /** \brief Request termination, if a generatePlan() function is currently computing plans */
  void terminate() const;

  /** \brief Get the names of the planning plugins used */
  [[nodiscard]] const std::vector<std::string>& getPlannerPluginNames() const
  {
    return pipeline_parameters_.planning_plugins;
  }

  /** \brief Get the names of the planning request adapter plugins used */
  [[nodiscard]] const std::vector<std::string>& getRequestAdapterPluginNames() const
  {
    return pipeline_parameters_.request_adapters;
  }

  /** \brief Get the names of the planning response adapter plugins in the order they are processed. */
  [[nodiscard]] const std::vector<std::string>& getResponseAdapterPluginNames() const
  {
    return pipeline_parameters_.response_adapters;
  }

  /** \brief Get the robot model that this pipeline is using */
  [[nodiscard]] const moveit::core::RobotModelConstPtr& getRobotModel() const
  {
    return robot_model_;
  }

  /** \brief Get current status of the planning pipeline */
  [[nodiscard]] bool isActive() const
  {
    return active_;
  }

  /** \brief Return the parameter namespace as name of the planning pipeline. */
  [[nodiscard]] std::string getName() const
  {
    return parameter_namespace_;
  }

  /** \brief Get access to planner plugin */
  const planning_interface::PlannerManagerPtr getPlannerManager(const std::string& planner_name)
  {
    if (planner_map_.find(planner_name) == planner_map_.end())
    {
      RCLCPP_ERROR(node_->get_logger(), "Cannot retrieve planner because '%s' does not exist.", planner_name.c_str());
      return nullptr;
    }
    return planner_map_.at(planner_name);
  }

private:
  /// \brief Helper function that is called by both constructors to configure and initialize a PlanningPipeline instance
  void configure();

  /**
   * @brief Helper function to publish the planning pipeline state during the planning process
   *
   * @param req Current request to publish
   * @param res Current pipeline result
   * @param pipeline_stage Last pipeline stage that got invoked
   */
  void publishPipelineState(moveit_msgs::msg::MotionPlanRequest req, const planning_interface::MotionPlanResponse& res,
                            const std::string& pipeline_stage) const;

  // Flag that indicates whether or not the planning pipeline is currently solving a planning problem
  mutable std::atomic<bool> active_;

  // ROS node and parameters
  std::shared_ptr<rclcpp::Node> node_;
  const std::string parameter_namespace_;
  planning_pipeline_parameters::Params pipeline_parameters_;

  // Planner plugin
  std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader_;
  std::unordered_map<std::string, planning_interface::PlannerManagerPtr> planner_map_;

  // Plan request adapters
  std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlanningRequestAdapter>> request_adapter_plugin_loader_;
  std::vector<planning_interface::PlanningRequestAdapterConstPtr> planning_request_adapter_vector_;

  // Plan response adapters
  std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlanningResponseAdapter>> response_adapter_plugin_loader_;
  std::vector<planning_interface::PlanningResponseAdapterConstPtr> planning_response_adapter_vector_;

  // Robot model
  moveit::core::RobotModelConstPtr robot_model_;

  /// Publish the planning pipeline progress
  rclcpp::Publisher<moveit_msgs::msg::PipelineState>::SharedPtr progress_publisher_;

  rclcpp::Logger logger_;
};

MOVEIT_CLASS_FORWARD(PlanningPipeline);  // Defines PlanningPipelinePtr, ConstPtr, WeakPtr... etc
}  // namespace planning_pipeline
