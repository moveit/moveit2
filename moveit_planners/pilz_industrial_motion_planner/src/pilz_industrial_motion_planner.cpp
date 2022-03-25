/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
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
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
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

#include <rclcpp/logging.hpp>

#include "pilz_industrial_motion_planner/pilz_industrial_motion_planner.h"

#include "pilz_industrial_motion_planner/planning_context_loader.h"
#include "pilz_industrial_motion_planner/planning_context_loader_ptp.h"
#include "pilz_industrial_motion_planner/planning_exceptions.h"

#include "pilz_industrial_motion_planner/cartesian_limits_aggregator.h"
#include "pilz_industrial_motion_planner/joint_limits_aggregator.h"

// Boost includes
#include <boost/scoped_ptr.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <pluginlib/class_loader.hpp>

#include <utility>

namespace pilz_industrial_motion_planner
{
static const std::string PARAM_NAMESPACE_LIMITS = "robot_description_planning";
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit.pilz_industrial_motion_planner");

bool CommandPlanner::initialize(const moveit::core::RobotModelConstPtr& model, const rclcpp::Node::SharedPtr& node,
                                const std::string& ns)
{
  // Call parent class initialize
  planning_interface::PlannerManager::initialize(model, node, ns);

  // Store the model, node, and namespace
  model_ = model;
  node_ = node;
  namespace_ = ns;

  loadPlannerConfigurations();

  // Load the planning context loader
  planner_context_loader = std::make_unique<pluginlib::ClassLoader<PlanningContextLoader>>(
      "pilz_industrial_motion_planner", "pilz_industrial_motion_planner::PlanningContextLoader");

  // List available plugins
  const std::vector<std::string>& factories = planner_context_loader->getDeclaredClasses();
  std::stringstream ss;
  for (const auto& factory : factories)
  {
    ss << factory << " ";
  }

  RCLCPP_INFO_STREAM(LOGGER, "Available plugins: " << ss.str());

  // Load each factory
  for (const auto& factory : factories)
  {
    RCLCPP_INFO_STREAM(LOGGER, "About to load: " << factory);
    PlanningContextLoaderPtr loader_pointer(planner_context_loader->createSharedInstance(factory));

    pilz_industrial_motion_planner::LimitsContainer limits;
    limits.setJointLimits(aggregated_limit_active_joints_);
    limits.setCartesianLimits(cartesian_limit_);

    loader_pointer->setLimits(limits);
    loader_pointer->setModel(model_);

    registerContextLoader(loader_pointer);
  }

  return true;
}

bool CommandPlanner::loadPlannerConfiguration(const std::string& group_name, const std::string& planner_id,
                                              const std::map<std::string, std::string>& group_params,
                                              planning_interface::PlannerConfigurationSettings& planner_config) const
{
  const rcl_interfaces::msg::ListParametersResult planner_params_result =
      node_->list_parameters({ namespace_ + ".planner_configs." + planner_id }, 2);

  if (planner_params_result.names.empty())
  {
    RCLCPP_DEBUG_STREAM(LOGGER, "Could not find parameters for the planner configuration '" << planner_id << "'");
    return false;
  }

  planner_config.name = group_name + "[" + planner_id + "]";
  planner_config.group = group_name;

  // Default to specified parameters of the group (overridden by configuration
  // specific parameters)
  planner_config.config = group_params;

  // Read parameters specific for this configuration
  for (const auto& planner_param : planner_params_result.names)
  {
    const rclcpp::Parameter param = node_->get_parameter(planner_param);
    const auto param_name = planner_param.substr(planner_param.find(planner_id) + planner_id.size() + 1);
    planner_config.config[param_name] = param.value_to_string();
  }

  return true;
}

void CommandPlanner::loadPlannerConfigurations()
{
  // Obtain the aggregated joint limits
  aggregated_limit_active_joints_ = pilz_industrial_motion_planner::JointLimitsAggregator::getAggregatedLimits(
      node_, PARAM_NAMESPACE_LIMITS, model_->getActiveJointModels());

  // Obtain cartesian limits
  cartesian_limit_ =
      pilz_industrial_motion_planner::CartesianLimitsAggregator::getAggregatedLimits(node_, PARAM_NAMESPACE_LIMITS);

  // Read the planning configuration for each group
  planning_interface::PlannerConfigurationMap config;

  for (const std::string& group_name : model_->getJointModelGroupNames())
  {
    // the set of planning parameters that can be specific for the group (inherited by configurations of that group)
    // with their expected parameter type
    static const std::pair<std::string, rclcpp::ParameterType> KNOWN_GROUP_PARAMS[] = {};

    // The set of planning parameters that can be specific for the group
    // (inherited by configurations of that group) with their expected parameter type
    const std::string group_name_param = namespace_ + "." + group_name;

    // get parameters specific for the robot planning group
    std::map<std::string, std::string> specific_group_params;
    for (const auto& k : KNOWN_GROUP_PARAMS)
    {
      const std::string param_name{ group_name_param + "." + k.first };
      if (node_->has_parameter(param_name))
      {
        const rclcpp::Parameter parameter = node_->get_parameter(param_name);
        if (parameter.get_type() != k.second)
        {
          RCLCPP_ERROR_STREAM(LOGGER, "Invalid type for parameter '" << k.first << "' expected ["
                                                                     << rclcpp::to_string(k.second) << "] got ["
                                                                     << rclcpp::to_string(parameter.get_type()) << "]");
          continue;
        }
        specific_group_params[k.first] = parameter.value_to_string();
      }
    }

    // Add default planner configuration
    planning_interface::PlannerConfigurationSettings default_pc;
    std::string default_planner_id;
    if (node_->get_parameter(group_name_param + ".default_planner_config", default_planner_id))
    {
      if (not loadPlannerConfiguration(group_name, default_planner_id, specific_group_params, default_pc))
      {
        default_planner_id = "";
      }
    }

    if (default_planner_id.empty())
    {
      default_pc.group = group_name;
      default_pc.config = specific_group_params;
    }

    default_pc.name = group_name;  // this is the name of the default config
    config[default_pc.name] = default_pc;

    // Get parameters specific to each planner type
    std::vector<std::string> config_names;
    if (node_->get_parameter(group_name_param + ".planner_configs", config_names))
    {
      for (const auto& planner_id : config_names)
      {
        planning_interface::PlannerConfigurationSettings pc;
        if (loadPlannerConfiguration(group_name, planner_id, specific_group_params, pc))
        {
          config[pc.name] = pc;
        }
      }
    }
  }

  for (const auto& [name, config_settings] : config)
  {
    RCLCPP_DEBUG_STREAM(LOGGER, "Parameters for configuration '" << name << "'");

    for (const auto& [param_name, param_value] : config_settings.config)
    {
      RCLCPP_DEBUG_STREAM(LOGGER, " - " << param_name << " = " << param_value);
    }
  }

  setPlannerConfigurations(config);
}

std::string CommandPlanner::getDescription() const
{
  return "Pilz Industrial Motion Planner";
}

void CommandPlanner::getPlanningAlgorithms(std::vector<std::string>& algs) const
{
  algs.clear();

  for (const auto& context_loader : context_loader_map_)
  {
    algs.push_back(context_loader.first);
  }
}

planning_interface::PlanningContextPtr
CommandPlanner::getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                   const moveit_msgs::msg::MotionPlanRequest& req,
                                   moveit_msgs::msg::MoveItErrorCodes& error_code) const
{
  // TODO(henningkayser): print req
  // RCLCPP_DEBUG_STREAM(LOGGER, "Loading PlanningContext for request\n<request>\n" << req << "\n</request>");
  RCLCPP_DEBUG(LOGGER, "Loading PlanningContext");

  // Check that a loaded for this request exists
  if (!canServiceRequest(req))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "No ContextLoader for planner_id '" << req.planner_id.c_str()
                                                                    << "' found. Planning not possible.");
    return nullptr;
  }

  planning_interface::PlanningContextPtr planning_context;

  if (context_loader_map_.at(req.planner_id)->loadContext(planning_context, req.planner_id, req.group_name))
  {
    RCLCPP_DEBUG_STREAM(LOGGER, "Found planning context loader for " << req.planner_id << " group:" << req.group_name);
    planning_context->setMotionPlanRequest(req);
    planning_context->setPlanningScene(planning_scene);
    return planning_context;
  }
  else
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return nullptr;
  }
}

bool CommandPlanner::canServiceRequest(const moveit_msgs::msg::MotionPlanRequest& req) const
{
  return context_loader_map_.find(req.planner_id) != context_loader_map_.end();
}

void CommandPlanner::registerContextLoader(
    const pilz_industrial_motion_planner::PlanningContextLoaderPtr& planning_context_loader)
{
  // Only add if command is not already in list, throw exception if not
  if (context_loader_map_.find(planning_context_loader->getAlgorithm()) == context_loader_map_.end())
  {
    context_loader_map_[planning_context_loader->getAlgorithm()] = planning_context_loader;
    RCLCPP_INFO_STREAM(LOGGER, "Registered Algorithm [" << planning_context_loader->getAlgorithm() << "]");
  }
  else
  {
    throw ContextLoaderRegistrationException("The command [" + planning_context_loader->getAlgorithm() +
                                             "] is already registered");
  }
}

}  // namespace pilz_industrial_motion_planner

PLUGINLIB_EXPORT_CLASS(pilz_industrial_motion_planner::CommandPlanner, planning_interface::PlannerManager)
