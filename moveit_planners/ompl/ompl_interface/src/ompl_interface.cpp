/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/ompl_interface/detail/constrained_valid_state_sampler.h>

#include <moveit/utils/lexical_casts.h>
#include <fstream>

namespace ompl_interface
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit.ompl_planning.ompl_interface");

OMPLInterface::OMPLInterface(const moveit::core::RobotModelConstPtr& robot_model, const rclcpp::Node::SharedPtr& node,
                             const std::string& parameter_namespace)
  : node_(node)
  , parameter_namespace_(parameter_namespace)
  , robot_model_(robot_model)
  , constraint_sampler_manager_(std::make_shared<constraint_samplers::ConstraintSamplerManager>())
  , context_manager_(robot_model, constraint_sampler_manager_)
  , use_constraints_approximations_(true)
  , simplify_solutions_(true)
{
  RCLCPP_DEBUG(LOGGER, "Initializing OMPL interface using ROS parameters");
  loadPlannerConfigurations();
  loadConstraintSamplers();
}

OMPLInterface::OMPLInterface(const moveit::core::RobotModelConstPtr& robot_model,
                             const planning_interface::PlannerConfigurationMap& pconfig,
                             const rclcpp::Node::SharedPtr& node, const std::string& parameter_namespace)
  : node_(node)
  , parameter_namespace_(parameter_namespace)
  , robot_model_(robot_model)
  , constraint_sampler_manager_(std::make_shared<constraint_samplers::ConstraintSamplerManager>())
  , context_manager_(robot_model, constraint_sampler_manager_)
  , use_constraints_approximations_(true)
  , simplify_solutions_(true)
{
  RCLCPP_DEBUG(LOGGER, "Initializing OMPL interface using specified configuration");
  setPlannerConfigurations(pconfig);
  loadConstraintSamplers();
}

OMPLInterface::~OMPLInterface() = default;

void OMPLInterface::setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pconfig)
{
  planning_interface::PlannerConfigurationMap pconfig2 = pconfig;

  // construct default configurations for planning groups that don't have configs already passed in
  for (const moveit::core::JointModelGroup* group : robot_model_->getJointModelGroups())
  {
    if (pconfig.find(group->getName()) == pconfig.end())
    {
      planning_interface::PlannerConfigurationSettings empty;
      empty.name = empty.group = group->getName();
      pconfig2[empty.name] = empty;
    }
  }

  context_manager_.setPlannerConfigurations(pconfig2);
}

ModelBasedPlanningContextPtr
OMPLInterface::getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                  const planning_interface::MotionPlanRequest& req) const
{
  moveit_msgs::msg::MoveItErrorCodes dummy;
  return getPlanningContext(planning_scene, req, dummy);
}

ModelBasedPlanningContextPtr
OMPLInterface::getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                  const planning_interface::MotionPlanRequest& req,
                                  moveit_msgs::msg::MoveItErrorCodes& error_code) const
{
  ModelBasedPlanningContextPtr ctx =
      context_manager_.getPlanningContext(planning_scene, req, error_code, node_, use_constraints_approximations_);
  if (ctx)
  {
    configureContext(ctx);
  }
  return ctx;
}

void OMPLInterface::configureContext(const ModelBasedPlanningContextPtr& context) const
{
  context->simplifySolutions(simplify_solutions_);
}

void OMPLInterface::loadConstraintSamplers()
{
  constraint_sampler_manager_loader_ =
      std::make_shared<constraint_sampler_manager_loader::ConstraintSamplerManagerLoader>(node_,
                                                                                          constraint_sampler_manager_);
}

bool OMPLInterface::loadPlannerConfiguration(const std::string& group_name, const std::string& planner_id,
                                             const std::map<std::string, std::string>& group_params,
                                             planning_interface::PlannerConfigurationSettings& planner_config)
{
  rcl_interfaces::msg::ListParametersResult planner_params_result =
      node_->list_parameters({ parameter_namespace_ + ".planner_configs." + planner_id }, 2);

  if (planner_params_result.names.empty())
  {
    RCLCPP_ERROR(LOGGER, "Could not find the planner configuration '%s' on the param server", planner_id.c_str());
    return false;
  }

  planner_config.name = group_name + "[" + planner_id + "]";
  planner_config.group = group_name;

  // default to specified parameters of the group (overridden by configuration specific parameters)
  planner_config.config = group_params;

  // read parameters specific for this configuration
  for (const auto& planner_param : planner_params_result.names)
  {
    const rclcpp::Parameter param = node_->get_parameter(planner_param);
    auto param_name = planner_param.substr(planner_param.find(planner_id) + planner_id.size() + 1);
    planner_config.config[param_name] = param.value_to_string();
  }

  return true;
}

void OMPLInterface::loadPlannerConfigurations()
{
  // read the planning configuration for each group
  planning_interface::PlannerConfigurationMap pconfig;
  pconfig.clear();

  for (const std::string& group_name : robot_model_->getJointModelGroupNames())
  {
    // the set of planning parameters that can be specific for the group (inherited by configurations of that group)
    // with their expected parameter type
    static const std::pair<std::string, rclcpp::ParameterType> KNOWN_GROUP_PARAMS[] = {
      { "projection_evaluator", rclcpp::ParameterType::PARAMETER_STRING },
      { "longest_valid_segment_fraction", rclcpp::ParameterType::PARAMETER_DOUBLE },
      { "enforce_joint_model_state_space", rclcpp::ParameterType::PARAMETER_BOOL },
      { "enforce_constrained_state_space", rclcpp::ParameterType::PARAMETER_BOOL }
    };

    const std::string group_name_param = parameter_namespace_ + "." + group_name;

    // get parameters specific for the robot planning group
    std::map<std::string, std::string> specific_group_params;
    for (const auto& [name, type] : KNOWN_GROUP_PARAMS)
    {
      std::string param_name{ group_name_param };
      param_name += ".";
      param_name += name;
      if (node_->has_parameter(param_name))
      {
        const rclcpp::Parameter parameter = node_->get_parameter(param_name);
        if (parameter.get_type() != type)
        {
          RCLCPP_ERROR_STREAM(LOGGER, "Invalid type for parameter '" << name << "' expected ["
                                                                     << rclcpp::to_string(type) << "] got ["
                                                                     << rclcpp::to_string(parameter.get_type()) << "]");
          continue;
        }
        if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
          specific_group_params[name] = parameter.as_string();
        else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
          specific_group_params[name] = moveit::core::toString(parameter.as_double());
        else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
          specific_group_params[name] = std::to_string(parameter.as_int());
        else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
          specific_group_params[name] = std::to_string(parameter.as_bool());
      }
    }

    // add default planner configuration
    planning_interface::PlannerConfigurationSettings default_pc;
    std::string default_planner_id;
    if (node_->get_parameter(group_name_param + ".default_planner_config", default_planner_id))
    {
      if (!loadPlannerConfiguration(group_name, default_planner_id, specific_group_params, default_pc))
      {
        default_planner_id = "";
      }
    }

    if (default_planner_id.empty())
    {
      default_pc.group = group_name;
      default_pc.config = specific_group_params;
      default_pc.config["type"] = "geometric::RRTConnect";
    }

    default_pc.name = group_name;  // this is the name of the default config
    pconfig[default_pc.name] = default_pc;

    // get parameters specific to each planner type
    std::vector<std::string> config_names;
    if (node_->get_parameter(group_name_param + ".planner_configs", config_names))
    {
      for (const auto& planner_id : config_names)
      {
        planning_interface::PlannerConfigurationSettings pc;
        if (loadPlannerConfiguration(group_name, planner_id, specific_group_params, pc))
        {
          pconfig[pc.name] = pc;
        }
      }
    }
  }

  for (const auto& [name, config_settings] : pconfig)
  {
    RCLCPP_DEBUG(LOGGER, "Parameters for configuration '%s'", name.c_str());

    for (const auto& [param_name, param_value] : config_settings.config)
    {
      RCLCPP_DEBUG_STREAM(LOGGER, " - " << param_name << " = " << param_value);
    }
  }

  setPlannerConfigurations(pconfig);
}

void OMPLInterface::printStatus()
{
  RCLCPP_INFO(LOGGER, "OMPL ROS interface is running.");
}
}  // namespace ompl_interface
