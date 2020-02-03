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
#include <moveit/profiler/profiler.h>
#include <moveit/utils/lexical_casts.h>
#include <fstream>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_ompl_planning.ompl_interface");

ompl_interface::OMPLInterface::OMPLInterface(const robot_model::RobotModelConstPtr& robot_model,
                                             const rclcpp::Node::SharedPtr& node,
                                             const std::string& parameter_namespace)
  : node_(node)
  , parameter_namespace_(parameter_namespace)
  , robot_model_(robot_model)
  , constraint_sampler_manager_(new constraint_samplers::ConstraintSamplerManager())
  , context_manager_(robot_model, constraint_sampler_manager_)
  , use_constraints_approximations_(true)
  , simplify_solutions_(true)
{
  RCLCPP_INFO(LOGGER, "Initializing OMPL interface using ROS parameters");
  loadPlannerConfigurations();
  loadConstraintSamplers();
}

ompl_interface::OMPLInterface::OMPLInterface(const robot_model::RobotModelConstPtr& robot_model,
                                             const planning_interface::PlannerConfigurationMap& pconfig,
                                             const rclcpp::Node::SharedPtr& node,
                                             const std::string& parameter_namespace)
  : node_(node)
  , parameter_namespace_(parameter_namespace)
  , robot_model_(robot_model)
  , constraint_sampler_manager_(new constraint_samplers::ConstraintSamplerManager())
  , context_manager_(robot_model, constraint_sampler_manager_)
  , use_constraints_approximations_(true)
  , simplify_solutions_(true)
{
  RCLCPP_INFO(LOGGER, "Initializing OMPL interface using specified configuration");
  setPlannerConfigurations(pconfig);
  loadConstraintSamplers();
}

ompl_interface::OMPLInterface::~OMPLInterface() = default;

void ompl_interface::OMPLInterface::setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pconfig)
{
  planning_interface::PlannerConfigurationMap pconfig2 = pconfig;

  // construct default configurations for planning groups that don't have configs already passed in
  for (const robot_model::JointModelGroup* group : robot_model_->getJointModelGroups())
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

ompl_interface::ModelBasedPlanningContextPtr ompl_interface::OMPLInterface::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene, const planning_interface::MotionPlanRequest& req) const
{
  moveit_msgs::msg::MoveItErrorCodes dummy;
  return getPlanningContext(planning_scene, req, dummy);
}

ompl_interface::ModelBasedPlanningContextPtr
ompl_interface::OMPLInterface::getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                  const planning_interface::MotionPlanRequest& req,
                                                  moveit_msgs::msg::MoveItErrorCodes& error_code) const
{
  ModelBasedPlanningContextPtr ctx =
      context_manager_.getPlanningContext(planning_scene, req, error_code, node_, use_constraints_approximations_);
  if (ctx)
    configureContext(ctx);
  return ctx;
}

void ompl_interface::OMPLInterface::configureContext(const ModelBasedPlanningContextPtr& context) const
{
  context->simplifySolutions(simplify_solutions_);
}

void ompl_interface::OMPLInterface::loadConstraintSamplers()
{
  constraint_sampler_manager_loader_.reset(
      new constraint_sampler_manager_loader::ConstraintSamplerManagerLoader(node_, constraint_sampler_manager_));
}

bool ompl_interface::OMPLInterface::loadPlannerConfiguration(
    const std::string& group_name, const std::string& planner_id,
    const std::map<std::string, std::string>& group_params,
    planning_interface::PlannerConfigurationSettings& planner_config)
{
  rcl_interfaces::msg::ListParametersResult planner_params_result = node_->list_parameters(
      { parameter_namespace_ + ".planner_configs." + planner_id }, 1);  // TODO(henningkayser): verify search depth

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
    // TODO(henningkayser): verify name is working for config map
    const rclcpp::Parameter param = node_->get_parameter(parameter_namespace_ + "." + planner_param);
    planner_config.config[param.get_name()] = param.value_to_string();
  }

  return true;
}

void ompl_interface::OMPLInterface::loadPlannerConfigurations()
{
  // read the planning configuration for each group
  planning_interface::PlannerConfigurationMap pconfig;
  pconfig.clear();

  for (const std::string& group_name : robot_model_->getJointModelGroupNames())
  {
    // the set of planning parameters that can be specific for the group (inherited by configurations of that group)
    static const std::string KNOWN_GROUP_PARAMS[] = { "projection_evaluator", "longest_valid_segment_fraction",
                                                      "enforce_joint_model_state_space" };
    const std::string group_name_param = parameter_namespace_ + "." + group_name;

    // get parameters specific for the robot planning group
    std::map<std::string, std::string> specific_group_params;
    for (const std::string& k : KNOWN_GROUP_PARAMS)
    {
      if (node_->has_parameter(group_name_param + "." + k))
      {
        std::string value;
        if (node_->get_parameter(group_name_param + "." + k, value))
        {
          if (!value.empty())
            specific_group_params[k] = value;
          continue;
        }

        double value_d;
        if (node_->get_parameter(group_name_param + "." + k, value_d))
        {
          // convert to string using no locale
          specific_group_params[k] = moveit::core::toString(value_d);
          continue;
        }

        int value_i;
        if (node_->get_parameter(group_name_param + "." + k, value_i))
        {
          specific_group_params[k] = std::to_string(value_i);
          continue;
        }

        bool value_b;
        if (node_->get_parameter(group_name_param + "." + k, value_b))
        {
          specific_group_params[k] = std::to_string(value_b);
          continue;
        }
      }
    }

    // add default planner configuration
    planning_interface::PlannerConfigurationSettings default_pc;
    std::string default_planner_id;
    if (node_->get_parameter(group_name_param + ".default_planner_config", default_planner_id))
    {
      if (!loadPlannerConfiguration(group_name, default_planner_id, specific_group_params, default_pc))
        default_planner_id = "";
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
    if (node_->get_parameter(group_name + ".planner_configs", config_names))
    {
      for (const auto& planner_id : config_names)
      {
        planning_interface::PlannerConfigurationSettings pc;
        if (loadPlannerConfiguration(group_name, planner_id, specific_group_params, pc))
          pconfig[pc.name] = pc;
      }
    }
  }

  for (const std::pair<const std::string, planning_interface::PlannerConfigurationSettings>& config : pconfig)
  {
    RCLCPP_DEBUG(LOGGER, "Parameters for configuration '%s'", config.first);

    for (const std::pair<const std::string, std::string>& parameters : config.second.config)
      RCLCPP_DEBUG(LOGGER, " - %s = %s", parameters.first, parameters.second);
  }

  setPlannerConfigurations(pconfig);
}

void ompl_interface::OMPLInterface::printStatus()
{
  RCLCPP_INFO(LOGGER, "OMPL ROS interface is running.");
}
