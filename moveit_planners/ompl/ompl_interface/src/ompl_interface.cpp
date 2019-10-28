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

ompl_interface::OMPLInterface::OMPLInterface(const std::shared_ptr<const robot_model::RobotModel>& robot_model,
                                             const rclcpp::Node::SharedPtr& node)
  : node_(node)
  , robot_model_(robot_model)
  , constraint_sampler_manager_(new constraint_samplers::ConstraintSamplerManager())
  , context_manager_(robot_model, constraint_sampler_manager_)
  , constraints_library_(new ConstraintsLibrary(context_manager_))
  , use_constraints_approximations_(true)
  , simplify_solutions_(true)
{
  RCLCPP_INFO(node_->get_logger(), "Initializing OMPL interface using ROS parameters");
  loadPlannerConfigurations();
  loadConstraintApproximations();
  loadConstraintSamplers();
}

ompl_interface::OMPLInterface::OMPLInterface(const std::shared_ptr<const robot_model::RobotModel>& robot_model,
                                             const planning_interface::PlannerConfigurationMap& pconfig,
                                             const rclcpp::Node::SharedPtr& node)
  : node_(node)
  , robot_model_(robot_model)
  , constraint_sampler_manager_(new constraint_samplers::ConstraintSamplerManager())
  , context_manager_(robot_model, constraint_sampler_manager_)
  , constraints_library_(new ConstraintsLibrary(context_manager_))
  , use_constraints_approximations_(true)
  , simplify_solutions_(true)
{
  RCLCPP_INFO(node_->get_logger(), "Initializing OMPL interface using specified configuration");
  setPlannerConfigurations(pconfig);
  loadConstraintApproximations();
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
  ModelBasedPlanningContextPtr ctx = context_manager_.getPlanningContext(planning_scene, req, error_code);
  if (ctx)
    configureContext(ctx);
  return ctx;
}

ompl_interface::ModelBasedPlanningContextPtr
ompl_interface::OMPLInterface::getPlanningContext(const std::string& config, const std::string& factory_type) const
{
  ModelBasedPlanningContextPtr ctx = context_manager_.getPlanningContext(config, factory_type);
  if (ctx)
    configureContext(ctx);
  return ctx;
}

void ompl_interface::OMPLInterface::configureContext(const ModelBasedPlanningContextPtr& context) const
{
  if (use_constraints_approximations_)
    context->setConstraintsApproximations(constraints_library_);
  else
    context->setConstraintsApproximations(ConstraintsLibraryPtr());
  context->simplifySolutions(simplify_solutions_);
}

void ompl_interface::OMPLInterface::loadConstraintApproximations(const std::string& path)
{
  constraints_library_->loadConstraintApproximations(path);
  std::stringstream ss;
  constraints_library_->printConstraintApproximations(ss);
  RCLCPP_INFO(node_->get_logger(), ss.str());
}

void ompl_interface::OMPLInterface::saveConstraintApproximations(const std::string& path)
{
  constraints_library_->saveConstraintApproximations(path);
}

bool ompl_interface::OMPLInterface::saveConstraintApproximations()
{
  auto constraint_approximations_path_parameter = std::make_shared<rclcpp::SyncParametersClient>(node_);

  if (constraint_approximations_path_parameter->has_parameter("constraint_approximations_path"))
  {
    std::string cpath = node_->get_parameter("constraint_approximations_path").get_value<std::string>();
    saveConstraintApproximations(cpath);
    return true;
  }
  RCLCPP_WARN(node_->get_logger(),
              "ROS param 'constraint_approximations' not found. Unable to save constraint approximations");
  return false;
}

bool ompl_interface::OMPLInterface::loadConstraintApproximations()
{
  auto constraint_approximations_path_parameter = std::make_shared<rclcpp::SyncParametersClient>(node_);

  if (constraint_approximations_path_parameter->has_parameter("constraint_approximations_path"))
  {
    std::string cpath = node_->get_parameter("constraint_approximations_path").get_value<std::string>();
    loadConstraintApproximations(cpath);
    return true;
  }
  return false;
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
  auto planner_config_parameter = std::make_shared<rclcpp::SyncParametersClient>(node_);
  auto planner_config_param_prefix = planner_config_parameter->list_parameters({"planner_configs"},10);

  if (!planner_config_parameter->has_parameter("planner_configs/" + planner_id))
  {
    RCLCPP_ERROR(node_->get_logger(), "Could not find the planner configuration '%s' on the param server",
                 planner_id.c_str());
    return false;
  }
  // TODO (anasarrak): Adapt for ros2 parameters
  // if (xml_config.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  // {
  //   RCLCPP_ERROR(node_->get_logger(),"A planning configuration should be of type XmlRpc Struct type (for
  //   configuration '%s')",
  //             planner_id.c_str());
  //   return false;
  // }

  planner_config.name = group_name + "[" + planner_id + "]";
  planner_config.group = group_name;

  // default to specified parameters of the group (overridden by configuration specific parameters)
  planner_config.config = group_params;

  // read parameters specific for this configuration
    for (auto & name : planner_config_param_prefix.names) {
      std::string type = node_->get_parameter(name).get_type_name();
      std::istringstream iss(name);
      std::vector<std::string> indexes;
      std::string index;
      while (std::getline(iss, index, '.')) {
      if (!index.empty())
          indexes.push_back(index);
      }
      if(type.find("string") != std::string::npos){
        auto s_index  = node_->get_parameter(name).as_string();
        planner_config.config[indexes[2]] = static_cast<std::string>(s_index);
      }
      if(type.find("double") != std::string::npos){
        auto d_index  = node_->get_parameter(name).as_double();
        planner_config.config[indexes[2]] = moveit::core::toString(static_cast<double>(d_index));
      }
      if(type.find("int") != std::string::npos){
        auto i_index  = node_->get_parameter(name).as_int();
        planner_config.config[indexes[2]] = std::to_string(static_cast<int>(i_index));
      }
      if(type.find("boolean") != std::string::npos){
        auto b_index  = node_->get_parameter(name).as_bool();
        planner_config.config[indexes[2]] = std::to_string(static_cast<bool>(b_index));
      }
  }

  return true;
}

void ompl_interface::OMPLInterface::loadPlannerConfigurations()
{
  // read the planning configuration for each group
  planning_interface::PlannerConfigurationMap pconfig;
  pconfig.clear();
  auto known_group_parameters = std::make_shared<rclcpp::SyncParametersClient>(node_);
  for (const std::string& group_name : robot_model_->getJointModelGroupNames())
  {
    // the set of planning parameters that can be specific for the group (inherited by configurations of that group)
    static const std::string KNOWN_GROUP_PARAMS[] = { "projection_evaluator", "longest_valid_segment_fraction",
                                                      "enforce_joint_model_state_space" };

    // get parameters specific for the robot planning group
    std::map<std::string, std::string> specific_group_params;
    for (const std::string& k : KNOWN_GROUP_PARAMS)
    {
      if (known_group_parameters->has_parameter(group_name + "/" + k))
      {
        if (node_->get_parameter(group_name + "/" + k).get_type_name().compare("string") == 0)
        {
          std::string value = node_->get_parameter(group_name + "/" + k).get_value<std::string>();
          if (!value.empty())
            specific_group_params[k] = value;
          continue;
        }

        if (node_->get_parameter(group_name + "/" + k).get_type_name().compare("double") == 0)
        {
          double value_d = node_->get_parameter(group_name + "/" + k).get_value<double>();
          // convert to string using no locale
          specific_group_params[k] = moveit::core::toString(value_d);
          continue;
        }

        if (node_->get_parameter(group_name + "/" + k).get_type_name().compare("integer") == 0)
        {
          int value_i = node_->get_parameter(group_name + "/" + k).get_value<int>();
          specific_group_params[k] = std::to_string(value_i);
          continue;
        }

        if (node_->get_parameter(group_name + "/" + k).get_type_name().compare("bool") == 0)
        {
          bool value_b = node_->get_parameter(group_name + "/" + k).get_value<bool>();
          specific_group_params[k] = std::to_string(value_b);
          continue;
        }
      }
    }

    // add default planner configuration
    planning_interface::PlannerConfigurationSettings default_pc;
    std::string default_planner_id;
    if (known_group_parameters->has_parameter(group_name + "/default_planner_config"))
    {
      default_planner_id = node_->get_parameter(group_name + "/default_planner_config").get_value<std::string>();
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
    // XmlRpc::XmlRpcValue config_names;

    if (known_group_parameters->has_parameter(group_name + "/planner_configs"))
    {
      std::vector<std::string> config_names = node_->get_parameter(group_name + "/planner_configs").as_string_array();
      auto config_type = node_->get_parameter(group_name + "/planner_configs").get_type_name();
      if (config_type.find("array") == std::string::npos)
      {
        RCLCPP_ERROR(node_->get_logger(), "The planner_configs argument of a group configuration "
                                          "should be an array of strings (for group '%s')",
                     group_name.c_str());
        continue;
      }

      for (int j = 0; j < config_names.size(); ++j)
      {
        std::string type = typeid(config_names[j]).name();
        if (type.find("string") == std::string::npos)
        {
          RCLCPP_ERROR(node_->get_logger(), "Planner configuration names must be of type string (for group '%s')",
                       group_name.c_str());
          continue;
        }

        const std::string planner_id = static_cast<std::string>(config_names[j]);

        planning_interface::PlannerConfigurationSettings pc;
        if (loadPlannerConfiguration(group_name, planner_id, specific_group_params, pc))
          pconfig[pc.name] = pc;
      }
    }
  }

  for (const std::pair<std::string, planning_interface::PlannerConfigurationSettings>& config : pconfig)
  {
    RCLCPP_DEBUG(node_->get_logger(), "Parameters for configuration '%s'", config.first.c_str());

    for (const std::pair<std::string, std::string>& parameters : config.second.config)
    {
      RCLCPP_DEBUG(node_->get_logger(), "parameters - %s = %s", parameters.first.c_str(), parameters.second.c_str());
    }
  }

  setPlannerConfigurations(pconfig);
}

void ompl_interface::OMPLInterface::printStatus()
{
  RCLCPP_INFO(node_->get_logger(), "OMPL ROS interface is running.");
}
