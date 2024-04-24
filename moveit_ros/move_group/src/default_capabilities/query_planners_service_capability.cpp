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

/* Author: Ioan Sucan, Robert Haschke */

#include "query_planners_service_capability.h"
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/utils/logger.hpp>

namespace move_group
{
namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.ros.move_group.query_planners_service_capability");
}
}  // namespace
MoveGroupQueryPlannersService::MoveGroupQueryPlannersService() : MoveGroupCapability("query_planners_service")
{
}

void MoveGroupQueryPlannersService::initialize()
{
  query_service_ = context_->moveit_cpp_->getNode()->create_service<moveit_msgs::srv::QueryPlannerInterfaces>(
      QUERY_PLANNERS_SERVICE_NAME,
      [this](const std::shared_ptr<moveit_msgs::srv::QueryPlannerInterfaces::Request>& req,
             const std::shared_ptr<moveit_msgs::srv::QueryPlannerInterfaces::Response>& res) {
        queryInterface(req, res);
      });

  get_service_ = context_->moveit_cpp_->getNode()->create_service<moveit_msgs::srv::GetPlannerParams>(
      GET_PLANNER_PARAMS_SERVICE_NAME,
      [this](const std::shared_ptr<moveit_msgs::srv::GetPlannerParams::Request>& req,
             const std::shared_ptr<moveit_msgs::srv::GetPlannerParams::Response>& res) { getParams(req, res); });

  set_service_ = context_->moveit_cpp_->getNode()->create_service<moveit_msgs::srv::SetPlannerParams>(
      SET_PLANNER_PARAMS_SERVICE_NAME,
      [this](const std::shared_ptr<moveit_msgs::srv::SetPlannerParams::Request>& req,
             const std::shared_ptr<moveit_msgs::srv::SetPlannerParams::Response>& res) { setParams(req, res); });
}

void MoveGroupQueryPlannersService::queryInterface(
    const std::shared_ptr<moveit_msgs::srv::QueryPlannerInterfaces::Request>& /* unused */,
    const std::shared_ptr<moveit_msgs::srv::QueryPlannerInterfaces::Response>& res)
{
  for (const auto& planning_pipelines : context_->moveit_cpp_->getPlanningPipelines())
  {
    const auto& planning_pipeline = planning_pipelines.second;

    // TODO(sjahr): Update for multiple planner plugins
    const auto& planner_plugin_names = planning_pipeline->getPlannerPluginNames();
    if (planner_plugin_names.empty())
    {
      RCLCPP_ERROR(getLogger(), "Pipeline '%s' does not have any planner plugins.", planning_pipelines.first.c_str());
      return;
    }
    const auto planner_interface = planning_pipeline->getPlannerManager(planner_plugin_names.at(0));
    if (!planner_interface)
    {
      RCLCPP_ERROR(getLogger(), "Requesting planner '%s' from '%s' returned a null pointer.",
                   planner_plugin_names.at(0).c_str(), planning_pipelines.first.c_str());
    }
    std::vector<std::string> algs;
    planner_interface->getPlanningAlgorithms(algs);
    moveit_msgs::msg::PlannerInterfaceDescription pi_desc;
    pi_desc.name = planner_interface->getDescription();
    pi_desc.pipeline_id = planning_pipelines.first;
    planner_interface->getPlanningAlgorithms(pi_desc.planner_ids);
    res->planner_interfaces.push_back(pi_desc);
  }
}

void MoveGroupQueryPlannersService::getParams(const std::shared_ptr<moveit_msgs::srv::GetPlannerParams::Request>& req,
                                              const std::shared_ptr<moveit_msgs::srv::GetPlannerParams::Response>& res)
{
  const planning_pipeline::PlanningPipelinePtr planning_pipeline = resolvePlanningPipeline(req->pipeline_id);
  if (!planning_pipeline)
  {
    RCLCPP_ERROR(getLogger(), "Pipeline '%s' does not exist.", req->pipeline_id.c_str());
    return;
  }

  // TODO(sjahr): Update for multiple planner plugins
  const auto& planner_plugin_names = planning_pipeline->getPlannerPluginNames();
  if (planner_plugin_names.empty())
  {
    RCLCPP_ERROR(getLogger(), "Pipeline '%s' does not have any planner plugins.", req->pipeline_id.c_str());
    return;
  }
  const auto planner_interface = planning_pipeline->getPlannerManager(planner_plugin_names.at(0));
  if (!planner_interface)
  {
    RCLCPP_ERROR(getLogger(), "Requesting planner '%s' from '%s' returned a null pointer.",
                 planner_plugin_names.at(0).c_str(), req->pipeline_id.c_str());
  }
  std::map<std::string, std::string> config;

  const planning_interface::PlannerConfigurationMap& configs = planner_interface->getPlannerConfigurations();

  planning_interface::PlannerConfigurationMap::const_iterator it =
      configs.find(req->planner_config);  // fetch default params first
  if (it != configs.end())
  {
    config.insert(it->second.config.begin(), it->second.config.end());
  }

  if (!req->group.empty())
  {  // merge in group-specific params
    it = configs.find(req->group + "[" + req->planner_config + "]");
    if (it != configs.end())
    {
      config.insert(it->second.config.begin(), it->second.config.end());
    }
  }

  for (const auto& key_value_pair : config)
  {
    res->params.keys.push_back(key_value_pair.first);
    res->params.values.push_back(key_value_pair.second);
  }
}

void MoveGroupQueryPlannersService::setParams(
    const std::shared_ptr<moveit_msgs::srv::SetPlannerParams::Request>& req,
    const std::shared_ptr<moveit_msgs::srv::SetPlannerParams::Response>& /*res*/)
{
  if (req->params.keys.size() != req->params.values.size())
  {
    RCLCPP_ERROR(getLogger(), "Number of parameter names does not match the number of parameters");
    return;
  }

  const planning_pipeline::PlanningPipelinePtr planning_pipeline = resolvePlanningPipeline(req->pipeline_id);
  if (!planning_pipeline)
  {
    RCLCPP_ERROR(getLogger(), "Pipeline '%s' does not exist.", req->pipeline_id.c_str());
    return;
  }

  // TODO(sjahr): Update for multiple planner plugins
  const auto& planner_plugin_names = planning_pipeline->getPlannerPluginNames();
  if (planner_plugin_names.empty())
  {
    RCLCPP_ERROR(getLogger(), "Pipeline '%s' does not have any planner plugins.", req->pipeline_id.c_str());
    return;
  }
  auto planner_interface = planning_pipeline->getPlannerManager(planner_plugin_names.at(0));
  if (!planner_interface)
  {
    RCLCPP_ERROR(getLogger(), "Requesting planner '%s' from '%s' returned a null pointer.",
                 planner_plugin_names.at(0).c_str(), req->pipeline_id.c_str());
    return;
  }

  planning_interface::PlannerConfigurationMap configs = planner_interface->getPlannerConfigurations();
  const std::string config_name =
      req->group.empty() ? req->planner_config : req->group + "[" + req->planner_config + "]";

  planning_interface::PlannerConfigurationSettings& config = configs[config_name];
  config.group = req->group;
  config.name = config_name;
  if (req->replace)
  {
    config.config.clear();
  }
  for (unsigned int i = 0, end = req->params.keys.size(); i < end; ++i)
  {
    config.config[req->params.keys.at(i)] = req->params.values.at(i);
  }

  planner_interface->setPlannerConfigurations(configs);
}
}  // namespace move_group

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(move_group::MoveGroupQueryPlannersService, move_group::MoveGroupCapability)
