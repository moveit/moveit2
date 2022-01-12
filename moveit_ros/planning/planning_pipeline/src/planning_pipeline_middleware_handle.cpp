/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik, Inc.
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
 *   * Neither the name of PickNik nor the names of its
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

/* Author: Anthony Baker */

#include <moveit/planning_pipeline/planning_pipeline_middleware_handle.hpp>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <rclcpp/rclcpp.hpp>
#include <boost/algorithm/string/join.hpp>
#include <string>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit.ros_planning.planning_pipeline_middleware_handle");

namespace planning_pipeline
{
namespace
{
static const auto SUBSCRIPTION_QOS = rclcpp::QoS(10);
}

PlanningPipelineMiddlewareHandle::PlanningPipelineMiddlewareHandle(const rclcpp::Node::SharedPtr& node) : node_(node)
{
}

void PlanningPipelineMiddlewareHandle::createDisplayPathPublisher(const std::string& topic)
{
  display_path_publisher_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>(topic, SUBSCRIPTION_QOS);
}

void PlanningPipelineMiddlewareHandle::createReceivedRequestPublisher(const std::string& topic)
{
  received_request_publisher_ = node_->create_publisher<moveit_msgs::msg::MotionPlanRequest>(topic, SUBSCRIPTION_QOS);
}

void PlanningPipelineMiddlewareHandle::createContactsPublisher(const std::string& topic)
{
  contacts_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(topic, SUBSCRIPTION_QOS);
}

std::string PlanningPipelineMiddlewareHandle::getContactsTopicName() const
{
  return contacts_publisher_->get_topic_name();
}

void PlanningPipelineMiddlewareHandle::resetDisplayPathPublisher()
{
  display_path_publisher_.reset();
}

void PlanningPipelineMiddlewareHandle::resetReceivedRequestPublisher()
{
  received_request_publisher_.reset();
}

void PlanningPipelineMiddlewareHandle::resetContactsPublisher()
{
  contacts_publisher_.reset();
}

bool PlanningPipelineMiddlewareHandle::has_parameter(const std::string& name) const
{
  return node_->has_parameter(name);
}

void PlanningPipelineMiddlewareHandle::get_parameters(const std::string& plugin_name, std::string& parameters) const
{
  node_->get_parameter(pluginName, plugin);
}

void PlanningPipelineMiddlewareHandle::createPlannerPlugin(const moveit::core::RobotModelConstPtr& robot_model)
{
  try
  {
    planner_plugin_loader_.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_FATAL(LOGGER, "Exception while creating planning plugin loader %s", ex.what());
  }

  const auto& classes = planner_plugin_loader_->getDeclaredClasses();
  if (classes.empty())
  {
    RCLCPP_ERROR(LOGGER, "No planning plugins available. Cannot create Planner Plugin.");
  }
  planner_plugin_name_ = classes.front();
  RCLCPP_INFO(LOGGER, "Using %s", planner_plugin_name_.c_str());

  try
  {
    planner_instance_ = planner_plugin_loader_->createUniqueInstance(planner_plugin_name_);
    if (!planner_instance_->initialize(robot_model, node_, parameter_namespace_))
    {
      throw std::runtime_error("Unable to initialize planning plugin");
    }
    RCLCPP_INFO(LOGGER, "Using planning interface '%s'", planner_instance_->getDescription().c_str());
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::string classes_str = boost::algorithm::join(classes, ", ");
    RCLCPP_ERROR(LOGGER,
                 "Exception while loading planner '%s': %s"
                 "Available plugins: %s",
                 planner_plugin_name_.c_str(), ex.what(), classes_str.c_str());
  }
}

const planning_interface::PlannerManagerPtr& PlanningPipelineMiddlewareHandle::getPlannerManager() const
{
  return planner_instance_;
}

void PlanningPipelineMiddlewareHandle::createAdapterPlugins(const std::vector<std::string>& name)
{  // load the planner request adapters
  if (!adapter_plugin_names_.empty())
  {
    std::vector<planning_request_adapter::PlanningRequestAdapterConstPtr> ads;
    try
    {
      adapter_plugin_loader_ =
            std::make_unique<pluginlib::ClassLoader<planning_request_adapter::PlanningRequestAdapter>>(
                "moveit_core", "planning_request_adapter::PlanningRequestAdapter");
    }
    catch (pluginlib::PluginlibException& ex)
    {
      RCLCPP_ERROR(LOGGER, "Exception while creating planning plugin loader %s", ex.what());
    }

    if (adapter_plugin_loader_)
    {
      for (const auto& adapter_plugin_name : adapter_plugin_names_)
      {
        try
        {
          auto adapter = adapter_plugin_loader_->createUniqueInstance(adapter_plugin_name);
          ad->initialize(node_, parameter_namespace_);
          ads.push_back(std::move(ad));
        }
        catch (pluginlib::PluginlibException& ex)
        {
          RCLCPP_ERROR(LOGGER, "Exception while loading planning adapter plugin '%s': %s", adapter_plugin_name.c_str(),
                       ex.what());
        }
      }
    }
    if (!ads.empty())
    {
      adapter_chain_.reset(new planning_request_adapter::PlanningRequestAdapterChain());
      for (planning_request_adapter::PlanningRequestAdapterConstPtr& ad : ads)
      {
        RCLCPP_INFO(LOGGER, "Using planning request adapter '%s'", ad->getDescription().c_str());
        adapter_chain_->addAdapter(ad);
      }
    }
  }
}

bool PlanningPipelineMiddlewareHandle::plan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                            const planning_interface::MotionPlanRequest& req,
                                            planning_interface::MotionPlanResponse& res,
                                            std::vector<std::size_t>& adapter_added_state_index)
{
  bool isSolved = false;
  if (!planner_instance_)
  {
    RCLCPP_ERROR(LOGGER, "No planning plugin loaded. Cannot plan.");
    return false;
  }
  try
  {
    if (adapter_chain_)
    {
      isSolved = adapter_chain_->adaptAndPlan(planner_instance_, planning_scene, req, res, adapter_added_state_index);
      if (!adapter_added_state_index.empty())
      {
        std::stringstream ss;
        for (std::size_t added_index : adapter_added_state_index)
        {
          ss << added_index << " ";
        }
        RCLCPP_INFO(LOGGER, "Planning adapters have added states at index positions: [ %s]", ss.str().c_str());
      }
    }
    else
    {
      planning_interface::PlanningContextPtr context =
          planner_instance_->getPlanningContext(planning_scene, req, res.error_code_);
      isSolved = context ? context->solve(res) : false;
    }
  }
  catch (std::exception& ex)
  {
    RCLCPP_ERROR(LOGGER, "Exception caught: '%s'", ex.what());
    return false;
  }
  return isSolved;
}

void PlanningPipelineMiddlewareHandle::terminatePlannerPlugin()
{
  if (planner_instance_)
  {
    planner_instance_->terminate();
  }
}

void PlanningPipelineMiddlewareHandle::publishReceivedRequest(const planning_interface::MotionPlanRequest& req)
{
  received_request_publisher_->publish(req);
}
void PlanningPipelineMiddlewareHandle::publishContactsMarkerArray(const visualization_msgs::msg::MarkerArray& markerArray)
{
  contacts_publisher_->publish(markerArray);
}

void PlanningPipelineMiddlewareHandle::publishDisplayPathTrejectory(
    const moveit_msgs::msg::DisplayTrajectory& displayTrajectory)
{
  display_path_publisher_->publish(displayTrajectory);
}

}  // namespace planning_pipeline
