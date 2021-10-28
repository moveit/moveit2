/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
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

/* Author: Sebastian Jahr
 */

#include <moveit/hybrid_planning_manager/hybrid_planning_manager.h>
#include <moveit/hybrid_planning_manager/hybrid_planning_events.h>

using namespace std::chrono_literals;

namespace moveit::hybrid_planning
{
namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("hybrid_planning_manager");
}

HybridPlanningManager::HybridPlanningManager(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("global_planner_component", options) }
{
  if (!initialize())
  {
    throw std::runtime_error("Failed to initialize global planner");
  }
}

bool HybridPlanningManager::initialize()
{
  // Create planning logic plugin loader
  try
  {
    planner_logic_plugin_loader_ = std::make_unique<pluginlib::ClassLoader<PlannerLogicInterface>>(
        "moveit_hybrid_planning", "moveit::hybrid_planning::PlannerLogicInterface");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(LOGGER, "Exception while creating planner logic plugin loader '%s'", ex.what());
    return false;
  }

  // TODO(sjahr) Refactor parameter declaration and use repository wide solution
  std::string logic_plugin_name = "";
  if (node_->has_parameter("planner_logic_plugin_name"))
  {
    node_->get_parameter<std::string>("planner_logic_plugin_name", logic_plugin_name);
  }
  else
  {
    logic_plugin_name = node_->declare_parameter<std::string>("planner_logic_plugin_name",
                                                              "moveit_hybrid_planning/ReplanInvalidatedTrajectory");
  }

  // Create planning logic plugin instance
  try
  {
    planner_logic_instance_ = planner_logic_plugin_loader_->createSharedInstance(logic_plugin_name);
    if (!planner_logic_instance_->initialize(hybrid_planning_interface_))
    {
      throw std::runtime_error("Unable to initialize planner logic plugin");
    }
    RCLCPP_INFO(LOGGER, "Using planner logic interface '%s'", logic_plugin_name.c_str());
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(LOGGER, "Exception while loading planner logic '%s': '%s'", logic_plugin_name.c_str(), ex.what());
    return false;
  }
  return true;

  // Create hybrid planning interface
  hybrid_planning_interface_ = std::make_shared<HybridPlanningInterface>(node_, planner_logic_instance_);
}
}  // namespace moveit::hybrid_planning

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit::hybrid_planning::HybridPlanningManager)
