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

/* Author: Tyler Weaver */

#include <moveit/occupancy_map_monitor/occupancy_map_monitor_middleware_handle.hpp>
#include <moveit/occupancy_map_monitor/occupancy_map_updater.h>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace occupancy_map_monitor
{
namespace
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit.ros.occupancy_map_monitor.middleware_handle");
}

OccupancyMapMonitorMiddlewareHandle::OccupancyMapMonitorMiddlewareHandle(const rclcpp::Node::SharedPtr& node,
                                                                         double map_resolution,
                                                                         const std::string& map_frame)
  : node_{ node }, map_resolution_{ map_resolution }, map_frame_{ map_frame }, sensor_plugins_{}
{
  try
  {
    updater_plugin_loader_ = std::make_unique<pluginlib::ClassLoader<OccupancyMapUpdater>>(
        "moveit_ros_occupancy_map_monitor", "occupancy_map_monitor::OccupancyMapUpdater");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_FATAL_STREAM(LOGGER, "Exception while creating octomap updater plugin loader " << ex.what());
    throw ex;
  }

  if (map_resolution_ <= std::numeric_limits<double>::epsilon())
  {
    if (!node_->get_parameter("octomap_resolution", map_resolution_))
    {
      map_resolution_ = 0.1;
      RCLCPP_WARN(LOGGER, "Resolution not specified for Octomap. Assuming resolution = %g instead", map_resolution_);
    }
  }

  if (map_frame.empty())
  {
    node_->get_parameter("octomap_frame", map_frame_);
  }

  std::vector<std::string> sensor_names;
  if (!node_->get_parameter("sensors", sensor_names))
  {
    RCLCPP_ERROR(LOGGER, "No 3D sensor plugin(s) defined for octomap updates");
  }
  else if (sensor_names.empty())
  {
    RCLCPP_ERROR(LOGGER, "List of sensors is empty!");
  }

  for (const auto& sensor_name : sensor_names)
  {
    std::string sensor_plugin = "";
    if (!node_->get_parameter(sensor_name + ".sensor_plugin", sensor_plugin))
    {
      RCLCPP_ERROR(LOGGER, "No sensor plugin specified for octomap updater %s; ignoring.", sensor_name.c_str());
    }

    if (sensor_plugin.empty() || sensor_plugin[0] == '~')
    {
      RCLCPP_INFO_STREAM(LOGGER, "Skipping octomap updater plugin '" << sensor_plugin << "'");
      continue;
    }
    else
    {
      sensor_plugins_.push_back(std::make_pair(sensor_name, sensor_plugin));
    }
  }
}

double OccupancyMapMonitorMiddlewareHandle::getMapResolutionParameter() const
{
  return map_resolution_;
}

std::string OccupancyMapMonitorMiddlewareHandle::getMapFrameParameter() const
{
  return map_frame_;
}

std::vector<std::pair<std::string, std::string>> OccupancyMapMonitorMiddlewareHandle::getSensorPluginsParameter() const
{
  return sensor_plugins_;
}

OccupancyMapUpdaterPtr OccupancyMapMonitorMiddlewareHandle::loadOccupancyMapUpdater(const std::string& sensor_plugin)
{
  try
  {
    return updater_plugin_loader_->createUniqueInstance(sensor_plugin);
  }
  catch (pluginlib::PluginlibException& exception)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Exception while loading octomap updater '" << sensor_plugin
                                                                            << "': " << exception.what() << std::endl);
  }
  return nullptr;
}

void OccupancyMapMonitorMiddlewareHandle::initializeOccupancyMapUpdater(OccupancyMapUpdaterPtr occupancy_map_updater)
{
  if (!occupancy_map_updater->initialize(node_))
  {
    RCLCPP_ERROR(LOGGER, "Unable to initialize map updater of type %s", occupancy_map_updater->getType().c_str());
  }
}

void OccupancyMapMonitorMiddlewareHandle::createSaveMapService(
    OccupancyMapMonitor::MiddlewareHandle::SaveMapServiceCallback callback)
{
  save_map_srv_ = node_->create_service<moveit_msgs::srv::SaveMap>("save_map", callback);
}

void OccupancyMapMonitorMiddlewareHandle::createLoadMapService(
    OccupancyMapMonitor::MiddlewareHandle::LoadMapServiceCallback callback)
{
  load_map_srv_ = node_->create_service<moveit_msgs::srv::LoadMap>("load_map", callback);
}

}  // namespace occupancy_map_monitor
