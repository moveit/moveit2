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

#pragma once

#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <moveit/occupancy_map_monitor/occupancy_map_updater.h>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace occupancy_map_monitor
{
/**
 * @brief      This class contains the ros interfaces for OccupancyMapMontior
 */
class OccupancyMapMonitorMiddlewareHandle : public OccupancyMapMonitor::MiddlewareHandle
{
public:
  /**
   * @brief      Constructor, reads the parameters.
   *
   * @param[in]  node            The ros node
   * @param[in]  map_resolution  The map resolution, if not > 0 read from a parameter
   * @param[in]  map_frame       The map frame, if not empty read from a parameter
   */
  OccupancyMapMonitorMiddlewareHandle(const rclcpp::Node::SharedPtr& node, double map_resolution,
                                      const std::string& map_frame);

  /**
   * @brief      Gets the parameters struct.
   *
   * @return     The parameters.
   */
  OccupancyMapMonitor::Parameters getParameters() const override;

  /**
   * @brief      Loads an occupancy map updater using pluginlib.
   *
   * @param[in]  sensor_plugin  The sensor plugin type string
   *
   * @return     The occupancy map updater pointer.
   */
  OccupancyMapUpdaterPtr loadOccupancyMapUpdater(const std::string& sensor_plugin) override;

  /**
   * @brief      Initializes the occupancy map updater.  This must be called because of the interface of OccupancyMapUpdater.
   *
   * @param[in]  occupancy_map_updater  The occupancy map updater
   */
  void initializeOccupancyMapUpdater(OccupancyMapUpdaterPtr occupancy_map_updater) override;

  /**
   * @brief      Creates a save map service.
   *
   * @param[in]  callback  The callback closure
   */
  void createSaveMapService(OccupancyMapMonitor::MiddlewareHandle::SaveMapServiceCallback callback) override;

  /**
   * @brief      Creates a load map service.
   *
   * @param[in]  callback  The callback closure
   */
  void createLoadMapService(OccupancyMapMonitor::MiddlewareHandle::LoadMapServiceCallback callback) override;

private:
  rclcpp::Node::SharedPtr node_;                                       /*!< ROS node */
  rclcpp::Service<moveit_msgs::srv::SaveMap>::SharedPtr save_map_srv_; /*!< ROS service created by createSaveMapService */
  rclcpp::Service<moveit_msgs::srv::LoadMap>::SharedPtr load_map_srv_; /*!< ROS service created by createLoadMapService */
  std::unique_ptr<pluginlib::ClassLoader<OccupancyMapUpdater>>
      updater_plugin_loader_; /*!< Pluginlib loader for OccupancyMapUpdater */

  OccupancyMapMonitor::Parameters parameters_; /*!< ROS parameters for OccupancyMapMonitor */
  rclcpp::Logger logger_;
};

}  // namespace occupancy_map_monitor
