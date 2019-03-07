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

/* Author: Ioan Sucan, Jon Binney */

#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>

rclcpp::Logger logger_occupancy_map_updater = rclcpp::get_logger("occupancy_map_monitor");

namespace occupancy_map_monitor
{
OccupancyMapUpdater::OccupancyMapUpdater(const std::string& type) : type_(type)
{
}

OccupancyMapUpdater::~OccupancyMapUpdater()
{
}

void OccupancyMapUpdater::setMonitor(OccupancyMapMonitor* monitor)
{
  monitor_ = monitor;
  tree_ = monitor->getOcTreePtr();
}

void OccupancyMapUpdater::readXmlParam(const std::string& param_name, double* value)
{
  std::string isInt ("integer");
  auto node_occupancy_map = rclcpp::Node::make_shared("ocupancy_map_monitor_parameters");
  auto ocupancy_map_monitor_parameters = std::make_shared<rclcpp::SyncParametersClient>(node_occupancy_map);

  for (auto & parameter : ocupancy_map_monitor_parameters->get_parameters({param_name})) {
    if(isInt.compare(parameter.get_type_name()) == 0){
      *value = std::stoi(parameter.value_to_string());
    }else{
      *value = std::stod(parameter.value_to_string());
    }
  }

}

void OccupancyMapUpdater::readXmlParam(const std::string& param_name, unsigned int* value)
{
  auto node_occupancy_map = rclcpp::Node::make_shared("ocupancy_map_monitor_parameters");
  auto ocupancy_map_monitor_parameters = std::make_shared<rclcpp::SyncParametersClient>(node_occupancy_map);

  for (auto & parameter : ocupancy_map_monitor_parameters->get_parameters({param_name})) {
    *value = std::stoi(parameter.value_to_string());
  }
}

bool OccupancyMapUpdater::updateTransformCache(const std::string& target_frame, const rclcpp::Time& target_time)
{
  transform_cache_.clear();
  if (transform_provider_callback_)
    return transform_provider_callback_(target_frame, target_time, transform_cache_);
  else
  {
    // ROS_WARN_THROTTLE(1, "No callback provided for updating the transform cache for octomap updaters");
    while (true) {
      RCLCPP_ERROR(logger_occupancy_map_updater,"Incorrect mapping of mesh handles");
      boost::this_thread::sleep_for(boost::chrono::seconds(1));
    }
    return false;
  }
}
}
