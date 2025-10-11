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

/* Author: Jon Binney, Ioan Sucan */

#include <rclcpp/version.h>

#include <moveit/occupancy_map_monitor/occupancy_map_monitor.hpp>

#include <octomap_msgs/conversions.h>
// For Rolling, Kilted, and newer
#if RCLCPP_VERSION_GTE(29, 6, 0)
#include <tf2_ros/transform_listener.hpp>
// For Jazzy and older
#else
#include <tf2_ros/transform_listener.h>
#endif
#include <rclcpp/clock.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/version.h>
#include <moveit/utils/logger.hpp>
#if RCLCPP_VERSION_GTE(20, 0, 0)
#include <rclcpp/event_handler.hpp>
#else
#include <rclcpp/qos_event.hpp>
#endif
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <memory>
#include <sstream>

static void publishOctomap(const rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr& octree_binary_pub,
                           occupancy_map_monitor::OccupancyMapMonitor& server)
{
  octomap_msgs::msg::Octomap map;
  const auto logger = moveit::getLogger("moveit.ros.occupancy_map_monitor");

  map.header.frame_id = server.getMapFrame();
  map.header.stamp = rclcpp::Clock().now();

  server.getOcTreePtr()->lockRead();
  rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  try
  {
    if (!octomap_msgs::binaryMapToMsgData(*server.getOcTreePtr(), map.data))
    {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wold-style-cast"
      RCLCPP_ERROR_THROTTLE(logger, steady_clock, 1000, "Could not generate OctoMap message");
#pragma GCC diagnostic pop
    }
  }
  catch (...)
  {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wold-style-cast"
    RCLCPP_ERROR_THROTTLE(logger, steady_clock, 1000, "Exception thrown while generating OctoMap message");
#pragma GCC diagnostic pop
  }
  server.getOcTreePtr()->unlockRead();

  octree_binary_pub->publish(map);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("occupancy_map_server");
  moveit::setNodeLoggerName(node->get_name());
  auto octree_binary_pub =
      node->create_publisher<octomap_msgs::msg::Octomap>("octomap_binary", RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  auto clock_ptr = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  std::shared_ptr<tf2_ros::Buffer> buffer = std::make_shared<tf2_ros::Buffer>(clock_ptr, tf2::durationFromSec(5.0));
// For Rolling, L-turtle, and newer
#if RCLCPP_VERSION_GTE(30, 0, 0)
  tf2_ros::TransformListener listener(*buffer,
                                      tf2_ros::TransformListener::RequiredInterfaces{
                                          node->get_node_base_interface(), node->get_node_logging_interface(),
                                          node->get_node_parameters_interface(), node->get_node_topics_interface() },
                                      false  // spin_thread
  );
#else
  tf2_ros::TransformListener listener(*buffer, node, false /* spin_thread - disables executor */);
#endif
  occupancy_map_monitor::OccupancyMapMonitor server(node, buffer);
  server.setUpdateCallback(
      [&octree_binary_pub, &server, logger = node->get_logger()] { return publishOctomap(octree_binary_pub, server); });
  server.startMonitor();

  rclcpp::spin(node);
  return 0;
}
