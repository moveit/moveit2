/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, PickNik Robotics Inc.
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

/* Author: Tyler Weaver */

#include <rclcpp/rclcpp.hpp>
#include <moveit/utils/logger.hpp>
#include <rclcpp/version.h>
#include <unordered_map>
#include <string>

namespace moveit
{

const rclcpp::Logger& getLogger()
{
  return getLoggerMut();
}

rclcpp::Logger makeChildLogger(const std::string& name)
{
  // On versions of ROS older than Iron we need to create a node for each child logger
  // Remove once Humble is EOL
  // References:
  // Use parent logger (rcl PR) - https://github.com/ros2/rcl/pull/921
  // Request for backport (rclpy issue) - https://github.com/ros2/rclpy/issues/1131
  // MoveIt PR that added this - https://github.com/ros-planning/moveit2/pull/2445
#if !RCLCPP_VERSION_GTE(21, 0, 3)
  static std::unordered_map<std::string, std::shared_ptr<rclcpp::Node>> child_nodes;
  if (child_nodes.find(name) == child_nodes.end())
  {
    std::string ns = getLogger().get_name();
    child_nodes[name] = std::make_shared<rclcpp::Node>(name, ns);
  }
#endif

  return getLoggerMut().get_child(name);
}

rclcpp::Logger& getLoggerMut()
{
  static rclcpp::Logger logger = rclcpp::get_logger("moveit");
  return logger;
}

}  // namespace moveit
