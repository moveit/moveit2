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
#include <string>
#include <rsl/random.hpp>
#include <fmt/format.h>

namespace moveit
{

// This is the function that stores the global logger used by moveit.
// As it returns a reference to the static logger it can be changed through the
// `setNodeLoggerName` function.
rclcpp::Logger& getGlobalRootLogger()
{
  static rclcpp::Logger logger = [&] {
    // A random number is appended to the name used for the node to make it unique.
    // This unique node and logger name is only used if a user does not set a logger
    // through the `setNodeLoggerName` method to their node's logger.
    auto name = fmt::format("moveit_{}", rsl::rng()());
    try
    {
      static auto* moveit_node = new rclcpp::Node(name);
      return moveit_node->get_logger();
    }
    catch (const std::exception& ex)
    {
      // rclcpp::init was not called so rcl context is null, return non-node logger
      auto logger = rclcpp::get_logger(name);
      RCLCPP_WARN_STREAM(logger, "exception thrown while creating node for logging: " << ex.what());
      RCLCPP_WARN(logger, "if rclcpp::init was not called, messages from this logger may be missing from /rosout");
      return logger;
    }
  }();
  return logger;
}

void setNodeLoggerName(const std::string& name)
{
  static auto node = std::make_shared<rclcpp::Node>("moveit", name);
  getGlobalRootLogger() = node->get_logger();
}

rclcpp::Logger getLogger(const std::string& name)
{
  return getGlobalRootLogger().get_child(name);
}

}  // namespace moveit
