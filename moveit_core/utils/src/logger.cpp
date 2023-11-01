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
// `setLogger` function.
//
// You can get a const reference to the logger through the public function
// `getLogger`.
//
// As there is no way to know if `getLogger` is called before `setLogger`
// we try to initialize a node that will setup /rosout logging with this logger.
// If that fails (likely due to rclcpp::init not haven't been called) we set the
// global logger to one that is not associated with a node. When a logger not
// associated with a node is used the logs only go to the console and files,
// they do not go to /rosout. This is because publishing is done by a node.
//
// The node and logger created here is not intended to be used. But if it is,
// we append a random number to the name of the node name and logger to make it
// unique. This helps to prevent problems that arise from when multiple
// nodes use the same name.
rclcpp::Logger& getGlobalLoggerRef()
{
  static rclcpp::Logger logger = [&] {
    // A random number is appended to the name used for the node to make it unique.
    // This unique node and logger name is only used if a user does not set a logger
    // through the `setLogger` method to their node's logger.
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

const rclcpp::Logger& getLogger()
{
  return getGlobalLoggerRef();
}

rclcpp::Logger makeChildLogger(const std::string& name)
{
  RCLCPP_INFO_STREAM(getLogger(), "Creating child logger: " << name);
  return getGlobalLoggerRef().get_child(name);
}

void setLogger(const rclcpp::Logger& logger)
{
  getGlobalLoggerRef() = logger;
}

}  // namespace moveit
