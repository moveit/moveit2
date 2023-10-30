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
 *   * Neither the name of PickNik Robotics Inc. nor the names of its
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

#include <rclcpp/logger.hpp>
#include <string>

namespace moveit
{

// Get reference to global logger.
// Use `setLogger` to set the logger to a node logger.
// This can be used in place in macro logger lines. For example:
// ```c++
// RCLCPP_WARN(moveit::getLogger(), "Something went wrong");
// ```
const rclcpp::Logger& getLogger();

// Make a child logger.
// This should only be used after a node logger has been set through the
// `setLogger` function as it calls `get_child` on the result of `getLogger`.
//
// On Humble the resulting child logger does not log to /rosout.
// We had the option of logging to /rosout or keeping namespaces (on Humble)
// and we chose namespaces as /rosout was not working before this change.
//
// On Iron and Rolling, child loggers log to /rosout.
rclcpp::Logger makeChildLogger(const std::string& name);

// Set the global logger.
// Use:
// ```c++
// moveit::setLogger(node->get_logger());
// ```
void setLogger(const rclcpp::Logger& logger);

}  // namespace moveit
