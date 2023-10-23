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

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <moveit/utils/logger.hpp>
#include <thread>
#include <rclcpp/version.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("logger_test_node");

  // Not a node logger, should print but should not be in file or rosout output
  RCLCPP_INFO(moveit::get_logger(), "Before");

  // Set the node logger
  moveit::get_logger_mut() = node->get_logger();

  // A node logger, should be in the file output and rosout
  RCLCPP_INFO(moveit::get_logger(), "After");

  // A child logger, should also be in the file output and rosout
  const auto child_logger = moveit::get_child_logger("child");
  RCLCPP_INFO(child_logger, "Child");

  // Spin the node to publish to /rosout
#if RCLCPP_VERSION_GTE(22, 1, 0)  // https://github.com/ros2/rclcpp/commit/a17d26b20ac41cc9d5bf3583de8475bb7c18bb1e
  rclcpp::spin_all(node, std::chrono::seconds(1));
#else  // Humble or Iron
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin_all(std::chrono::seconds(1));
#endif

  std::this_thread::sleep_for(std::chrono::seconds(5));
}
