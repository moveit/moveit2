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

#include <string>
#include <memory>
#include <thread>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <fmt/format.h>
#include <rsl/random.hpp>

rclcpp::Logger& get_root_logger()
{
  static auto moveit_node = std::make_shared<rclcpp::Node>(fmt::format("____moveit_{}", rsl::rng()()));
  static auto logger = moveit_node->get_logger();
  return logger;
}

void set_node_logger_name(const std::string& name)
{
  static auto node = std::make_shared<rclcpp::Node>("moveit", name);
  get_root_logger() = node->get_logger();
}

rclcpp::Logger get_logger(const std::string& name)
{
  return get_root_logger().get_child(name);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("dut_node");
  set_node_logger_name(node->get_name());

  RCLCPP_ERROR(get_logger("child1"), "A");
  RCLCPP_ERROR(get_logger("child2"), "A");
  RCLCPP_ERROR(get_logger("child3"), "A");
  RCLCPP_ERROR(get_logger("child4"), "A");

  RCLCPP_ERROR(get_logger("child1"), "B");
  RCLCPP_ERROR(get_logger("child2"), "B");
  RCLCPP_ERROR(get_logger("child3"), "B");
  RCLCPP_ERROR(get_logger("child4"), "B");

  std::this_thread::sleep_for(std::chrono::seconds(1));
}
