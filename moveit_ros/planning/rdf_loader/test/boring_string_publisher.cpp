/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik Robotics
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
 *   * Neither the name of PickNik Robotics nor the names of its
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

/* Author: David V. Lu!! */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <fstream>
#include <sstream>
#include <sys/stat.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("boring_string_publisher");

  // Get Two String Parameters
  std::string topic, content_path;
  node->declare_parameter("topic", rclcpp::ParameterType::PARAMETER_STRING);
  node->get_parameter("topic", topic);
  node->declare_parameter("content_path", rclcpp::ParameterType::PARAMETER_STRING);
  node->get_parameter("content_path", content_path);

  if (content_path.empty())
  {
    RCLCPP_FATAL(node->get_logger(), "content_path parameter was not specified or is empty");
    return -1;
  }

  // Check if content exists
  struct stat statistics;
  if (stat(content_path.c_str(), &statistics) != 0)
  {
    RCLCPP_FATAL(node->get_logger(), "%s does not exist!", content_path.c_str());
    return -1;
  }

  // Set Up Publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_publisher =
      node->create_publisher<std_msgs::msg::String>(topic, rclcpp::QoS(1).transient_local());

  // Read in Content
  std::ifstream content_stream(content_path.c_str());
  std::stringstream buffer;
  buffer << content_stream.rdbuf();

  // Publish Content
  std_msgs::msg::String msg;
  msg.data = buffer.str();
  string_publisher->publish(msg);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
