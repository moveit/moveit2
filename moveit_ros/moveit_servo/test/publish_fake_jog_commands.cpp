/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
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
 *   * Neither the name of PickNik Inc. nor the names of its
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

/* Author    : Adam Pettinger
   Desc      : Simple node to publish commands to the Servo demo
   Title     : test_servo_parameters.cpp
   Project   : moveit_servo
   Created   : 07/16/2020
*/

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos_event.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.publish_fake_jog_commands.cpp");

  // ROS objects
  rclcpp::NodeOptions node_options;
  auto node = std::make_shared<rclcpp::Node>("publish_fake_jog_commands", node_options);

  // Call the start service to init and start the servo component
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client =
      node->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
  servo_start_client->wait_for_service(1s);
  servo_start_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

  // Create a publisher for publishing the jog commands
  auto pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("/servo_node/delta_twist_cmds", 10);
  std::weak_ptr<std::remove_pointer<decltype(pub.get())>::type> captured_pub = pub;
  std::weak_ptr<std::remove_pointer<decltype(node.get())>::type> captured_node = node;
  auto callback = [captured_pub, captured_node]() -> void {
    auto pub_ptr = captured_pub.lock();
    auto node_ptr = captured_node.lock();
    if (!pub_ptr || !node_ptr)
    {
      return;
    }
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = node_ptr->now();
    msg->header.frame_id = "panda_link0";
    msg->twist.linear.x = 0.3;
    msg->twist.angular.z = 0.5;
    pub_ptr->publish(std::move(msg));
  };
  auto timer = node->create_wall_timer(50ms, callback);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
