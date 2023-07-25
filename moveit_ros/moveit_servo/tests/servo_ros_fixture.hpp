/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, PickNik Inc.
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

/* Author    : V Mohammed Ibrahim
   Desc      : Resources used by servo ROS integration tests
   Title     : servo_ros_fixture.hpp
   Project   : moveit_servo
   Created   : 07/23/2023
*/

#include <gtest/gtest.h>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <moveit_servo/utils/datatypes.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int8.hpp>

class ServoRosFixture : public testing::Test
{
protected:
  void statusCallback(const std_msgs::msg::Int8::ConstSharedPtr msg)
  {
    status_ = static_cast<moveit_servo::StatusCode>(msg->data);
    status_count_++;
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> state_guard(joint_state_mutex);
    joint_states_ = *msg;
    state_count_++;
  }

  void SetUp()
  {
    executor_->add_node(servo_test_node_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });
  }

  void TearDown()
  {
    executor_->cancel();
    if (executor_thread_.joinable())
    {
      executor_thread_.join();
    }
  }

  void waitForService()
  {
    auto logger = servo_test_node_->get_logger();
    while (!switch_input_client_->service_is_ready())
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(logger, "Interrupted while waiting for the service. Exiting.");
      }

      rclcpp::sleep_for(std::chrono::milliseconds(500));
    }
    RCLCPP_INFO(logger, "SERVICE READY");
  }

  ServoRosFixture()
  {
    // Create a node to be given to Servo.
    servo_test_node_ = std::make_shared<rclcpp::Node>("moveit_servo_test");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    status_ = moveit_servo::StatusCode::INVALID;

    status_subscriber_ = servo_test_node_->create_subscription<std_msgs::msg::Int8>(
        "/moveit_servo/status", 1, std::bind(&ServoRosFixture::statusCallback, this, std::placeholders::_1));

    joint_state_subscriber_ = servo_test_node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 1, std::bind(&ServoRosFixture::jointStateCallback, this, std::placeholders::_1));

    switch_input_client_ =
        servo_test_node_->create_client<moveit_msgs::srv::ServoCommandType>("moveit_servo/switch_command_type");

    waitForService();
  }

  std::shared_ptr<rclcpp::Node> servo_test_node_;

  // Executor and a thread to run the executor.
  rclcpp::Executor::SharedPtr executor_;
  std::thread executor_thread_;

  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr status_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_input_client_;

  std::atomic<int> status_count_;
  std::atomic<moveit_servo::StatusCode> status_;

  std::mutex joint_state_mutex;
  std::atomic<int> state_count_;
  sensor_msgs::msg::JointState joint_states_;
};
