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
#include <moveit_msgs/msg/servo_status.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <moveit_servo/utils/datatypes.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

class ServoRosFixture : public testing::Test
{
protected:
  void statusCallback(const moveit_msgs::msg::ServoStatus::ConstSharedPtr& msg)
  {
    status_ = static_cast<moveit_servo::StatusCode>(msg->code);
    status_count_++;
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg)
  {
    joint_states_ = *msg;
    state_count_++;
  }

  void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr& msg)
  {
    published_trajectory_ = *msg;
    traj_count_++;
  }

  void SetUp() override
  {
    // Create a node to be given to Servo.
    servo_test_node_ = std::make_shared<rclcpp::Node>("moveit_servo_test");

    status_ = moveit_servo::StatusCode::INVALID;

    status_subscriber_ = servo_test_node_->create_subscription<moveit_msgs::msg::ServoStatus>(
        "/servo_node/status", rclcpp::SystemDefaultsQoS(),
        [this](const moveit_msgs::msg::ServoStatus::ConstSharedPtr& msg) { return statusCallback(msg); });

    joint_state_subscriber_ = servo_test_node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) { return jointStateCallback(msg); });

    trajectory_subscriber_ = servo_test_node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "/panda_arm_controller/joint_trajectory", rclcpp::SystemDefaultsQoS(),
        [this](const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr& msg) { return trajectoryCallback(msg); });

    switch_input_client_ =
        servo_test_node_->create_client<moveit_msgs::srv::ServoCommandType>("/servo_node/switch_command_type");

    waitForService();

    executor_.add_node(servo_test_node_);
    executor_thread_ = std::thread([this]() { executor_.spin(); });
  }

  void TearDown() override
  {
    executor_.remove_node(servo_test_node_);
    executor_.cancel();
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
        std::exit(EXIT_FAILURE);
      }

      rclcpp::sleep_for(std::chrono::milliseconds(500));
    }
    RCLCPP_INFO(logger, "MoveIt Servo input switching service ready!");
  }

  void waitForJointStates()
  {
    auto logger = servo_test_node_->get_logger();
    auto wait_timeout = rclcpp::Duration::from_seconds(5);
    auto start_time = servo_test_node_->now();

    while (rclcpp::ok() && state_count_ == 0)
    {
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      auto elapsed_time = servo_test_node_->now() - start_time;
      if (elapsed_time >= wait_timeout)
      {
        RCLCPP_ERROR(logger, "Timed out waiting for joint states");
        FAIL();
      }
    }
  }

  ServoRosFixture() : state_count_{ 0 }, traj_count_{ 0 }
  {
  }

  std::shared_ptr<rclcpp::Node> servo_test_node_;

  // Executor and a thread to run the executor.
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread executor_thread_;

  rclcpp::Subscription<moveit_msgs::msg::ServoStatus>::SharedPtr status_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_subscriber_;
  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_input_client_;

  sensor_msgs::msg::JointState joint_states_;
  trajectory_msgs::msg::JointTrajectory published_trajectory_;

  std::atomic<int> status_count_;
  std::atomic<moveit_servo::StatusCode> status_;

  std::atomic<int> state_count_;
  std::atomic<int> traj_count_;
};
