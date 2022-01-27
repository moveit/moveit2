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

/* Author: Tyler Weaver, Andy Zelenak
   Desc:   Basic Functionality tests
*/

// System
#include <string>

// ROS
#include <rclcpp/rclcpp.hpp>

// Testing
#include <gtest/gtest.h>

// Servo
#include <moveit_servo/make_shared_from_pool.h>
#include <moveit_servo/servo.h>
#include "servo_launch_test_common.hpp"

namespace moveit_servo
{
TEST_F(ServoFixture, SendTwistStampedTest)
{
  auto log_time_start = node_->now();
  ASSERT_TRUE(setupStartClient());
  auto log_time_end = node_->now();
  RCLCPP_INFO_STREAM(LOGGER, "Setup time: " << (log_time_end - log_time_start).seconds());

  // Start Servo
  ASSERT_TRUE(start());
  EXPECT_EQ(latest_status_, moveit_servo::StatusCode::NO_WARNING);

  auto parameters = servo_parameters_;

  // count trajectory messages sent by servo
  size_t received_count = 0;
  std::function<void(const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr)> traj_callback =
      [&received_count](const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr /* unused */) { ++received_count; };
  auto traj_sub = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      resolveServoTopicName(parameters->command_out_topic), 1, traj_callback);

  // Create publisher to send servo commands
  auto twist_stamped_pub = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
      resolveServoTopicName(parameters->cartesian_command_in_topic), 1);

  constexpr double test_duration = 1.0;
  const double publish_period = parameters->publish_period;
  const size_t num_commands = static_cast<size_t>(test_duration / publish_period);

  // Set the rate differently from the publish period from the parameters to show that
  // the number of outputs is set by the number of commands sent and not the rate they are sent.
  rclcpp::Rate publish_rate(2. / publish_period);

  // Send a few Cartesian velocity commands
  for (size_t i = 0; i < num_commands && rclcpp::ok(); ++i)
  {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = "panda_link0";
    msg.twist.angular.y = 1.0;

    // Send the message
    twist_stamped_pub->publish(msg);
    publish_rate.sleep();
  }

  EXPECT_GT(received_count, num_commands - 20);
  EXPECT_GT(received_count, (unsigned)0);
  EXPECT_LT(received_count, num_commands + 20);
}

TEST_F(ServoFixture, SendJointServoTest)
{
  auto log_time_start = node_->now();
  ASSERT_TRUE(setupStartClient());
  auto log_time_end = node_->now();
  RCLCPP_INFO_STREAM(LOGGER, "Setup time: " << (log_time_end - log_time_start).seconds());

  // Start Servo
  ASSERT_TRUE(start());
  EXPECT_EQ(latest_status_, moveit_servo::StatusCode::NO_WARNING);

  auto parameters = servo_parameters_;
  auto cmd_out_topic = resolveServoTopicName(parameters->command_out_topic);
  auto cmd_in_topic = resolveServoTopicName(parameters->joint_command_in_topic);
  RCLCPP_INFO_STREAM(LOGGER, "\nOut: " << cmd_out_topic << " In: " << cmd_in_topic << "\n");

  // count trajectory messages sent by servo
  size_t received_count = 0;
  std::function<void(const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr)> traj_callback =
      [&received_count](const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr /* unused */) { ++received_count; };
  auto traj_sub = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(cmd_out_topic, 1, traj_callback);

  // Create publisher to send servo commands
  auto joint_servo_pub = node_->create_publisher<control_msgs::msg::JointJog>(cmd_in_topic, 1);

  constexpr double test_duration = 1.0;
  const double publish_period = parameters->publish_period;
  const size_t num_commands = static_cast<size_t>(test_duration / publish_period);

  // Set the rate differently from the publish period from the parameters to show that
  // the number of outputs is set by the number of commands sent and not the rate they are sent.
  rclcpp::Rate publish_rate(2. / publish_period);

  // Send a few joint velocity commands
  for (size_t i = 0; i < num_commands && rclcpp::ok(); ++i)
  {
    control_msgs::msg::JointJog msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = "panda_link3";
    msg.velocities.push_back(0.1);

    // Send the message
    joint_servo_pub->publish(msg);
    publish_rate.sleep();
  }

  EXPECT_GT(received_count, num_commands - 20);
  EXPECT_GT(received_count, (unsigned)0);
  EXPECT_LT(received_count, num_commands + 20);
}
}  // namespace moveit_servo

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
