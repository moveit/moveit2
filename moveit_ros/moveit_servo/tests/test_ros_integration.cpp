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
   Desc      : ROS API integration tests
   Title     : test_ros_integration.cpp
   Project   : moveit_servo
   Created   : 07/23/2023
*/

#include "servo_ros_fixture.hpp"
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

/**
 * Order of joints in joint state message
 *
 * - panda_finger_joint1
 * - panda_joint1
 * - panda_joint2
 * - panda_finger_joint2
 * - panda_joint3
 * - panda_joint4
 * - panda_joint5
 * - panda_joint6
 * - panda_joint7 --> index 8
 */

namespace
{
TEST_F(ServoRosFixture, testJointJog)
{
  auto joint_jog_publisher = servo_test_node_->create_publisher<control_msgs::msg::JointJog>(
      "/moveit_servo/delta_joint_cmds", rclcpp::SystemDefaultsQoS());

  // Call input type service
  auto request = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
  request->command_type = 0;
  auto response = switch_input_client_->async_send_request(request);
  ASSERT_EQ(response.get()->expected_type, 0);

  if (state_count_ > 0)
    RCLCPP_INFO_STREAM(servo_test_node_->get_logger(), "State available");
  else
    RCLCPP_INFO_STREAM(servo_test_node_->get_logger(), "state not available");

  sensor_msgs::msg::JointState prev_state, curr_state;
  {
    std::lock_guard<std::mutex> state_guard(joint_state_mutex);
    prev_state = joint_states_;
  }

  control_msgs::msg::JointJog jog_cmd;
  jog_cmd.header.stamp = servo_test_node_->now();
  jog_cmd.header.frame_id = "panda_link0";
  jog_cmd.joint_names.resize(7);
  jog_cmd.velocities.resize(7);

  jog_cmd.joint_names = { "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                          "panda_joint5", "panda_joint6", "panda_joint7" };

  std::fill(jog_cmd.velocities.begin(), jog_cmd.velocities.end(), 0.0);

  jog_cmd.velocities[6] = 1.0;

  size_t count = 0;
  while (rclcpp::ok() && count < 5)
  {
    joint_jog_publisher->publish(jog_cmd);
    count++;
    rclcpp::sleep_for(std::chrono::milliseconds(200));
  }

  {
    std::lock_guard<std::mutex> state_guard(joint_state_mutex);
    curr_state = joint_states_;
  }

  ASSERT_NE(curr_state.position[8], prev_state.position[8]);

  moveit_servo::StatusCode status = status_;
  RCLCPP_INFO_STREAM(servo_test_node_->get_logger(), "Status after jointjog: " << static_cast<size_t>(status));
  ASSERT_EQ(status_, moveit_servo::StatusCode::NO_WARNING);
}

TEST_F(ServoRosFixture, testTwist)
{
  auto twist_publisher = servo_test_node_->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/moveit_servo/delta_twist_cmds", rclcpp::SystemDefaultsQoS());

  auto request = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
  request->command_type = 1;
  auto response = switch_input_client_->async_send_request(request);
  ASSERT_EQ(response.get()->expected_type, 1);

  if (state_count_ > 0)
    RCLCPP_INFO_STREAM(servo_test_node_->get_logger(), "State available");
  else
    RCLCPP_INFO_STREAM(servo_test_node_->get_logger(), "state not available");

  sensor_msgs::msg::JointState prev_state, curr_state;
  {
    std::lock_guard<std::mutex> state_guard(joint_state_mutex);
    prev_state = joint_states_;
  }

  geometry_msgs::msg::TwistStamped twist_cmd;
  twist_cmd.header.stamp = servo_test_node_->now();
  twist_cmd.header.frame_id = "panda_link0";  // Planning frame
  twist_cmd.twist.linear.x = 0.0;
  twist_cmd.twist.linear.y = 0.0;
  twist_cmd.twist.linear.z = 0.0;
  twist_cmd.twist.angular.x = 0.0;
  twist_cmd.twist.angular.y = 0.0;
  twist_cmd.twist.angular.z = 0.5;

  size_t count = 0;
  while (rclcpp::ok() && count < 5)
  {
    twist_publisher->publish(twist_cmd);
    count++;
    rclcpp::sleep_for(std::chrono::milliseconds(200));
  }

  {
    std::lock_guard<std::mutex> state_guard(joint_state_mutex);
    curr_state = joint_states_;
  }

  ASSERT_NE(curr_state.position[8], prev_state.position[8]);

  moveit_servo::StatusCode status = status_;
  RCLCPP_INFO_STREAM(servo_test_node_->get_logger(), "Status after twist: " << static_cast<size_t>(status));
  ASSERT_EQ(status_, moveit_servo::StatusCode::NO_WARNING);
}

TEST_F(ServoRosFixture, testPose)
{
  auto pose_publisher = servo_test_node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/moveit_servo/pose_target_cmds", rclcpp::SystemDefaultsQoS());

  auto request = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
  request->command_type = 2;
  auto response = switch_input_client_->async_send_request(request);
  ASSERT_EQ(response.get()->expected_type, 2);

  geometry_msgs::msg::PoseStamped pose_cmd;
  pose_cmd.header.stamp = servo_test_node_->now();
  pose_cmd.header.frame_id = "panda_link0";  // Planning frame

  pose_cmd.pose.position.x = 0.5;
  pose_cmd.pose.position.y = 0.0;
  pose_cmd.pose.position.z = 0.5;
  pose_cmd.pose.orientation.w = 1.0;
  pose_cmd.pose.orientation.x = 0.0;
  pose_cmd.pose.orientation.y = 0.0;
  pose_cmd.pose.orientation.z = 0.0;

  if (state_count_ > 0)
    RCLCPP_INFO_STREAM(servo_test_node_->get_logger(), "State available");
  else
    RCLCPP_INFO_STREAM(servo_test_node_->get_logger(), "state not available");

  sensor_msgs::msg::JointState prev_state, curr_state;
  {
    std::lock_guard<std::mutex> state_guard(joint_state_mutex);
    prev_state = joint_states_;
  }

  size_t count = 0;
  while (rclcpp::ok() && count < 5)
  {
    pose_publisher->publish(pose_cmd);
    count++;
    rclcpp::sleep_for(std::chrono::milliseconds(200));
  }

  {
    std::lock_guard<std::mutex> state_guard(joint_state_mutex);
    curr_state = joint_states_;
  }

  ASSERT_NE(curr_state.position[8], prev_state.position[8]);

  moveit_servo::StatusCode status = status_;
  RCLCPP_INFO_STREAM(servo_test_node_->get_logger(), "Status after pose: " << static_cast<size_t>(status));
  ASSERT_EQ(status_, moveit_servo::StatusCode::NO_WARNING);
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
