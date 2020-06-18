/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

/* Author: Dave Coleman and Tyler Weaver
   Desc:   Test for the C++ interface to moveit_servo
*/

#include "servo_launch_test_common.hpp"

namespace moveit_servo
{
TEST_F(ServoFixture, StartStopTest)
{
  // Setup the start/stop clients, and the command callback
  ASSERT_TRUE(setupStartClient());
  ASSERT_TRUE(setupStatusSub());
  ASSERT_TRUE(setupJointStateSub());

  // Wait for a joint state message to ensure fake_joint_driver is up
  bool got_msg = false;
  while (!got_msg)
  {
    if (getNumJointState() > 0)
      got_msg = true;

    rclcpp::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_TRUE(got_msg);

  // Try to start Servo
  auto start_result = client_servo_start_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  EXPECT_TRUE(start_result.get()->success);

  // With servo running we should get a status that should be NO_WARNING
  rclcpp::sleep_for(std::chrono::milliseconds(200));
  EXPECT_TRUE(getNumStatus() > 0);
  EXPECT_TRUE(latest_status_ == moveit_servo::StatusCode::NO_WARNING);

  // Now stop servo and wait
  auto stop_result = client_servo_stop_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  EXPECT_TRUE(stop_result.get()->success);
  rclcpp::sleep_for(std::chrono::seconds(1));
  resetNumStatus();

  // Restart and recheck Servo
  start_result = client_servo_start_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  EXPECT_TRUE(start_result.get()->success);
  rclcpp::sleep_for(std::chrono::milliseconds(200));
  EXPECT_TRUE(getNumStatus() > 0);
  EXPECT_TRUE(latest_status_ == moveit_servo::StatusCode::NO_WARNING);
}

TEST_F(ServoFixture, SendTwistStampedTest)
{
  ASSERT_TRUE(setupStartClient());
  ASSERT_TRUE(setupCommandSub(parameters_->command_out_type));

  // Start Servo
  auto start_result = client_servo_start_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_TRUE(start_result.get()->success);

  // We want to count the number of commands Servo publishes, we need timing
  auto time_start = node_->now();

  // Publish N messages with some time between, ensure it's less than the timeout for Servo
  size_t num_commands = 30;
  rclcpp::Rate loop_rate(2 / parameters_->incoming_command_timeout);
  resetNumCommands();
  for (size_t i = 0; i < num_commands && rclcpp::ok(); ++i)
  {
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = node_->now();
    msg->twist.linear.x = 0.1;
    msg->twist.angular.z = 0.5;
    pub_twist_cmd_->publish(std::move(msg));
    loop_rate.sleep();
  }

  // Capture the time and number of recieved messages
  auto time_end = node_->now();
  auto num_recieved = getNumCommands();

  // Compare actual number recieved to expected number
  auto num_expected = (time_end - time_start).seconds() / parameters_->publish_period;

  EXPECT_GT(num_recieved, 0.5 * num_expected);
  EXPECT_LT(num_recieved, 1.5 * num_expected);
}

TEST_F(ServoFixture, SendJointServoTest)
{
  ASSERT_TRUE(setupStartClient());
  ASSERT_TRUE(setupCommandSub(parameters_->command_out_type));

  // Start Servo
  auto start_result = client_servo_start_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_TRUE(start_result.get()->success);

  // We want to count the number of commands Servo publishes, we need timing
  auto time_start = node_->now();

  // Publish N messages with some time between, ensure it's less than the timeout for Servo
  size_t num_commands = 30;
  rclcpp::Rate loop_rate(2 / parameters_->incoming_command_timeout);
  resetNumCommands();
  for (size_t i = 0; i < num_commands && rclcpp::ok(); ++i)
  {
    auto msg = std::make_unique<control_msgs::msg::JointJog>();
    msg->header.stamp = node_->now();
    msg->header.frame_id = "panda_link3";
    msg->joint_names.push_back("panda_joint1");
    msg->velocities.push_back(0.1);
    pub_joint_cmd_->publish(std::move(msg));
    loop_rate.sleep();
  }

  // Capture the time and number of recieved messages
  auto time_end = node_->now();
  auto num_recieved = getNumCommands();

  // Compare actual number recieved to expected number
  auto num_expected = (time_end - time_start).seconds() / parameters_->publish_period;

  EXPECT_GT(num_recieved, 0.5 * num_expected);
  EXPECT_LT(num_recieved, 1.5 * num_expected);
}

TEST_F(ServoFixture, StaleCommandStop)
{
  ASSERT_TRUE(setupStartClient());
  ASSERT_TRUE(setupCommandSub(parameters_->command_out_type));

  // Start Servo
  auto start_result = client_servo_start_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_TRUE(start_result.get()->success);

  // Setup the message to publish (only once)
  auto msg = std::make_unique<control_msgs::msg::JointJog>();
  msg->joint_names.push_back("panda_joint1");
  msg->header.frame_id = "panda_link3";
  msg->velocities.push_back(0.1);

  // Wait the stale limit, plus a little extra
  const int sleep_time = 5 * 1000 * parameters_->incoming_command_timeout;
  rclcpp::sleep_for(std::chrono::milliseconds(sleep_time));

  // Get current position
  double start_position = getLatestTrajCommand().points[0].positions[0];

  // Publish once
  msg->header.stamp = node_->now();
  pub_joint_cmd_->publish(std::move(msg));

  // Wait the stale limit, plus a little extra
  rclcpp::sleep_for(std::chrono::milliseconds(sleep_time));

  // Get the new current position (should be different than first)
  double middle_position = getLatestTrajCommand().points[0].positions[0];
  EXPECT_NE(start_position, middle_position);

  // Wait the stale limit
  rclcpp::sleep_for(std::chrono::milliseconds(sleep_time));

  // Get the current position (should be no change)
  double end_position = getLatestTrajCommand().points[0].positions[0];
  EXPECT_NEAR(middle_position, end_position, 0.001);
}
}  // namespace moveit_servo

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
