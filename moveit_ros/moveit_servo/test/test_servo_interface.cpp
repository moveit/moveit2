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
  auto log_time_start = node_->now();
  ASSERT_TRUE(setupStartClient());
  ASSERT_TRUE(setupJointStateSub());
  auto log_time_end = node_->now();
  RCLCPP_INFO_STREAM(LOGGER, "Setup time: " << (log_time_end - log_time_start).seconds());

  // Wait for a joint state message to ensure fake_joint_driver is up
  log_time_start = node_->now();
  bool got_msg = false;
  while (!got_msg)
  {
    if (getNumJointState() > 0)
      got_msg = true;

    publish_loop_rate_.sleep();
  }
  ASSERT_TRUE(got_msg);
  log_time_end = node_->now();
  RCLCPP_INFO_STREAM(LOGGER, "Wait for joint state message time: " << (log_time_end - log_time_start).seconds());

  // Try to start Servo
  ASSERT_TRUE(start());
  EXPECT_EQ(latest_status_, moveit_servo::StatusCode::NO_WARNING);

  // Now stop servo
  ASSERT_TRUE(stop());

  // Restart and recheck Servo
  ASSERT_TRUE(start());
  EXPECT_EQ(latest_status_, moveit_servo::StatusCode::NO_WARNING);
}

TEST_F(ServoFixture, SendTwistStampedTest)
{
  auto log_time_start = node_->now();
  ASSERT_TRUE(setupStartClient());
  ASSERT_TRUE(setupCommandSub(parameters_->command_out_type));
  auto log_time_end = node_->now();
  RCLCPP_INFO_STREAM(LOGGER, "Setup time: " << (log_time_end - log_time_start).seconds());

  // Start Servo
  ASSERT_TRUE(start());
  EXPECT_EQ(latest_status_, moveit_servo::StatusCode::NO_WARNING);

  // We want to count the number of commands Servo publishes, we need timing
  auto time_start = node_->now();

  // Publish N messages with some time between, ensure it's less than the timeout for Servo
  size_t num_commands = 30;
  resetNumCommands();
  for (size_t i = 0; i < num_commands && rclcpp::ok(); ++i)
  {
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = node_->now();
    msg->twist.linear.x = 0.1;
    msg->twist.angular.z = 0.5;
    pub_twist_cmd_->publish(std::move(msg));
    publish_loop_rate_.sleep();
  }

  // Capture the time and number of received messages
  auto time_end = node_->now();
  auto num_received = getNumCommands();

  // Compare actual number received to expected number
  auto num_expected = (time_end - time_start).seconds() / parameters_->publish_period;
  RCLCPP_INFO_STREAM(LOGGER, "Wait publish messages: " << (time_end - time_start).seconds());

  EXPECT_GT(num_received, 0.5 * num_expected);
  EXPECT_LT(num_received, 1.5 * num_expected);
}

TEST_F(ServoFixture, SendJointServoTest)
{
  auto log_time_start = node_->now();
  ASSERT_TRUE(setupStartClient());
  ASSERT_TRUE(setupCommandSub(parameters_->command_out_type));
  auto log_time_end = node_->now();
  RCLCPP_INFO_STREAM(LOGGER, "Setup time: " << (log_time_end - log_time_start).seconds());

  // Start Servo
  ASSERT_TRUE(start());
  EXPECT_EQ(latest_status_, moveit_servo::StatusCode::NO_WARNING);

  // We want to count the number of commands Servo publishes, we need timing
  auto time_start = node_->now();

  // Publish N messages with some time between, ensure it's less than the timeout for Servo
  size_t num_commands = 30;
  resetNumCommands();
  for (size_t i = 0; i < num_commands && rclcpp::ok(); ++i)
  {
    auto msg = std::make_unique<control_msgs::msg::JointJog>();
    msg->header.stamp = node_->now();
    msg->header.frame_id = "panda_link3";
    msg->joint_names.push_back("panda_joint1");
    msg->velocities.push_back(0.1);
    pub_joint_cmd_->publish(std::move(msg));
    publish_loop_rate_.sleep();
  }

  // Capture the time and number of received messages
  auto time_end = node_->now();
  auto num_received = getNumCommands();

  // Compare actual number received to expected number
  auto num_expected = (time_end - time_start).seconds() / parameters_->publish_period;
  log_time_end = node_->now();
  RCLCPP_INFO_STREAM(LOGGER, "Wait publish messages: " << (time_end - time_start).seconds());

  EXPECT_GT(num_received, 0.5 * num_expected);
  EXPECT_LT(num_received, 1.5 * num_expected);

  // Now let's test the Servo input going stale
  // We expect the command we were publishing above to continue for a while, then
  // to continually receive Servo output, but with 0 velocity/delta_position
  log_time_start = node_->now();

  // Allow the last command to go stale and measure the output position
  const int sleep_time = 1.5 * 1000 * parameters_->incoming_command_timeout;
  rclcpp::sleep_for(std::chrono::milliseconds(sleep_time));
  double joint_position_first = getLatestTrajCommand().points[0].positions[0];

  // Now if we sleep a bit longer and check again, it should be the same
  rclcpp::sleep_for(std::chrono::milliseconds(sleep_time));
  double joint_position_later = getLatestTrajCommand().points[0].positions[0];
  EXPECT_NEAR(joint_position_first, joint_position_later, 0.001);

  log_time_end = node_->now();
  RCLCPP_INFO_STREAM(LOGGER, "Wait stale command: " << (log_time_end - log_time_start).seconds());
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
