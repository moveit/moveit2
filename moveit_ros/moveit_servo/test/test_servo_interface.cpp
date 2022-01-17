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

/* Author: Dave Coleman and Tyler Weaver
   Desc:   Test for the C++ interface to moveit_servo
*/

#include "servo_launch_test_common.hpp"

using namespace std::chrono_literals;

namespace moveit_servo
{
TEST_F(ServoFixture, SendTwistStampedTest)
{
  auto log_time_start = node_->now();
  ASSERT_TRUE(setupStartClient());
  ASSERT_TRUE(setupCommandSub(servo_parameters_->command_out_type));
  auto log_time_end = node_->now();
  RCLCPP_INFO_STREAM(LOGGER, "Setup time: " << (log_time_end - log_time_start).seconds());

  // Start Servo
  ASSERT_TRUE(start());
  EXPECT_EQ(latest_status_, moveit_servo::StatusCode::NO_WARNING);

  // We want to count the number of commands Servo publishes, we need timing
  auto time_start = node_->now();

  // Publish N messages with some time between, ensure it's less than the timeout for Servo
  rclcpp::Rate publish_loop_rate(test_parameters_->publish_hz);
  size_t num_commands = 30;
  resetNumCommands();
  for (size_t i = 0; i < num_commands && rclcpp::ok(); ++i)
  {
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = node_->now();
    msg->twist.linear.x = 0.1;
    msg->twist.angular.z = 0.5;
    pub_twist_cmd_->publish(std::move(msg));
    publish_loop_rate.sleep();
  }

  // Capture the time and number of received messages
  auto time_end = node_->now();
  auto num_received = getNumCommands();

  // Compare actual number received to expected number
  auto num_expected = (time_end - time_start).seconds() / servo_parameters_->publish_period;
  RCLCPP_INFO_STREAM(LOGGER, "Wait publish messages: " << (time_end - time_start).seconds());

  EXPECT_GT(num_received, 0.5 * num_expected);
  EXPECT_LT(num_received, 1.5 * num_expected);
}

TEST_F(ServoFixture, SendJointServoTest)
{
  auto log_time_start = node_->now();
  ASSERT_TRUE(setupStartClient());
  ASSERT_TRUE(setupCommandSub(servo_parameters_->command_out_type));
  auto log_time_end = node_->now();
  RCLCPP_INFO_STREAM(LOGGER, "Setup time: " << (log_time_end - log_time_start).seconds());

  // Start Servo
  ASSERT_TRUE(start());
  EXPECT_EQ(latest_status_, moveit_servo::StatusCode::NO_WARNING);

  // We want to count the number of commands Servo publishes, we need timing
  auto time_start = node_->now();

  // Publish N messages with some time between, ensure it's less than the timeout for Servo
  rclcpp::Rate publish_loop_rate(test_parameters_->publish_hz);
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
    publish_loop_rate.sleep();
  }

  // Capture the time and number of received messages
  auto time_end = node_->now();
  auto num_received = getNumCommands();

  // Compare actual number received to expected number
  auto num_expected = (time_end - time_start).seconds() / servo_parameters_->publish_period;
  log_time_end = node_->now();
  RCLCPP_INFO_STREAM(LOGGER, "Wait publish messages: " << (time_end - time_start).seconds());

  EXPECT_GT(num_received, 0.5 * num_expected);
  EXPECT_LT(num_received, 1.5 * num_expected);

  // Now let's test the Servo input going stale
  // We expect the command we were publishing above to continue for a while, then
  // to continually receive Servo output, but with 0 velocity/delta_position
  log_time_start = node_->now();

  // Allow the last command to go stale and measure the output position
  const int sleep_time = 1.5 * 1000 * servo_parameters_->incoming_command_timeout;
  rclcpp::sleep_for(std::chrono::milliseconds(sleep_time));
  double joint_position_first = getLatestTrajCommand().points[0].positions[0];

  // Now if we sleep a bit longer and check again, it should be the same
  rclcpp::sleep_for(std::chrono::milliseconds(sleep_time));
  double joint_position_later = getLatestTrajCommand().points[0].positions[0];
  EXPECT_NEAR(joint_position_first, joint_position_later, 0.001);

  log_time_end = node_->now();
  RCLCPP_INFO_STREAM(LOGGER, "Wait stale command: " << (log_time_end - log_time_start).seconds());
}

TEST_F(ServoFixture, DynamicParameterTest)
{
  ASSERT_TRUE(setupStartClient());

  // Start Servo
  ASSERT_TRUE(start());
  EXPECT_EQ(latest_status_, moveit_servo::StatusCode::NO_WARNING);

  auto set_parameters_client =
      node_->create_client<rcl_interfaces::srv::SetParameters>(test_parameters_->servo_node_name + "/set_parameters");

  while (!set_parameters_client->service_is_ready())
  {
    if (!rclcpp::ok())
    {
      ASSERT_TRUE(false) << "Interrupted while waiting for the service. Exiting.";
    }
    RCLCPP_INFO(LOGGER, "set_parameters_client service not yet available, waiting again...");
    rclcpp::sleep_for(500ms);
  }

  // Test changing dynamic parameter
  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

  // This is a dynamic parameter
  request->parameters.push_back(
      rclcpp::Parameter("moveit_servo.robot_link_command_frame", "panda_link4").to_parameter_msg());

  // This is not even a parameter that is declared (it should fail)
  request->parameters.push_back(rclcpp::Parameter("moveit_servo.not_set_parameter", 1.0).to_parameter_msg());

  auto set_parameter_response = set_parameters_client->async_send_request(request).get();

  // The first one should have succeeded
  ASSERT_TRUE(set_parameter_response->results[0].successful) << set_parameter_response->results[0].reason;

  // This parameter should fail to set.
  ASSERT_FALSE(set_parameter_response->results[1].successful)
      << "`not_set_parameter` is not a parameter and should fail to be set";
}
}  // namespace moveit_servo

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);

  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
