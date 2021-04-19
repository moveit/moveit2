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

/*      Title     : test_servo_singularity.cpp
 *      Project   : moveit_servo
 *      Created   : 08/05/2020
 *      Author    : Adam Pettinger
 */

#include "servo_launch_test_common.hpp"
#include <moveit_msgs/msg/planning_scene.hpp>

namespace moveit_servo
{
TEST_F(ServoFixture, ReachSingular)
{
  ASSERT_TRUE(setupStartClient());

  // Start Servo
  ASSERT_TRUE(start());
  EXPECT_EQ(latest_status_, moveit_servo::StatusCode::NO_WARNING);

  // Look for DECELERATE_FOR_SINGULARITY status
  watchForStatus(moveit_servo::StatusCode::DECELERATE_FOR_SINGULARITY);

  // Publish some twist commands that will bring us to singularity
  rclcpp::Rate publish_loop_rate(test_parameters_->publish_hz);
  auto log_time_start = node_->now();
  size_t iterations = 0;
  while (!sawTrackedStatus() && iterations++ < test_parameters_->timeout_iterations)
  {
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = node_->now();
    msg->twist.linear.x = 0.8;
    pub_twist_cmd_->publish(std::move(msg));
    publish_loop_rate.sleep();
  }

  // Test that we didn't timeout
  EXPECT_LT(iterations, test_parameters_->timeout_iterations);
  auto log_time_end = node_->now();
  RCLCPP_INFO_STREAM(LOGGER, "Wait for singularity: " << (log_time_end - log_time_start).seconds());

  // Look for NO_WARNING status
  watchForStatus(moveit_servo::StatusCode::NO_WARNING);
  resetNumStatus();

  // If we move the other way (away from singular), we should get no warnings
  log_time_start = node_->now();
  iterations = 0;
  while (!sawTrackedStatus() && iterations++ < test_parameters_->timeout_iterations)
  {
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = node_->now();
    msg->twist.linear.x = -0.1;
    msg->twist.angular.z = -0.5;
    pub_twist_cmd_->publish(std::move(msg));
    publish_loop_rate.sleep();
  }
  EXPECT_EQ(getLatestStatus(), moveit_servo::StatusCode::NO_WARNING);

  // Test that we didn't timeout
  EXPECT_LT(iterations, test_parameters_->timeout_iterations);
  log_time_end = node_->now();
  RCLCPP_INFO_STREAM(LOGGER, "Wait for no warning: " << (log_time_end - log_time_start).seconds());
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
