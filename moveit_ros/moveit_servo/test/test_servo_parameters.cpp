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

/*      Title     : test_servo_parameters.cpp
 *      Project   : moveit_servo
 *      Created   : 07/22/2020
 *      Author    : Adam Pettinger
 *      Desc      : The launch_test for testing ServoParameters loading and checking
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit_servo/servo_parameters.cpp>
#include <std_srvs/srv/trigger.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.test_servo_parameters.cpp");

using namespace std::chrono_literals;

TEST(TestServoParameters, LoadParamsSuccess)
{
  auto node = std::make_shared<rclcpp::Node>("test_servo_parameters");

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr load_param_client =
      node->create_client<std_srvs::srv::Trigger>("get_loading_result");

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr equal_param_client =
      node->create_client<std_srvs::srv::Trigger>("get_equal_expected_result");

  while (!load_param_client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "get_loading_result service not available, waiting again...");
  }

  while (!equal_param_client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "get_equal_expected_result service not available, waiting again...");
  }

  // Test the loading of the parameters
  auto loading_result = load_param_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_TRUE(rclcpp::spin_until_future_complete(node, loading_result) == rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(loading_result.get()->success);

  // Test if the loaded parameters equal the expected parameters
  auto equal_result = equal_param_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_TRUE(rclcpp::spin_until_future_complete(node, equal_result) == rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(equal_result.get()->success);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
