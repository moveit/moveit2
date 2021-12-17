/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik Inc.
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

/* Author: Tyler Weaver
   Desc:   Input Resampler Tests
*/

#include <chrono>
#include <functional>
#include <variant>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <gtest/gtest.h>
#include <moveit_servo/detail/input_command.hpp>
#include <rclcpp/rclcpp.hpp>

#include <moveit_servo/detail/input_resampler.hpp>

#include "input_visitors.hpp"

namespace moveit_servo
{
namespace
{
bool waitFor(const rclcpp::Node::SharedPtr& node, std::function<bool()> condition, rclcpp::Duration timeout,
             rclcpp::Duration poll_period = rclcpp::Duration::from_seconds(0.01f))
{
  auto start = node->now();
  do
  {
    if (condition())
    {
      return true;
    }
    rclcpp::sleep_for(poll_period.to_chrono<std::chrono::milliseconds>());
  } while (start - node->now() < timeout);
  return false;
}
}  // namespace

using detail::InputCommand;
using detail::InputResampler;

TEST(InputReamplerTests, NoOutputVisit)
{
  // GIVEN a InputResampler with with a Visitor that counts output calls
  auto visitor = std::make_shared<CountingVisitor>();
  auto node = std::make_shared<rclcpp::Node>("test_node_0");
  auto resampler = InputResampler(node, std::chrono::nanoseconds(100000000), visitor);

  // WHEN we sleep for twice the period (without ever visiting the resampler)
  rclcpp::sleep_for(std::chrono::nanoseconds(200000000));

  // THEN we expect the count to still be 0
  EXPECT_EQ(visitor->count, 0U) << "Visitor should not have been called";
}

TEST(InputReamplerTests, OneVisit)
{
  // GIVEN a InputResampler with with a Visitor that counts output calls
  auto visitor = std::make_shared<CountingVisitor>();
  auto node = std::make_shared<rclcpp::Node>("test_node_1");
  auto resampler = InputResampler(node, std::chrono::nanoseconds(100000000), visitor);

  // WHEN we visit the resampler with one message and wait
  std::visit(resampler, InputCommand{ TwistStamped{} });
  ASSERT_TRUE(waitFor(
      node, [&]() { return visitor->count >= 1; }, rclcpp::Duration::from_seconds(10.f)))
      << "Timeout waiting for message to be received by visitor";

  // THEN we expect the count to be one
  EXPECT_GT(visitor->count, 0U) << "Visitor should have only been atleast once";
}

TEST(InputReamplerTests, WaitForSomeOutput)
{
  // GIVEN a InputResampler with with a Visitor that counts output calls
  auto visitor = std::make_shared<CountingVisitor>();
  auto node = std::make_shared<rclcpp::Node>("test_node_2");
  auto resampler = InputResampler(node, std::chrono::nanoseconds(10000000), visitor);

  // WHEN we visit the resampler with one message and wait for some time
  std::visit(resampler, InputCommand{ TwistStamped{} });
  ASSERT_TRUE(waitFor(
      node, [&]() { return visitor->count >= 10; }, rclcpp::Duration::from_seconds(10.f)))
      << "Timeout waiting for messages to be received by visitor";

  // THEN we expect the callable count to be called about 10
  EXPECT_GT(visitor->count, 9U) << "Count should have been called atleast 10 times";
}

TEST(InputReamplerTests, ReceivedEqualsSent)
{
  // GIVEN a InputResampler with Visitor that copies the received command into a local variant
  auto visitor = std::make_shared<ReceivedCommandVisitor>();
  auto node = std::make_shared<rclcpp::Node>("test_node_3");
  auto resampler = InputResampler(node, std::chrono::nanoseconds(10000000), visitor);

  // WHEN we visit the resampler with one message and wait
  auto sent_msg = JointJog{};
  std::visit(resampler, InputCommand{ sent_msg });
  ASSERT_TRUE(waitFor(
      node, [&]() { return std::holds_alternative<decltype(sent_msg)>(visitor->received_command); },
      rclcpp::Duration::from_seconds(10.f)))
      << "Timeout waiting for message to be received by visitor";

  // THEN we expect the received command to be the same as the sent message
  ASSERT_TRUE(std::holds_alternative<decltype(sent_msg)>(visitor->received_command))
      << "Received command should be of the same type as sent message";
  ASSERT_NO_THROW(std::get<JointJog>(visitor->received_command))
      << "Received command variant should be of type JointJog";
  EXPECT_EQ(std::get<JointJog>(visitor->received_command), sent_msg)
      << "Received command variant should be equal to sent message";
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
