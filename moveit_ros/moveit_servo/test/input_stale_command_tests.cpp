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
   Desc:   Input Stale Command Tests
*/

#include <iostream>
#include <variant>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <gtest/gtest.h>
#include <moveit_servo/detail/input_command.hpp>
#include <rclcpp/rclcpp.hpp>

#include <moveit_servo/detail/input_stale_command.hpp>

#include "input_visitors.hpp"

namespace moveit_servo
{
namespace
{
const auto TIME_EPSILON = 1e-1;
}  // namespace

using detail::InputCommand;
using detail::StaleCommandHalt;
using detail::TimestampNow;

TEST(InputStaleCommandTests, TimeStampIsNearNowJointJog)
{
  // GIVEN a TimestampNow with with a Visitor that saves modified message locally
  auto visitor = ReceivedCommandVisitor{};
  auto node = std::make_shared<rclcpp::Node>("test_node_0");
  auto timestamp_now = TimestampNow(node, &visitor);

  // WHEN we visit the timestamp_now with one message
  auto sent_msg = JointJog{};
  std::visit(timestamp_now, InputCommand{ sent_msg });

  // THEN we expect the stamp to have changed
  ASSERT_TRUE(std::holds_alternative<decltype(sent_msg)>(visitor.received_command))
      << "Received command should be of the same type as sent message";
  auto received_msg = std::get<JointJog>(visitor.received_command);
  EXPECT_NE(received_msg.header.stamp, sent_msg.header.stamp)
      << "Received message should have a different timestamp than sent one";
  EXPECT_NEAR((node->now() - received_msg.header.stamp).seconds(), 0, TIME_EPSILON)
      << "Time difference between now and received_msg should be close to 0";
}

TEST(InputStaleCommandTests, TimeStampIsNearNowTwistStamped)
{
  // GIVEN a TimestampNow with with a Visitor that saves modified message locally
  auto visitor = ReceivedCommandVisitor{};
  auto node = std::make_shared<rclcpp::Node>("test_node_1");
  auto timestamp_now = TimestampNow(node, &visitor);

  // WHEN we visit the timestamp_now with one message
  auto sent_msg = TwistStamped{};
  std::visit(timestamp_now, InputCommand{ sent_msg });

  // THEN we expect the stamp to have changed
  ASSERT_TRUE(std::holds_alternative<decltype(sent_msg)>(visitor.received_command))
      << "Received command should be of the same type as sent message";
  auto received_msg = std::get<TwistStamped>(visitor.received_command);
  EXPECT_NE(received_msg.header.stamp, sent_msg.header.stamp)
      << "Received message should have a different timestamp than sent one";
  EXPECT_NEAR((node->now() - received_msg.header.stamp).seconds(), 0, TIME_EPSILON)
      << "Time difference between now and received_msg should be close to 0";
}

TEST(InputStaleCommandTests, HaltTwistStamped)
{
  // GIVEN a StaleCommandHalt with with a Visitor and halt function that counts calls
  auto visitor = CountingVisitor{};
  unsigned int halt_count{ 0 };
  auto stale_command_halt = StaleCommandHalt(
      std::make_shared<rclcpp::Node>("test_node_2"), rclcpp::Duration::from_seconds(1), [&]() { halt_count++; },
      &visitor);

  // WHEN we visit the stale_command_halt with one message that is default constructed
  std::visit(stale_command_halt, InputCommand{ TwistStamped{} });

  // THEN we expect the visitor count to still be 0 and the halt count should be 1
  EXPECT_EQ(visitor.count, 0U) << "Visitor should not have been visited";
  EXPECT_EQ(halt_count, 1U) << "Halt should have been called once";
}

TEST(InputStaleCommandTests, HaltJointJog)
{
  // GIVEN a StaleCommandHalt with with a Visitor and halt function that counts calls
  auto visitor = CountingVisitor{};
  unsigned int halt_count{ 0 };
  auto stale_command_halt = StaleCommandHalt(
      std::make_shared<rclcpp::Node>("test_node_3"), rclcpp::Duration::from_seconds(1), [&]() { halt_count++; },
      &visitor);

  // WHEN we visit the stale_command_halt with one message that is default constructed
  std::visit(stale_command_halt, InputCommand{ JointJog{} });

  // THEN we expect the visitor count to still be 0 and the halt count should be 1
  EXPECT_EQ(visitor.count, 0U) << "Visitor should not have been visited";
  EXPECT_EQ(halt_count, 1U) << "Halt should have been called once";
}

TEST(InputStaleCommandTests, NotStaleVisitTwistStamped)
{
  // GIVEN a StaleCommandHalt with with a Visitor and halt function that counts calls
  auto visitor = CountingVisitor{};
  unsigned int halt_count{ 0 };
  auto node = std::make_shared<rclcpp::Node>("test_node_4");
  auto stale_command_halt = StaleCommandHalt(
      node, rclcpp::Duration::from_seconds(1), [&]() { halt_count++; }, &visitor);

  // WHEN we visit the stale_command_halt with one message that has a current timestamp
  auto sent_msg = TwistStamped{};
  sent_msg.header.stamp = node->now();
  std::visit(stale_command_halt, InputCommand{ sent_msg });

  // THEN we expect the visitor count to be 1 and the halt count should be 0
  EXPECT_EQ(visitor.count, 1U) << "Visitor should have been visited";
  EXPECT_EQ(halt_count, 0U) << "Halt should not have been called once";
}

TEST(InputStaleCommandTests, NotStaleVisitJointJog)
{
  // GIVEN a StaleCommandHalt with with a Visitor and halt function that counts calls
  auto visitor = CountingVisitor{};
  unsigned int halt_count{ 0 };
  auto node = std::make_shared<rclcpp::Node>("test_node_5");
  auto stale_command_halt = StaleCommandHalt(
      node, rclcpp::Duration::from_seconds(1), [&]() { halt_count++; }, &visitor);

  // WHEN we visit the stale_command_halt with one message that has a current timestamp
  auto sent_msg = JointJog{};
  sent_msg.header.stamp = node->now();
  std::visit(stale_command_halt, InputCommand{ sent_msg });

  // THEN we expect the visitor count to be 1 and the halt count should be 0
  EXPECT_EQ(visitor.count, 1U) << "Visitor should have been visited";
  EXPECT_EQ(halt_count, 0U) << "Halt should not have been called once";
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
