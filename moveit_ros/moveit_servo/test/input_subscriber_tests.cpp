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
   Desc:   Input Subscriber Tests
*/

#include <memory>
#include <variant>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <gtest/gtest.h>
#include <moveit_servo/detail/input_command.hpp>
#include <rclcpp/rclcpp.hpp>

#include <moveit_servo/detail/input_subscriber.hpp>

#include "input_visitors.hpp"

namespace moveit_servo
{
using detail::InputCommand;
using detail::InputSubscriber;

TEST(InputSubscriberTests, NoPublish)
{
  // GIVEN a InputSubscriber with with a Visitor that counts calls
  auto visitor = std::make_shared<CountingVisitor>();
  auto node = std::make_shared<rclcpp::Node>("test_node_0");
  auto input_subscriber = InputSubscriber(node, "a", "b", visitor);

  // WHEN we spin (without ever publishing a message to either topic)
  rclcpp::spin_some(node);

  // THEN we expect the count to still be 0
  EXPECT_EQ(visitor->count, 0U) << "Visitor should not have been visited";
}

TEST(InputSubscriberTests, TwistStampedVisit)
{
  // GIVEN a InputSubscriber with with a Visitor that counts calls to each type of topic
  auto visitor = std::make_shared<TypeCountingVisitor>();
  auto node = std::make_shared<rclcpp::Node>("test_node_1");
  auto input_subscriber = InputSubscriber(node, "twist_stamped_topic", "joint_jog_topic", visitor);
  auto pub = node->create_publisher<TwistStamped>("twist_stamped_topic", rclcpp::SystemDefaultsQoS());

  // WHEN we publish a single message to it and spin
  pub->publish(TwistStamped{});
  rclcpp::spin_some(node);

  // THEN we expect the count of twist stamped to be 1 and the joint jog count to be 0
  EXPECT_EQ(visitor->twist_stamped_count, 1U) << "Visitor count for twist stamped type should be 1";
  EXPECT_EQ(visitor->joint_jog_count, 0U) << "Visitor count for joint jog type should be 0";
}

TEST(InputSubscriberTests, JointJogVisit)
{
  // GIVEN a InputSubscriber with with a Visitor that counts calls to each type of topic
  auto visitor = std::make_shared<TypeCountingVisitor>();
  auto node = std::make_shared<rclcpp::Node>("test_node_2");
  auto input_subscriber = InputSubscriber(node, "twist_stamped_topic", "joint_jog_topic", visitor);
  auto pub = node->create_publisher<JointJog>("joint_jog_topic", rclcpp::SystemDefaultsQoS());

  // WHEN we publish a single message to it and spin
  pub->publish(JointJog{});
  rclcpp::spin_some(node);

  // THEN we expect the count of twist stamped to be 0 and the joint jog count to be 1
  EXPECT_EQ(visitor->twist_stamped_count, 0U) << "Visitor count for twist stamped type should be 0";
  EXPECT_EQ(visitor->joint_jog_count, 1U) << "Visitor count for joint jog type should be 1";
}

TEST(InputSubscriberTests, ReceivedEqualsSent)
{
  // GIVEN a InputSubscriber with with a Visitor that copies the received message into a local variant
  auto visitor = std::make_shared<ReceivedCommandVisitor>();
  auto node = std::make_shared<rclcpp::Node>("test_node_3");
  auto input_subscriber = InputSubscriber(node, "twist_stamped_topic", "joint_jog_topic", visitor);
  auto pub = node->create_publisher<JointJog>("joint_jog_topic", rclcpp::SystemDefaultsQoS());

  // WHEN we publish a single message to it and spin
  auto sent_msg = JointJog{};
  pub->publish(sent_msg);
  rclcpp::spin_some(node);

  // THEN we expect the received command to be the same as the sent message
  ASSERT_TRUE(std::holds_alternative<decltype(sent_msg)>(visitor->received_command))
      << "Received command variant should be of the same type as sent message";
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
