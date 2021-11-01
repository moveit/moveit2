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
   Desc:   Topic Resampler Tests
*/

#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <moveit_servo/topic_resampler.hpp>

using std_msgs::msg::Bool;
using std_msgs::msg::Int8;

namespace moveit_servo
{
TEST(TopicResamplerTests, NoPublishTest)
{
  std::atomic<unsigned int> callback_count{ 0 };
  auto callback = [&](std::shared_ptr<Bool> /*unused*/) { callback_count++; };

  // GIVEN a topic resampler with a connected publisher
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto resampler = TopicResampler<Bool>(node, "test_topic", rclcpp::Duration::from_seconds(10), callback);

  // WHEN we spin (without ever publishing a message to that topic)
  rclcpp::spin_some(node);

  // THEN we expect the callback count to still be 0
  EXPECT_EQ(callback_count, 0U) << "Callback should not have been called";
}

TEST(TopicResamplerTests, OnePublishTest)
{
  std::atomic<unsigned int> callback_count{ 0 };
  auto callback = [&](std::shared_ptr<Bool> /*unused*/) { callback_count++; };

  // GIVEN a topic resampler with a connected publisher with a long period
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto resampler = TopicResampler<Bool>(node, "test_topic", rclcpp::Duration::from_seconds(10), callback);
  auto pub = node->create_publisher<Bool>("test_topic", rclcpp::SystemDefaultsQoS());

  // WHEN we publish a single message to it and spin
  pub->publish(Bool());
  rclcpp::spin_some(node);

  // THEN we expect the callback count to be one
  EXPECT_EQ(callback_count, 1U) << "Callback should have only been called once";
}

TEST(TopicResamplerTests, OneSecTest)
{
  std::atomic<unsigned int> callback_count{ 0 };
  auto callback = [&](std::shared_ptr<Bool> /*unused*/) { callback_count++; };

  // GIVEN a topic resampler with a connected publisher with a 1/10s period
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto resampler = TopicResampler<Bool>(node, "test_topic", rclcpp::Duration::from_seconds(0.1), callback);
  auto pub = node->create_publisher<Bool>("test_topic", rclcpp::SystemDefaultsQoS());

  // WHEN we publish a single message to it and spin for 1 second
  pub->publish(Bool());
  auto start = std::chrono::steady_clock::now();
  while ((std::chrono::steady_clock::now() - start) < std::chrono::seconds(1))
  {
    rclcpp::spin_some(node);
  }

  // THEN we expect the callback count to be called about 10 times
  EXPECT_NEAR(callback_count, 10U, 2U) << "Callback should have been called about 10 times";
}

TEST(TopicResamplerTests, SameMessageTest)
{
  auto returned_msg = std::make_shared<Int8>();
  returned_msg->data = 0;
  auto callback = [&](std::shared_ptr<Int8> msg) { returned_msg = msg; };

  // GIVEN a topic resampler with a connected publisher
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto resampler = TopicResampler<Int8>(node, "test_topic", rclcpp::Duration::from_seconds(1), callback);
  auto pub = node->create_publisher<Int8>("test_topic", rclcpp::SystemDefaultsQoS());

  // WHEN we publish a single message to it and spin
  auto sent_msg = Int8();
  sent_msg.data = 10;
  pub->publish(sent_msg);
  rclcpp::spin_some(node);

  // THEN we expect the message we received to have the same data
  EXPECT_EQ(returned_msg->data, sent_msg.data) << "Returned data should be the same as the sent data";
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
