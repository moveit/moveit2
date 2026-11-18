/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Giorgio Medico
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
 *   * Neither the name of the copyright holder nor the names of its
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

#include <moveit/rdf_loader/synchronized_string_parameter.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <future>
#include <string>
#include <thread>

TEST(SynchronizedStringParameter, continuous_topic_updates_reach_the_callback)
{
  // GIVEN a latched string topic
  auto node = std::make_shared<rclcpp::Node>("continuous_topic_updates");
  auto publisher = node->create_publisher<std_msgs::msg::String>("continuous_string", rclcpp::QoS(1).transient_local());
  std_msgs::msg::String msg;
  msg.data = "first";
  publisher->publish(msg);

  // WHEN the initial value is loaded with continuous updates and the node is spun
  std::promise<std::string> update;
  rdf_loader::SynchronizedStringParameter ssp;
  const std::string initial = ssp.loadInitialValue(
      node, "continuous_string", [&update](const std::string& value) { update.set_value(value); }, true, 5.0);
  EXPECT_EQ("first", initial);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinning_thread([&executor] { executor.spin(); });

  msg.data = "second";
  publisher->publish(msg);

  // THEN the new value reaches the callback
  auto future = update.get_future();
  const bool received = future.wait_for(std::chrono::seconds(5)) == std::future_status::ready;
  executor.cancel();
  spinning_thread.join();
  ASSERT_TRUE(received);
  EXPECT_EQ("second", future.get());
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);

  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
