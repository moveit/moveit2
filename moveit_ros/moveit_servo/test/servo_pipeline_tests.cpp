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
   Desc:   Input Pipeline Tests
*/

#include <memory>
#include <variant>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <gtest/gtest.h>
#include <moveit_servo/detail/input_command.hpp>
#include <moveit_servo/servo_parameters.h>
#include <rclcpp/rclcpp.hpp>

#include <moveit_servo/detail/servo_pipeline.hpp>

#include "input_visitors.hpp"

namespace moveit_servo
{
using detail::ServoPipeline;

TEST(ServoPipelineTests, makeResampling)
{
  // GIVEN a ServoPipeline
  auto node = std::make_shared<rclcpp::Node>("test_node_0");
  auto visitor = std::make_shared<DoNothingVisitor>();
  auto parameters = std::make_shared<ServoParameters>();

  // WHEN low_latency_mode set to false
  parameters->low_latency_mode = false;

  // THEN we expect ServoPipeline constructor to not throw
  EXPECT_NO_THROW(auto servo_pipeline = ServoPipeline(
                      node, parameters, []() {}, visitor));
}

TEST(ServoPipelineTests, makeReactive)
{
  // GIVEN a ServoPipeline
  auto node = std::make_shared<rclcpp::Node>("test_node_0");
  auto visitor = std::make_shared<DoNothingVisitor>();
  auto parameters = std::make_shared<ServoParameters>();

  // WHEN low_latency_mode set to true
  parameters->low_latency_mode = true;

  // THEN we expect ServoPipeline constructor to not throw
  EXPECT_NO_THROW(auto servo_pipeline = ServoPipeline(
                      node, parameters, []() {}, visitor));
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
