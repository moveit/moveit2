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
   Desc:   Input Stale Input Check Valid
*/

#include <cmath>
#include <iostream>
#include <variant>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <gtest/gtest.h>
#include <moveit_servo/detail/input_command.hpp>
#include <moveit_servo/servo_parameters.h>
#include <rclcpp/rclcpp.hpp>

#include <moveit_servo/detail/input_check_valid.hpp>

#include "input_visitors.hpp"

namespace moveit_servo
{
using detail::InputCheckValid;
using detail::InputCommand;

TEST(InputCheckValidTests, DefaultTwistStamp)
{
  // GIVEN a InputCheckValid with with a Visitor that counts
  auto visitor = CountingVisitor{};
  auto input_check_valid = InputCheckValid(std::make_shared<rclcpp::Node>("test_node"), "unitless", &visitor);

  // WHEN we visit with a default constructed TwistStamped
  std::visit(input_check_valid, InputCommand{ TwistStamped{} });

  // THEN we expect the visitor to have been called
  ASSERT_EQ(visitor.count, 1U) << "The visitor should have been called.";
}

TEST(InputCheckValidTests, DefaultJointJog)
{
  // GIVEN a InputCheckValid with with a Visitor that counts
  auto visitor = CountingVisitor{};
  auto input_check_valid = InputCheckValid(std::make_shared<rclcpp::Node>("test_node"), "unitless", &visitor);

  // WHEN we visit with a default constructed JointJog
  std::visit(input_check_valid, InputCommand{ JointJog{} });

  // THEN we expect the visitor to have been called
  ASSERT_EQ(visitor.count, 1U) << "The visitor should have been called.";
}

TEST(InputCheckValidTests, InvalidUnitlessTwistStamp)
{
  // GIVEN a InputCheckValid with with a Visitor that counts
  auto visitor = CountingVisitor{};
  auto input_check_valid = InputCheckValid(std::make_shared<rclcpp::Node>("test_node"), "unitless", &visitor);

  // WHEN we visit with a TwistStamped that is invlid in "unitless mode"
  auto command = TwistStamped{};
  command.twist.angular.z = 10.f;
  std::visit(input_check_valid, InputCommand{ command });

  // THEN we expect the visitor was not called
  ASSERT_EQ(visitor.count, 0U) << "The visitor should NOT have been called.";
}

TEST(InputCheckValidTests, ValidNotUnitlessTwistStamp)
{
  // GIVEN a InputCheckValid with with a Visitor that counts
  auto visitor = CountingVisitor{};
  auto input_check_valid = InputCheckValid(std::make_shared<rclcpp::Node>("test_node"), "", &visitor);

  // WHEN we visit with a TwistStamped that is invlid in "unitless mode"
  auto command = TwistStamped{};
  command.twist.angular.z = 10.f;
  std::visit(input_check_valid, InputCommand{ command });

  // THEN we expect the visitor was called
  ASSERT_EQ(visitor.count, 1U) << "The visitor should have been called.";
}

TEST(InputCheckValidTests, NanTwistStamp)
{
  // GIVEN a InputCheckValid with with a Visitor that counts
  auto visitor = CountingVisitor{};
  auto input_check_valid = InputCheckValid(std::make_shared<rclcpp::Node>("test_node"), "unitless", &visitor);

  // WHEN we visit with a TwistStamped containing a NaN
  auto command = TwistStamped{};
  command.twist.angular.z = NAN;
  std::visit(input_check_valid, InputCommand{ command });

  // THEN we expect the visitor was not called
  ASSERT_EQ(visitor.count, 0U) << "The visitor should NOT have been called.";
}

TEST(InputCheckValidTests, NanJointJog)
{
  // GIVEN a InputCheckValid with with a Visitor that counts
  auto visitor = CountingVisitor{};
  auto input_check_valid = InputCheckValid(std::make_shared<rclcpp::Node>("test_node"), "unitless", &visitor);

  // WHEN we visit with a JointJog containing a NaN
  auto command = JointJog{};
  command.velocities.push_back(NAN);
  std::visit(input_check_valid, InputCommand{ command });

  // THEN we expect the visitor was not called
  ASSERT_EQ(visitor.count, 0U) << "The visitor should NOT have been called.";
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
