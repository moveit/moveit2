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
   Desc:   Input Visitor Mocks
*/

#include <variant>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_servo/detail/input_command.hpp>
#include <rclcpp/rclcpp.hpp>

using control_msgs::msg::JointJog;
using geometry_msgs::msg::TwistStamped;

namespace moveit_servo
{
class DoNothingVisitor : public detail::InputVisitor
{
public:
  void operator()(TwistStamped /*unused*/) override{};
  void operator()(JointJog /*unused*/) override{};
  ~DoNothingVisitor() override = default;
};

class CountingVisitor : public detail::InputVisitor
{
public:
  unsigned int count = 0;
  void operator()(TwistStamped /*unused*/) override
  {
    count++;
  };
  void operator()(JointJog /*unused*/) override
  {
    count++;
  };
  ~CountingVisitor() override = default;
};

class TypeCountingVisitor : public detail::InputVisitor
{
public:
  unsigned int twist_stamped_count = 0;
  unsigned int joint_jog_count = 0;
  void operator()(TwistStamped /*unused*/) override
  {
    twist_stamped_count++;
  };
  void operator()(JointJog /*unused*/) override
  {
    joint_jog_count++;
  };
  ~TypeCountingVisitor() override = default;
};

class ReceivedCommandVisitor : public detail::InputVisitor
{
public:
  detail::InputCommand received_command;
  void operator()(TwistStamped command) override
  {
    received_command = command;
  };
  void operator()(JointJog command) override
  {
    received_command = command;
  };
  ~ReceivedCommandVisitor() override = default;
};

}  // namespace moveit_servo
