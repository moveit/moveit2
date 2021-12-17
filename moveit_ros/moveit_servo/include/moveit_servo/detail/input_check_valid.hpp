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

/*      Title     : input_check_valid.hpp
 *      Project   : moveit_servo
 *      Created   : 11/06/2021
 *      Author    : Tyler Weaver
 */

#pragma once

#include <functional>
#include <variant>
#include <memory>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_servo/detail/input_command.hpp>
#include <rclcpp/rclcpp.hpp>

namespace moveit_servo
{
namespace detail
{
/**
 * @brief      Visitor that checks if the input is valid, only calling next if that is true
 */
class InputCheckValid : public InputVisitor
{
  rclcpp::Node::SharedPtr node_ = nullptr;
  std::string command_in_type_ = "unitless";
  std::shared_ptr<InputVisitor> next_ = nullptr;

public:
  InputCheckValid(const rclcpp::Node::SharedPtr& node, std::string_view command_in_type,
                  const std::shared_ptr<InputVisitor>& next)
    : node_{ node }, command_in_type_{ command_in_type }, next_{ next }
  {
    assert(node_ != nullptr);
    assert(next_ != nullptr);
  }
  ~InputCheckValid() override = default;

  void operator()(const geometry_msgs::msg::TwistStamped& command) override;
  void operator()(const control_msgs::msg::JointJog& command) override;
};

}  // namespace detail
}  // namespace moveit_servo
