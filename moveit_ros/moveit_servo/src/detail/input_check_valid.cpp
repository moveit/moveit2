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

/*      Title     : input_check_valid.cpp
 *      Project   : moveit_servo
 *      Created   : 11/06/2021
 *      Author    : Tyler Weaver
 */

#include <cmath>
#include <functional>
#include <memory>
#include <variant>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_servo/detail/input_command.hpp>
#include <rclcpp/rclcpp.hpp>

#include <moveit_servo/detail/input_check_valid.hpp>

namespace moveit_servo
{
namespace detail
{
namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.input_check_valid");
constexpr auto ROS_LOG_THROTTLE_PERIOD = std::chrono::milliseconds(1000).count();
}  // namespace

using std::fabs;
using std::isnan;

void InputCheckValid::operator()(geometry_msgs::msg::TwistStamped command)
{
  if (isnan(command.twist.linear.x) || isnan(command.twist.linear.y) || isnan(command.twist.linear.z) ||
      isnan(command.twist.angular.x) || isnan(command.twist.angular.y) || isnan(command.twist.angular.z))
  {
    rclcpp::Clock& clock = *node_->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD,
                                "nan in incoming command. Skipping this datapoint.");
    return;
  }

  // If incoming commands should be in the range [-1:1], check for |delta|>1
  if (command_in_type_ == "unitless")
  {
    if ((fabs(command.twist.linear.x) > 1) || (fabs(command.twist.linear.y) > 1) ||
        (fabs(command.twist.linear.z) > 1) || (fabs(command.twist.angular.x) > 1) ||
        (fabs(command.twist.angular.y) > 1) || (fabs(command.twist.angular.z) > 1))
    {
      rclcpp::Clock& clock = *node_->get_clock();
      RCLCPP_WARN_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD,
                                  "Component of incoming command is >1. Skipping this datapoint.");
      return;
    }
  }

  // All is good, visit the next one
  std::visit(*next_, InputCommand{ command });
}

void InputCheckValid::operator()(control_msgs::msg::JointJog command)
{
  for (double velocity : command.velocities)
  {
    if (isnan(velocity))
    {
      rclcpp::Clock& clock = *node_->get_clock();
      RCLCPP_WARN_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD,
                                  "nan in incoming command. Skipping this datapoint.");
      return;
    }
  }

  // All is good, visit the next one
  std::visit(*next_, InputCommand{ command });
}

}  // namespace detail
}  // namespace moveit_servo
