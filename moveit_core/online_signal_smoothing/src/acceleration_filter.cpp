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

/* Author: Paul Gesel
   Description: Applies smoothing by limiting the  acceleration between consecutivee commands
 */

#include <moveit/online_signal_smoothing/acceleration_filter.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>

// Disable -Wold-style-cast because all _THROTTLE macros trigger this
#pragma GCC diagnostic ignored "-Wold-style-cast"

namespace online_signal_smoothing
{

bool AccelerationLimitedPlugin::initialize(rclcpp::Node::SharedPtr node, moveit::core::RobotModelConstPtr /* unused */,
                                           size_t num_joints)
{
  node_ = node;
  num_joints_ = num_joints;

  cur_velocity_ = Eigen::VectorXd::Zero(num_joints);
  prev_velocity_ = Eigen::VectorXd::Zero(num_joints);
  cur_acceleration = Eigen::VectorXd::Zero(num_joints);

  auto param_listener_ = online_signal_smoothing::ParamListener(node_);
  auto params = param_listener_.get_params();
  joint_acceleration_limit_ = params.joint_acceleration_limit;
  update_timeout_ = params.update_timeout;

  return true;
}

bool AccelerationLimitedPlugin::doSmoothing(Eigen::VectorXd& positions, Eigen::VectorXd& /* unused */,
                                            Eigen::VectorXd& /* unused */)
{
  const size_t num_positions = positions.size();
  if (num_positions != num_joints_)
  {
    RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "The length of the joint positions parameter is not equal to the number of joints, expected %zu got %zu.",
        num_joints_, num_positions);
    return false;
  }
  else if (last_positions_[0].first.size() != num_joints_)
  {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "Reset must be called before doSmoothing for the AccelerationLimitedPlugin plugin.");
    return false;
  }

  if (!last_positions_[0].second.has_value())
  {
    positions = last_positions_[0].first;
    last_positions_[0].second = node_->now();
  }
  else if (!last_positions_[1].second.has_value())
  {
    positions = last_positions_[1].first;
    last_positions_[1].second = node_->now();
  }
  else
  {
    rclcpp::Time cur_time = node_->now();
    double dt_1 = (last_positions_[1].second.value() - last_positions_[0].second.value()).seconds();
    double dt_2 = (cur_time - last_positions_[1].second.value()).seconds();
    if (dt_1 > update_timeout_ || dt_2 > update_timeout_)
    {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                            "The maximum allowed time between filter updates exceeded! The `doSmoothing` method must "
                            "be called at a higher rate.");
      return false;
    }

    prev_velocity_ = (last_positions_[1].first - last_positions_[0].first) / dt_1;
    cur_velocity_ = (positions - last_positions_[1].first) / dt_2;
    cur_acceleration = (cur_velocity_ - prev_velocity_) / dt_2;
    double scale = 1.0;
    for (size_t i = 0; i < num_joints_; i++)
    {
      scale = std::min(scale, std::clamp(cur_acceleration[i], -joint_acceleration_limit_, joint_acceleration_limit_) /
                                  cur_acceleration[i]);
    }
    cur_acceleration *= scale;
    cur_velocity_ = prev_velocity_ + cur_acceleration * dt_2;
    positions = last_positions_[1].first + cur_velocity_ * dt_2;

    last_positions_[0] = last_positions_[1];
    last_positions_[1].first = positions;
    last_positions_[1].second = cur_time;
  }

  return true;
}

bool AccelerationLimitedPlugin::reset(const Eigen::VectorXd& positions, const Eigen::VectorXd& /* unused */,
                                      const Eigen::VectorXd& /* unused */)
{
  last_positions_[0] = { positions, {} };
  last_positions_[1] = { positions, {} };
  cur_velocity_ = Eigen::VectorXd::Zero(num_joints_);
  prev_velocity_ = Eigen::VectorXd::Zero(num_joints_);
  cur_acceleration = Eigen::VectorXd::Zero(num_joints_);

  return true;
}

}  // namespace online_signal_smoothing

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(online_signal_smoothing::AccelerationLimitedPlugin, online_signal_smoothing::SmoothingBaseClass)
