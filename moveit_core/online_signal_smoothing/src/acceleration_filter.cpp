/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, PickNik Inc.
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
   Description: Applies smoothing by limiting the  acceleration between consecutive commands
 */

#include <moveit/online_signal_smoothing/acceleration_filter.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>

// Disable -Wold-style-cast because all _THROTTLE macros trigger this
#pragma GCC diagnostic ignored "-Wold-style-cast"

namespace online_signal_smoothing
{
// The threshold above which `override_velocity_scaling_factor` will be used instead of computing the scaling from joint bounds.
constexpr double SCALING_OVERRIDE_THRESHOLD = 0.01;

bool AccelerationLimitedPlugin::initialize(rclcpp::Node::SharedPtr node, moveit::core::RobotModelConstPtr robot_model,
                                           size_t num_joints)
{
  node_ = node;
  num_joints_ = num_joints;
  robot_model_ = robot_model;
  cur_acceleration_ = Eigen::VectorXd::Zero(num_joints);

  auto param_listener = online_signal_smoothing::ParamListener(node_);
  auto params = param_listener.get_params();
  update_rate_ = params.update_rate;
  move_group_name_ = params.move_group_name;

  return true;
}

double jointLimitAccelerationScalingFactor(const Eigen::VectorXd& accelerations,
                                           const moveit::core::JointBoundsVector& joint_bounds)
{
  double min_scaling_factor = 1.0;

  // Now get the scaling factor from joint limits.
  size_t idx = 0;
  for (const auto& joint_bound : joint_bounds)
  {
    for (const auto& variable_bound : *joint_bound)
    {
      const auto& target_accel = accelerations(idx);
      if (variable_bound.acceleration_bounded_ && target_accel != 0.0)
      {
        // Find the ratio of clamped acceleration to original acceleration
        const auto bounded_vel =
            std::clamp(target_accel, variable_bound.min_acceleration_, variable_bound.max_acceleration_);
        double joint_scaling_factor = bounded_vel / target_accel;
        min_scaling_factor = std::min(min_scaling_factor, joint_scaling_factor);
      }
      ++idx;
    }
  }

  return min_scaling_factor;
}

bool AccelerationLimitedPlugin::doSmoothing(Eigen::VectorXd& positions, Eigen::VectorXd& velocities,
                                            Eigen::VectorXd& /* unused */)
{
  const size_t num_positions = velocities.size();
  if (num_positions != num_joints_)
  {
    RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "The length of the joint positions parameter is not equal to the number of joints, expected %zu got %zu.",
        num_joints_, num_positions);
    return false;
  }
  else if (last_velocities_.size() != (long)num_joints_)
  {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "Make sure reset is called before doSmoothing for the AccelerationLimitedPlugin plugin.");
    return false;
  }

  // estimate velocity and acceleration with finite difference using two previous positions
  cur_acceleration_ = (velocities - last_velocities_) / update_rate_;

  // scale acceleration using robot joint limits
  auto joint_model_group = robot_model_->getJointModelGroup(move_group_name_);
  cur_acceleration_ *=
      jointLimitAccelerationScalingFactor(cur_acceleration_, joint_model_group->getActiveJointModelsBounds());

  // estimate new filtered velocity using clamped acceleration
  velocities = last_velocities_ + cur_acceleration_ * update_rate_;
  last_velocities_ = velocities;
  positions = last_positions_ + last_velocities_ * update_rate_;
  last_positions_ = positions;

  return true;
}

bool AccelerationLimitedPlugin::reset(const Eigen::VectorXd& positions, const Eigen::VectorXd& velocities,
                                      const Eigen::VectorXd& /* unused */)
{
  last_velocities_ = velocities;
  last_positions_ = positions;
  cur_acceleration_ = Eigen::VectorXd::Zero(num_joints_);

  return true;
}

}  // namespace online_signal_smoothing

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(online_signal_smoothing::AccelerationLimitedPlugin, online_signal_smoothing::SmoothingBaseClass)
