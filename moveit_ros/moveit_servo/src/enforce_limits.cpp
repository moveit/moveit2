/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*      Title     : enforce_limits.cpp
 *      Project   : moveit_servo
 *      Created   : 7/5/2021
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson, Tyler Weaver
 */

#include <Eigen/Core>
#include <sensor_msgs/msg/joint_state.hpp>

#include <moveit/robot_model/joint_model_group.h>
#include <moveit_servo/enforce_limits.hpp>

namespace moveit_servo
{
namespace
{
double getVelocityScalingFactor(const moveit::core::JointModelGroup* joint_model_group, const Eigen::VectorXd& velocity)
{
  std::size_t joint_delta_index{ 0 };
  double velocity_scaling_factor{ 1.0 };
  for (const moveit::core::JointModel* joint : joint_model_group->getActiveJointModels())
  {
    const auto& bounds = joint->getVariableBounds(joint->getName());
    if (bounds.velocity_bounded_ && velocity(joint_delta_index) != 0.0)
    {
      const double unbounded_velocity = velocity(joint_delta_index);
      // Clamp each joint velocity to a joint specific [min_velocity, max_velocity] range.
      const auto bounded_velocity = std::min(std::max(unbounded_velocity, bounds.min_velocity_), bounds.max_velocity_);
      velocity_scaling_factor = std::min(velocity_scaling_factor, bounded_velocity / unbounded_velocity);
    }
    ++joint_delta_index;
  }

  return velocity_scaling_factor;
}

}  // namespace

void enforceVelocityLimits(const moveit::core::JointModelGroup* joint_model_group, const double publish_period,
                           sensor_msgs::msg::JointState& joint_state, const double override_velocity_scaling_factor)
{
  // Get the velocity scaling factor
  Eigen::VectorXd velocity =
      Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(joint_state.velocity.data(), joint_state.velocity.size());
  double velocity_scaling_factor = override_velocity_scaling_factor;
  // if the override velocity scaling factor is approximately zero then the user is not overriding the value.
  if (override_velocity_scaling_factor < 0.01)
    velocity_scaling_factor = getVelocityScalingFactor(joint_model_group, velocity);

  // Take a smaller step if the velocity scaling factor is less than 1
  if (velocity_scaling_factor < 1)
  {
    Eigen::VectorXd velocity_residuals = (1 - velocity_scaling_factor) * velocity;
    Eigen::VectorXd positions =
        Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(joint_state.position.data(), joint_state.position.size());
    positions -= velocity_residuals * publish_period;

    velocity *= velocity_scaling_factor;
    // Back to sensor_msgs type
    joint_state.velocity = std::vector<double>(velocity.data(), velocity.data() + velocity.size());
    joint_state.position = std::vector<double>(positions.data(), positions.data() + positions.size());
  }
}

}  // namespace moveit_servo
