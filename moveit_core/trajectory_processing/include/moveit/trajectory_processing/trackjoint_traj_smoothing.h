/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, PickNik Robotics
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

/* Author: Jack Center, Wyatt Rees, Andy Zelenak */

#pragma once

#include <Eigen/Core>
#include <list>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <trackjoint/trajectory_generator.h>

namespace trajectory_processing
{
class TrackJointSmoothing
{
public:
  static bool applySmoothing(robot_trajectory::RobotTrajectory& desired_trajectory,
                             const double max_velocity_scaling_factor = 1.0,
                             const double max_acceleration_scaling_factor = 1.0);

private:
  /**
   * \brief Set TrackJoint position/vel/accel
   */
  static void setTrackJointState(const size_t waypoint_idx, const robot_trajectory::RobotTrajectory& trajectory,
                                 const size_t num_dof, const std::vector<int>& joint_group_indices,
                                 std::vector<trackjoint::KinematicState>& joint_states);

  /**
   * \brief Set TrackJoint position/vel/accel limits
   */
  static void setTrackJointLimits(const moveit::core::JointModelGroup* group, const size_t num_dof,
                                  const double max_velocity_scaling_factor,
                                  const double max_acceleration_scaling_factor,
                                  std::vector<trackjoint::Limits>& limits);

  static void
  buildRobotTrajectoryFromTrackJointOutput(const robot_trajectory::RobotTrajectory& desired_trajectory,
                                           const size_t num_dof, const std::vector<int>& joint_group_indices,
                                           const std::vector<trackjoint::JointTrajectory>& trackjoint_output,
                                           robot_trajectory::RobotTrajectory& trajectory);
};
}  // namespace trajectory_processing
