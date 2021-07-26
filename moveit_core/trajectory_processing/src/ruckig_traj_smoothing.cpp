/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Tobias Kunz <tobias@gatech.edu>
 * Date: 05/2012
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * Algorithm details and publications:
 * http://www.golems.org/node/1570
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <algorithm>
#include <cmath>
#include <Eigen/Geometry>
#include <limits>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#include <rclcpp/rclcpp.hpp>
#include <ruckig/ruckig.hpp>
#include <vector>

namespace trajectory_processing
{
static const rclcpp::Logger LOGGER =
    rclcpp::get_logger("moveit_trajectory_processing.time_optimal_trajectory_generation");
constexpr double DEFAULT_MAX_VELOCITY = DBL_MAX;      // rad/s
constexpr double DEFAULT_MAX_ACCELERATION = DBL_MAX;  // rad/s^2
constexpr double DEFAULT_MAX_JERK = DBL_MAX;          // rad/s^3

bool RuckigSmoothing::computeTimeStamps(robot_trajectory::RobotTrajectory& trajectory,
                                        const double max_velocity_scaling_factor,
                                        const double max_acceleration_scaling_factor) const
{
  const moveit::core::JointModelGroup* group = trajectory.getGroup();
  if (!group)
  {
    RCLCPP_ERROR(LOGGER, "It looks like the planner did not set the group the plan was computed for");
    return false;
  }

  size_t num_waypoints = trajectory.getWayPointCount();
  if (num_waypoints < 2)
  {
    RCLCPP_WARN_STREAM(LOGGER, "Trajectory does not have enough points to smooth with Ruckig");
    return false;
  }

  const size_t num_dof = group->getVariableCount();

  // This lib does not actually work properly when angles wrap around, so we need to unwind the path first
  trajectory.unwind();

  // Instantiate the smoother
  double timestep = trajectory.getAverageSegmentDuration();
  RCLCPP_ERROR_STREAM(LOGGER, "timestep: " << timestep);
  ruckig::Ruckig<0> ruckig{ num_dof, timestep };
  ruckig::InputParameter<0> ruckig_input{ num_dof };
  ruckig::OutputParameter<0> ruckig_output{ num_dof };

  // Initialize the smoother
  //  ruckig_input.synchronization = ruckig::Synchronization::Time;
  //  ruckig_input.interface = ruckig::Interface::Velocity;
  const std::vector<int>& idx = group->getVariableIndexList();
  std::vector<double> current_positions_vector(num_dof);
  std::vector<double> current_velocities_vector(num_dof);
  std::vector<double> current_accelerations_vector(num_dof);
  moveit::core::RobotStatePtr waypoint = trajectory.getFirstWayPointPtr();
  for (size_t i = 0; i < num_dof; ++i)
  {
    current_positions_vector.at(i) = waypoint->getVariablePosition(idx[i]);
    current_velocities_vector.at(i) = waypoint->getVariableVelocity(idx[i]);
    current_accelerations_vector.at(i) = waypoint->getVariableAcceleration(idx[i]);
    RCLCPP_ERROR_STREAM(LOGGER, "Initial position: " << current_positions_vector.at(i));
    RCLCPP_ERROR_STREAM(LOGGER, "Initial velocity: " << current_velocities_vector.at(i));
    RCLCPP_ERROR_STREAM(LOGGER, "Initial acceleration: " << current_accelerations_vector.at(i));
  }
  std::copy_n(current_positions_vector.begin(), num_dof, ruckig_input.current_position.begin());
  std::copy_n(current_velocities_vector.begin(), num_dof, ruckig_input.current_velocity.begin());
  std::copy_n(current_accelerations_vector.begin(), num_dof, ruckig_input.current_acceleration.begin());
  // Initialize output data struct
  ruckig_output.new_position = ruckig_input.current_position;
  ruckig_output.new_velocity = ruckig_input.current_velocity;
  ruckig_output.new_acceleration = ruckig_input.current_acceleration;

  // Kinematic limits (vel/accel/jerk)
  const std::vector<std::string>& vars = group->getVariableNames();
  const moveit::core::RobotModel& rmodel = group->getParentModel();
  for (size_t i = 0; i < num_dof; ++i)
  {
    // TODO(andyz): read this from the joint group if/when jerk becomes supported
    ruckig_input.max_jerk[i] = DEFAULT_MAX_JERK;

    const moveit::core::VariableBounds& bounds = rmodel.getVariableBounds(vars[i]);

    // This assumes min/max bounds are symmetric
    if (bounds.velocity_bounded_)
    {
      ruckig_input.max_velocity[i] = max_velocity_scaling_factor * bounds.max_velocity_;
    }
    else
    {
      ruckig_input.max_velocity[i] = max_velocity_scaling_factor * DEFAULT_MAX_VELOCITY;
    }
    if (bounds.acceleration_bounded_)
    {
      ruckig_input.max_acceleration[i] = max_acceleration_scaling_factor * bounds.max_acceleration_;
    }
    else
    {
      ruckig_input.max_acceleration[i] = max_acceleration_scaling_factor * DEFAULT_MAX_ACCELERATION;
    }

    RCLCPP_ERROR_STREAM(LOGGER, "Max vel: " << ruckig_input.max_velocity[i]);
    RCLCPP_ERROR_STREAM(LOGGER, "Max accel: " << ruckig_input.max_acceleration[i]);
    RCLCPP_ERROR_STREAM(LOGGER, "Max jerk: " << ruckig_input.max_jerk[i]);
  }

  ruckig::Result ruckig_result;
  for (size_t waypoint_idx = 0; waypoint_idx < num_waypoints - 1; ++waypoint_idx)
  {
    waypoint = trajectory.getWayPointPtr(waypoint_idx);
    moveit::core::RobotStatePtr next_waypoint = trajectory.getWayPointPtr(waypoint_idx + 1);
    for (size_t joint = 0; joint < num_dof; ++joint)
    {
      // Feed output from the previous timestep back as input
      ruckig_input.current_position[joint] = ruckig_output.new_position[joint];
      ruckig_input.current_velocity[joint] = ruckig_output.new_velocity[joint];
      ruckig_input.current_acceleration[joint] = ruckig_output.new_acceleration[joint];

      // Target state is the next waypoint
      ruckig_input.target_position[joint] = next_waypoint->getVariablePosition(idx[joint]);
      ruckig_input.target_velocity[joint] = next_waypoint->getVariableVelocity(idx[joint]);
      ruckig_input.target_acceleration[joint] = next_waypoint->getVariableAcceleration(idx[joint]);

      RCLCPP_ERROR_STREAM(LOGGER, "Target pos: " << ruckig_input.target_position[joint]
                                                 << "  Current pos: " << ruckig_input.current_position[joint]);
      RCLCPP_ERROR_STREAM(LOGGER, "Target vel: " << ruckig_input.target_velocity[joint]
                                                 << "  Current vel: " << ruckig_input.current_velocity[joint]);
      RCLCPP_ERROR_STREAM(LOGGER, "Target accel: " << ruckig_input.target_acceleration[joint]
                                                   << "  Current accel: " << ruckig_input.current_acceleration[joint]);
    }
    ruckig_result = ruckig.update(ruckig_input, ruckig_output);

    if (ruckig_result != ruckig::Result::Working)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Ruckig trajectory smoothing failed at waypoint " << waypoint_idx);
      RCLCPP_ERROR_STREAM(LOGGER, "Ruckig error: " << ruckig_result);
      return false;
    }

    // Overwrite pos/vel/accel of the target waypoint
    for (size_t joint = 0; joint < num_dof; ++joint)
    {
      // RCLCPP_ERROR_STREAM(LOGGER, "Target pos: " << ruckig_input.target_position[joint]
      //                                            << "  Smoothed pos: " << ruckig_output.new_position[joint]);
      // RCLCPP_ERROR_STREAM(LOGGER, "Target vel: " << ruckig_input.target_velocity[joint]
      //                                            << "  Smoothed vel: " << ruckig_output.new_velocity[joint]);
      // RCLCPP_ERROR_STREAM(LOGGER, "Target accel: " << ruckig_input.target_acceleration[joint]
      //                                              << "  Smoothed accel: " << ruckig_output.new_acceleration[joint]);
      next_waypoint->setVariablePosition(idx[joint], ruckig_output.new_position[joint]);
      next_waypoint->setVariableVelocity(idx[joint], ruckig_output.new_velocity[joint]);
      next_waypoint->setVariableAcceleration(idx[joint], ruckig_output.new_acceleration[joint]);
    }
  }

  return true;
}
}  // namespace trajectory_processing
