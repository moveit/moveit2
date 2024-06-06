/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#pragma once

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

namespace trajectory_processing
{
/**
 * \brief Checks if a robot trajectory is empty.
 * \param [in] trajectory The robot trajectory to check.
 * \return True if the trajectory is empty, false otherwise.
 */
bool isTrajectoryEmpty(const moveit_msgs::msg::RobotTrajectory& trajectory);
/**
 * \brief Returns the number of waypoints in a robot trajectory.
 * \param [in] trajectory The robot trajectory to count waypoints in.
 * \return The number of waypoints in the trajectory.
 */
std::size_t trajectoryWaypointCount(const moveit_msgs::msg::RobotTrajectory& trajectory);
/**
 * \brief Applies time parameterization to a robot trajectory using the Time-Optimal Trajectory Generation (TOTG)
 * algorithm.
 * \param [in,out] trajectory The robot trajectory to be time parameterized.
 * \param [in] velocity_scaling_factor The factor by which to scale the maximum velocity of the trajectory.
 * \param [in] acceleration_scaling_factor The factor by which to scale the maximum acceleration of the trajectory.
 * \param [in] path_tolerance The path tolerance to use for time parameterization (default: 0.1).
 * \param [in] resample_dt The time step to use for time parameterization (default: 0.1).
 * \param [in] min_angle_change The minimum angle change to use for time parameterization (default: 0.001).
 * \return True if time parameterization was successful, false otherwise.
 */
bool applyTOTGTimeParameterization(robot_trajectory::RobotTrajectory& trajectory, double velocity_scaling_factor,
                                   double acceleration_scaling_factor, double path_tolerance = 0.1,
                                   double resample_dt = 0.1, double min_angle_change = 0.001);
/**
 * \brief Applies Ruckig smoothing to a robot trajectory.
 * \param [in,out] trajectory The robot trajectory to be smoothed.
 * \param [in] velocity_scaling_factor The factor by which to scale the maximum velocity of the trajectory.
 * \param [in] acceleration_scaling_factor The factor by which to scale the maximum acceleration of the trajectory.
 * \param [in] mitigate_overshoot Whether to mitigate overshoot during smoothing (default: false).
 * \param [in] overshoot_threshold The maximum allowed overshoot during smoothing (default: 0.01).
 * \return True if smoothing was successful, false otherwise.
 */
bool applyRuckigSmoothing(robot_trajectory::RobotTrajectory& trajectory, double velocity_scaling_factor,
                          double acceleration_scaling_factor, bool mitigate_overshoot = false,
                          double overshoot_threshold = 0.01);

/**
 * @brief Converts a `trajectory_processing::Trajectory` into a `JointTrajectory` message with a given sampling rate.
 */
[[nodiscard]] trajectory_msgs::msg::JointTrajectory
createTrajectoryMessage(const std::vector<std::string>& joint_names,
                        const trajectory_processing::Trajectory& trajectory, const int sampling_rate);
}  // namespace trajectory_processing
