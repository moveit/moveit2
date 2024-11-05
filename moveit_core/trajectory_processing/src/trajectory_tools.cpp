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

#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
namespace trajectory_processing
{

namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.trajectory_processing.trajectory_tools");
}
}  // namespace

bool isTrajectoryEmpty(const moveit_msgs::msg::RobotTrajectory& trajectory)
{
  return trajectory.joint_trajectory.points.empty() && trajectory.multi_dof_joint_trajectory.points.empty();
}

std::size_t trajectoryWaypointCount(const moveit_msgs::msg::RobotTrajectory& trajectory)
{
  return std::max(trajectory.joint_trajectory.points.size(), trajectory.multi_dof_joint_trajectory.points.size());
}
bool applyTOTGTimeParameterization(robot_trajectory::RobotTrajectory& trajectory, double velocity_scaling_factor,
                                   double acceleration_scaling_factor, double path_tolerance, double resample_dt,
                                   double min_angle_change)
{
  TimeOptimalTrajectoryGeneration totg(path_tolerance, resample_dt, min_angle_change);
  return totg.computeTimeStamps(trajectory, velocity_scaling_factor, acceleration_scaling_factor);
}
bool applyRuckigSmoothing(robot_trajectory::RobotTrajectory& trajectory, double velocity_scaling_factor,
                          double acceleration_scaling_factor, bool mitigate_overshoot, double overshoot_threshold)
{
  RuckigSmoothing time_param;
  return time_param.applySmoothing(trajectory, velocity_scaling_factor, acceleration_scaling_factor, mitigate_overshoot,
                                   overshoot_threshold);
}

trajectory_msgs::msg::JointTrajectory createTrajectoryMessage(const std::vector<std::string>& joint_names,
                                                              const trajectory_processing::Trajectory& trajectory,
                                                              const int sampling_rate)
{
  trajectory_msgs::msg::JointTrajectory trajectory_msg;
  if (sampling_rate <= 0)
  {
    RCLCPP_ERROR(getLogger(), "Cannot sample trajectory with sampling rate <= 0. Returning empty trajectory.");
    return trajectory_msg;
  }
  trajectory_msg.joint_names = joint_names;
  const double time_step = 1.0 / static_cast<double>(sampling_rate);
  const int n_samples = static_cast<int>(std::ceil(trajectory.getDuration() / time_step)) + 1;
  trajectory_msg.points.reserve(n_samples);
  for (int sample = 0; sample < n_samples; ++sample)
  {
    const double t = sample * time_step;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    auto position = trajectory.getPosition(t);
    auto velocity = trajectory.getVelocity(t);
    auto acceleration = trajectory.getAcceleration(t);
    for (std::size_t i = 0; i < joint_names.size(); i++)
    {
      point.positions.push_back(position[i]);
      point.velocities.push_back(velocity[i]);
      point.accelerations.push_back(acceleration[i]);
    }
    point.time_from_start = rclcpp::Duration(std::chrono::duration<double>(t));
    trajectory_msg.points.push_back(std::move(point));
  }
  return trajectory_msg;
}
}  // namespace trajectory_processing
