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

/* Author: Jack Center, Wyatt Rees, Andy Zelenak, Stephanie Eng */

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <algorithm>
#include <cmath>
#include <Eigen/Geometry>
#include <limits>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#include <vector>
#include <moveit/utils/logger.hpp>

namespace trajectory_processing
{
namespace
{
constexpr double DEFAULT_MAX_VELOCITY = 5;       // rad/s
constexpr double DEFAULT_MAX_ACCELERATION = 10;  // rad/s^2
constexpr double DEFAULT_MAX_JERK = 1000;        // rad/s^3
constexpr double MAX_DURATION_EXTENSION_FACTOR = 10.0;
constexpr double DURATION_EXTENSION_FRACTION = 1.1;
// If "mitigate_overshoot" is enabled, overshoot is checked with this timestep
constexpr double OVERSHOOT_CHECK_PERIOD = 0.01;  // sec

rclcpp::Logger getLogger()
{
  return moveit::getLogger("ruckig_traj_smoothing");
}
}  // namespace

bool RuckigSmoothing::applySmoothing(robot_trajectory::RobotTrajectory& trajectory,
                                     const double max_velocity_scaling_factor,
                                     const double max_acceleration_scaling_factor, const bool mitigate_overshoot,
                                     const double overshoot_threshold)
{
  if (!validateGroup(trajectory))
  {
    return false;
  }

  const size_t num_waypoints = trajectory.getWayPointCount();
  if (num_waypoints < 2)
  {
    RCLCPP_WARN(getLogger(),
                "Trajectory does not have enough points to smooth with Ruckig. Returning an unmodified trajectory.");
    return true;
  }

  // Kinematic limits (vels/accels/jerks) from RobotModel
  const moveit::core::JointModelGroup* const group = trajectory.getGroup();
  const size_t num_dof = group->getVariableCount();
  ruckig::InputParameter<ruckig::DynamicDOFs> ruckig_input{ num_dof };
  if (!getRobotModelBounds(max_velocity_scaling_factor, max_acceleration_scaling_factor, group, ruckig_input))
  {
    RCLCPP_ERROR(getLogger(), "Error while retrieving kinematic limits (vel/accel/jerk) from RobotModel.");
    return false;
  }

  return runRuckig(trajectory, ruckig_input, mitigate_overshoot, overshoot_threshold);
}

bool RuckigSmoothing::applySmoothing(robot_trajectory::RobotTrajectory& trajectory,
                                     const std::unordered_map<std::string, double>& velocity_limits,
                                     const std::unordered_map<std::string, double>& acceleration_limits,
                                     const std::unordered_map<std::string, double>& jerk_limits,
                                     const double max_velocity_scaling_factor,
                                     const double max_acceleration_scaling_factor, const bool mitigate_overshoot,
                                     const double overshoot_threshold)
{
  if (!validateGroup(trajectory))
  {
    return false;
  }

  const size_t num_waypoints = trajectory.getWayPointCount();
  if (num_waypoints < 2)
  {
    RCLCPP_WARN(getLogger(),
                "Trajectory does not have enough points to smooth with Ruckig. Returning an unmodified trajectory.");
    return true;
  }

  // Set default kinematic limits (vels/accels/jerks)
  const moveit::core::JointModelGroup* const group = trajectory.getGroup();
  const size_t num_dof = group->getVariableCount();
  ruckig::InputParameter<ruckig::DynamicDOFs> ruckig_input{ num_dof };
  if (!getRobotModelBounds(max_velocity_scaling_factor, max_acceleration_scaling_factor, group, ruckig_input))
  {
    RCLCPP_ERROR(getLogger(), "Error while retrieving kinematic limits (vel/accel/jerk) from RobotModel.");
    return false;
  }

  // Check if custom limits were supplied as arguments to overwrite the defaults
  const std::vector<std::string>& vars = group->getVariableNames();
  const unsigned num_joints = group->getVariableCount();
  for (size_t j = 0; j < num_joints; ++j)
  {
    // Velocity
    auto it = velocity_limits.find(vars[j]);
    if (it != velocity_limits.end())
    {
      ruckig_input.max_velocity.at(j) = it->second * max_velocity_scaling_factor;
    }
    // Acceleration
    it = acceleration_limits.find(vars[j]);
    if (it != acceleration_limits.end())
    {
      ruckig_input.max_acceleration.at(j) = it->second * max_acceleration_scaling_factor;
    }
    // Jerk
    it = jerk_limits.find(vars[j]);
    if (it != jerk_limits.end())
    {
      ruckig_input.max_jerk.at(j) = it->second;
    }
  }

  return runRuckig(trajectory, ruckig_input, mitigate_overshoot, overshoot_threshold);
}

bool RuckigSmoothing::applySmoothing(robot_trajectory::RobotTrajectory& trajectory,
                                     const std::vector<moveit_msgs::msg::JointLimits>& joint_limits,
                                     const double max_velocity_scaling_factor,
                                     const double max_acceleration_scaling_factor)
{
  std::unordered_map<std::string, double> velocity_limits;
  std::unordered_map<std::string, double> acceleration_limits;
  std::unordered_map<std::string, double> jerk_limits;
  for (const auto& limit : joint_limits)
  {
    // If custom limits are not defined here, they will be supplied from getRobotModelBounds() later
    if (limit.has_velocity_limits)
    {
      velocity_limits[limit.joint_name] = limit.max_velocity;
    }
    if (limit.has_acceleration_limits)
    {
      acceleration_limits[limit.joint_name] = limit.max_acceleration;
    }
    if (limit.has_jerk_limits)
    {
      jerk_limits[limit.joint_name] = limit.max_jerk;
    }
  }
  return applySmoothing(trajectory, velocity_limits, acceleration_limits, jerk_limits, max_velocity_scaling_factor,
                        max_acceleration_scaling_factor);
}

bool RuckigSmoothing::validateGroup(const robot_trajectory::RobotTrajectory& trajectory)
{
  const moveit::core::JointModelGroup* const group = trajectory.getGroup();
  if (!group)
  {
    RCLCPP_ERROR(getLogger(), "The planner did not set the group the plan was computed for");
    return false;
  }
  return true;
}

bool RuckigSmoothing::getRobotModelBounds(const double max_velocity_scaling_factor,
                                          const double max_acceleration_scaling_factor,
                                          const moveit::core::JointModelGroup* const group,
                                          ruckig::InputParameter<ruckig::DynamicDOFs>& ruckig_input)
{
  const size_t num_dof = group->getVariableCount();
  const std::vector<std::string>& vars = group->getVariableNames();
  const moveit::core::RobotModel& rmodel = group->getParentModel();
  for (size_t i = 0; i < num_dof; ++i)
  {
    const moveit::core::VariableBounds& bounds = rmodel.getVariableBounds(vars.at(i));

    // This assumes min/max bounds are symmetric
    if (bounds.velocity_bounded_)
    {
      ruckig_input.max_velocity.at(i) = max_velocity_scaling_factor * bounds.max_velocity_;
    }
    else
    {
      RCLCPP_WARN_STREAM_ONCE(getLogger(),
                              "Joint velocity limits are not defined. Using the default "
                                  << DEFAULT_MAX_VELOCITY
                                  << " rad/s. You can define velocity limits in the URDF or joint_limits.yaml.");
      ruckig_input.max_velocity.at(i) = max_velocity_scaling_factor * DEFAULT_MAX_VELOCITY;
    }
    if (bounds.acceleration_bounded_)
    {
      ruckig_input.max_acceleration.at(i) = max_acceleration_scaling_factor * bounds.max_acceleration_;
    }
    else
    {
      RCLCPP_WARN_STREAM_ONCE(getLogger(),
                              "Joint acceleration limits are not defined. Using the default "
                                  << DEFAULT_MAX_ACCELERATION
                                  << " rad/s^2. You can define acceleration limits in the URDF or joint_limits.yaml.");
      ruckig_input.max_acceleration.at(i) = max_acceleration_scaling_factor * DEFAULT_MAX_ACCELERATION;
    }
    ruckig_input.max_jerk.at(i) = bounds.jerk_bounded_ ? bounds.max_jerk_ : DEFAULT_MAX_JERK;
    if (bounds.jerk_bounded_)
    {
      ruckig_input.max_jerk.at(i) = bounds.max_jerk_;
    }
    else
    {
      RCLCPP_WARN_STREAM_ONCE(getLogger(), "Joint jerk limits are not defined. Using the default "
                                               << DEFAULT_MAX_JERK
                                               << " rad/s^3. You can define jerk limits in joint_limits.yaml.");
      ruckig_input.max_jerk.at(i) = DEFAULT_MAX_JERK;
    }
  }

  return true;
}

bool RuckigSmoothing::runRuckig(robot_trajectory::RobotTrajectory& trajectory,
                                ruckig::InputParameter<ruckig::DynamicDOFs>& ruckig_input,
                                const bool mitigate_overshoot, const double overshoot_threshold)
{
  const size_t num_waypoints = trajectory.getWayPointCount();
  const moveit::core::JointModelGroup* const group = trajectory.getGroup();
  const size_t num_dof = group->getVariableCount();
  ruckig::OutputParameter<ruckig::DynamicDOFs> ruckig_output{ num_dof };

  // This lib does not work properly when angles wrap, so we need to unwind the path first
  trajectory.unwind();

  // Initialize the smoother
  ruckig::Ruckig<ruckig::DynamicDOFs> ruckig(num_dof, trajectory.getAverageSegmentDuration());
  initializeRuckigState(*trajectory.getFirstWayPointPtr(), group, ruckig_input);

  // Cache the trajectory in case we need to reset it
  robot_trajectory::RobotTrajectory original_trajectory =
      robot_trajectory::RobotTrajectory(trajectory, true /* deep copy */);

  ruckig::Result ruckig_result;
  double duration_extension_factor = 1;
  bool smoothing_complete = false;
  size_t waypoint_idx = 0;
  while ((duration_extension_factor < MAX_DURATION_EXTENSION_FACTOR) && !smoothing_complete)
  {
    while (waypoint_idx < num_waypoints - 1)
    {
      moveit::core::RobotStatePtr curr_waypoint = trajectory.getWayPointPtr(waypoint_idx);
      moveit::core::RobotStatePtr next_waypoint = trajectory.getWayPointPtr(waypoint_idx + 1);

      getNextRuckigInput(curr_waypoint, next_waypoint, group, ruckig_input);

      // Run Ruckig
      ruckig::Trajectory<ruckig::DynamicDOFs, ruckig::StandardVector> ruckig_trajectory(num_dof);
      ruckig_result = ruckig.calculate(ruckig_input, ruckig_trajectory);

      // Step through the trajectory at the given OVERSHOOT_CHECK_PERIOD and check for overshoot.
      // We will extend the duration to mitigate it.
      bool overshoots = false;
      if (mitigate_overshoot)
      {
        overshoots = checkOvershoot(ruckig_trajectory, num_dof, ruckig_input, overshoot_threshold);
      }

      // The difference between Result::Working and Result::Finished is that Finished can be reached in one
      // Ruckig timestep (constructor parameter). Both are acceptable for trajectories.
      // (The difference is only relevant for streaming mode.)

      // If successful and at the last trajectory segment
      if (!overshoots && (waypoint_idx == num_waypoints - 2) &&
          (ruckig_result == ruckig::Result::Working || ruckig_result == ruckig::Result::Finished))
      {
        trajectory.setWayPointDurationFromPrevious(waypoint_idx + 1, ruckig_trajectory.get_duration());
        smoothing_complete = true;
        break;
      }

      // Extend the trajectory duration if Ruckig could not reach the waypoint successfully
      if (overshoots || (ruckig_result != ruckig::Result::Working && ruckig_result != ruckig::Result::Finished))
      {
        duration_extension_factor *= DURATION_EXTENSION_FRACTION;

        const std::vector<int>& move_group_idx = group->getVariableIndexList();
        extendTrajectoryDuration(duration_extension_factor, waypoint_idx, num_dof, move_group_idx, original_trajectory,
                                 trajectory);

        initializeRuckigState(*trajectory.getFirstWayPointPtr(), group, ruckig_input);
        // Continue the loop from failed segment, but with increased duration extension factor
        break;
      }
      ++waypoint_idx;
    }
  }

  if (ruckig_result != ruckig::Result::Working && ruckig_result != ruckig::Result::Finished)
  {
    RCLCPP_ERROR_STREAM(getLogger(), "Ruckig trajectory smoothing failed. Ruckig error: " << ruckig_result);
    return false;
  }

  return true;
}

void RuckigSmoothing::extendTrajectoryDuration(const double duration_extension_factor, size_t waypoint_idx,
                                               const size_t num_dof, const std::vector<int>& move_group_idx,
                                               const robot_trajectory::RobotTrajectory& original_trajectory,
                                               robot_trajectory::RobotTrajectory& trajectory)
{
  trajectory.setWayPointDurationFromPrevious(waypoint_idx + 1,
                                             duration_extension_factor *
                                                 original_trajectory.getWayPointDurationFromPrevious(waypoint_idx + 1));
  // re-calculate waypoint velocity and acceleration
  auto target_state = trajectory.getWayPointPtr(waypoint_idx + 1);
  const auto prev_state = trajectory.getWayPointPtr(waypoint_idx);

  double timestep = trajectory.getWayPointDurationFromPrevious(waypoint_idx + 1);

  for (size_t joint = 0; joint < num_dof; ++joint)
  {
    target_state->setVariableVelocity(move_group_idx.at(joint),
                                      (1 / duration_extension_factor) *
                                          target_state->getVariableVelocity(move_group_idx.at(joint)));

    double prev_velocity = prev_state->getVariableVelocity(move_group_idx.at(joint));
    double curr_velocity = target_state->getVariableVelocity(move_group_idx.at(joint));
    target_state->setVariableAcceleration(move_group_idx.at(joint), (curr_velocity - prev_velocity) / timestep);
  }
}

void RuckigSmoothing::initializeRuckigState(const moveit::core::RobotState& first_waypoint,
                                            const moveit::core::JointModelGroup* joint_group,
                                            ruckig::InputParameter<ruckig::DynamicDOFs>& ruckig_input)
{
  const size_t num_dof = joint_group->getVariableCount();
  const std::vector<int>& idx = joint_group->getVariableIndexList();

  std::vector<double> current_positions_vector(num_dof);
  std::vector<double> current_velocities_vector(num_dof);
  std::vector<double> current_accelerations_vector(num_dof);

  for (size_t i = 0; i < num_dof; ++i)
  {
    current_positions_vector.at(i) = first_waypoint.getVariablePosition(idx.at(i));
    current_velocities_vector.at(i) = first_waypoint.getVariableVelocity(idx.at(i));
    current_accelerations_vector.at(i) = first_waypoint.getVariableAcceleration(idx.at(i));
    // Clamp velocities/accelerations in case they exceed the limit due to small numerical errors
    current_velocities_vector.at(i) =
        std::clamp(current_velocities_vector.at(i), -ruckig_input.max_velocity.at(i), ruckig_input.max_velocity.at(i));
    current_accelerations_vector.at(i) = std::clamp(
        current_accelerations_vector.at(i), -ruckig_input.max_acceleration.at(i), ruckig_input.max_acceleration.at(i));
  }
  std::copy_n(current_positions_vector.begin(), num_dof, ruckig_input.current_position.begin());
  std::copy_n(current_velocities_vector.begin(), num_dof, ruckig_input.current_velocity.begin());
  std::copy_n(current_accelerations_vector.begin(), num_dof, ruckig_input.current_acceleration.begin());
}

void RuckigSmoothing::getNextRuckigInput(const moveit::core::RobotStateConstPtr& current_waypoint,
                                         const moveit::core::RobotStateConstPtr& next_waypoint,
                                         const moveit::core::JointModelGroup* joint_group,
                                         ruckig::InputParameter<ruckig::DynamicDOFs>& ruckig_input)
{
  const size_t num_dof = joint_group->getVariableCount();
  const std::vector<int>& idx = joint_group->getVariableIndexList();

  for (size_t joint = 0; joint < num_dof; ++joint)
  {
    ruckig_input.current_position.at(joint) = current_waypoint->getVariablePosition(idx.at(joint));
    ruckig_input.current_velocity.at(joint) = current_waypoint->getVariableVelocity(idx.at(joint));
    ruckig_input.current_acceleration.at(joint) = current_waypoint->getVariableAcceleration(idx.at(joint));

    // Target state is the next waypoint
    ruckig_input.target_position.at(joint) = next_waypoint->getVariablePosition(idx.at(joint));
    ruckig_input.target_velocity.at(joint) = next_waypoint->getVariableVelocity(idx.at(joint));
    ruckig_input.target_acceleration.at(joint) = next_waypoint->getVariableAcceleration(idx.at(joint));

    // Clamp velocities/accelerations in case they exceed the limit due to small numerical errors
    ruckig_input.current_velocity.at(joint) =
        std::clamp(ruckig_input.current_velocity.at(joint), -ruckig_input.max_velocity.at(joint),
                   ruckig_input.max_velocity.at(joint));
    ruckig_input.current_acceleration.at(joint) =
        std::clamp(ruckig_input.current_acceleration.at(joint), -ruckig_input.max_acceleration.at(joint),
                   ruckig_input.max_acceleration.at(joint));
    ruckig_input.target_velocity.at(joint) =
        std::clamp(ruckig_input.target_velocity.at(joint), -ruckig_input.max_velocity.at(joint),
                   ruckig_input.max_velocity.at(joint));
    ruckig_input.target_acceleration.at(joint) =
        std::clamp(ruckig_input.target_acceleration.at(joint), -ruckig_input.max_acceleration.at(joint),
                   ruckig_input.max_acceleration.at(joint));
  }
}

bool RuckigSmoothing::checkOvershoot(ruckig::Trajectory<ruckig::DynamicDOFs, ruckig::StandardVector>& ruckig_trajectory,
                                     const size_t num_dof, ruckig::InputParameter<ruckig::DynamicDOFs>& ruckig_input,
                                     const double overshoot_threshold)
{
  // For every timestep
  for (double time_from_start = OVERSHOOT_CHECK_PERIOD; time_from_start < ruckig_trajectory.get_duration();
       time_from_start += OVERSHOOT_CHECK_PERIOD)
  {
    std::vector<double> new_position(num_dof);
    std::vector<double> new_velocity(num_dof);
    std::vector<double> new_acceleration(num_dof);
    ruckig_trajectory.at_time(time_from_start, new_position, new_velocity, new_acceleration);
    // For every joint
    for (size_t joint = 0; joint < num_dof; ++joint)
    {
      // If the sign of the error changed and the threshold difference was exceeded
      double error = new_position[joint] - ruckig_input.target_position.at(joint);
      if (((error / (ruckig_input.current_position.at(joint) - ruckig_input.target_position.at(joint))) < 0.0) &&
          std::fabs(error) > overshoot_threshold)
      {
        return true;
      }
    }
  }
  return false;
}
}  // namespace trajectory_processing
