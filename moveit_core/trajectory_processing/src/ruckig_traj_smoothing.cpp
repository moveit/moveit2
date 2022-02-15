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

#include <algorithm>
#include <cmath>
#include <Eigen/Geometry>
#include <limits>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace trajectory_processing
{
namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_trajectory_processing.ruckig_traj_smoothing");
constexpr double DEFAULT_MAX_VELOCITY = 5;           // rad/s
constexpr double DEFAULT_MAX_ACCELERATION = 10;      // rad/s^2
constexpr double DEFAULT_MAX_JERK = 20;              // rad/s^3
constexpr double IDENTICAL_POSITION_EPSILON = 1e-3;  // rad
constexpr double MAX_DURATION_EXTENSION_FACTOR = 5.0;
constexpr double DURATION_EXTENSION_FRACTION = 1.1;
constexpr size_t MAX_WAYPOINT_ADJUSTMENT_ATTEMPTS = 100;
constexpr double BACKWARD_VELOCITY_THRESHOLD = 0.01;  // rad/s. Used in detection of backward motion
}  // namespace

bool RuckigSmoothing::applySmoothing(robot_trajectory::RobotTrajectory& trajectory,
                                     const double max_velocity_scaling_factor,
                                     const double max_acceleration_scaling_factor)
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
    RCLCPP_ERROR(LOGGER, "Trajectory does not have enough points to smooth with Ruckig");
    return false;
  }

  const size_t num_dof = group->getVariableCount();

  // This lib does not actually work properly when angles wrap around, so we need to unwind the path first
  trajectory.unwind();

  // Instantiate the smoother
  double timestep = trajectory.getAverageSegmentDuration();
  std::unique_ptr<ruckig::Ruckig<RUCKIG_DYNAMIC_DOF>> ruckig_ptr;
  ruckig_ptr = std::make_unique<ruckig::Ruckig<RUCKIG_DYNAMIC_DOF>>(num_dof, timestep);
  ruckig::InputParameter<RUCKIG_DYNAMIC_DOF> ruckig_input{ num_dof };
  ruckig::OutputParameter<RUCKIG_DYNAMIC_DOF> ruckig_output{ num_dof };

  // Initialize the smoother
  const std::vector<int>& idx = group->getVariableIndexList();
  initializeRuckigState(ruckig_input, ruckig_output, *trajectory.getFirstWayPointPtr(), num_dof, idx);

  // Kinematic limits (vel/accel/jerk)
  const std::vector<std::string>& vars = group->getVariableNames();
  const moveit::core::RobotModel& rmodel = group->getParentModel();
  for (size_t i = 0; i < num_dof; ++i)
  {
    // TODO(andyz): read this from the joint group if/when jerk limits are added to the JointModel
    ruckig_input.max_jerk.at(i) = DEFAULT_MAX_JERK;

    const moveit::core::VariableBounds& bounds = rmodel.getVariableBounds(vars.at(i));

    // This assumes min/max bounds are symmetric
    if (bounds.velocity_bounded_)
    {
      ruckig_input.max_velocity.at(i) = max_velocity_scaling_factor * bounds.max_velocity_;
    }
    else
    {
      ruckig_input.max_velocity.at(i) = max_velocity_scaling_factor * DEFAULT_MAX_VELOCITY;
    }
    if (bounds.acceleration_bounded_)
    {
      ruckig_input.max_acceleration.at(i) = max_acceleration_scaling_factor * bounds.max_acceleration_;
    }
    else
    {
      ruckig_input.max_acceleration.at(i) = max_acceleration_scaling_factor * DEFAULT_MAX_ACCELERATION;
    }
  }

  ruckig::Result ruckig_result;

  for (size_t waypoint_idx = 0; waypoint_idx < num_waypoints - 2 /* Don't modify the last waypoint */; ++waypoint_idx)
  {
    ruckig_result = ruckig::Result::Error;
    bool backward_motion_detected = false;
    moveit::core::RobotStatePtr next_waypoint = trajectory.getWayPointPtr(waypoint_idx + 1);

    getNextRuckigInput(ruckig_output, next_waypoint, num_dof, idx, ruckig_input);

    // Run Ruckig
    ruckig_result = ruckig_ptr->update(ruckig_input, ruckig_output);
    if (ruckig_result == ruckig::Result::Finished)
    {
      // Overwrite pos/vel/acc of waypoint_idx from previous Ruckig output
      setRobotStateFromRuckigOutput(ruckig_output, num_dof, idx, next_waypoint);

      // To the next waypoint_idx
      continue;
    }

    size_t num_iterations = 0;
    while ((backward_motion_detected || (ruckig_result != ruckig::Result::Finished)) &&
           (num_iterations < MAX_WAYPOINT_ADJUSTMENT_ATTEMPTS))
    {
      ++num_iterations;

      // If the requested velocity is too great, a joint can actually "move backward" to give itself more time to
      // accelerate to the target velocity. Stretch the duration of this waypoint to mitigate that.
      double duration_extension_factor = 1;
      if ((backward_motion_detected = checkForBackwardMotion(num_dof, ruckig_input, ruckig_output)) &&
          (duration_extension_factor < MAX_DURATION_EXTENSION_FACTOR))
      {
        // initializeRuckigState(ruckig_input, ruckig_output, *trajectory.getFirstWayPointPtr(), num_dof, idx);
        duration_extension_factor *= DURATION_EXTENSION_FRACTION;
        double new_timestep = DURATION_EXTENSION_FRACTION * trajectory.getWayPointDurationFromPrevious(waypoint_idx);
        trajectory.setWayPointDurationFromPrevious(waypoint_idx + 1, new_timestep);
        // Re-calculate waypoint accel since the timestep increased. Decrease velocity to help prevent overshoot.
        for (size_t joint = 0; joint < num_dof; ++joint)
        {
          ruckig_input.target_velocity.at(joint) *= 0.9;
          ruckig_input.target_acceleration.at(joint) =
              (ruckig_input.target_velocity.at(joint) - ruckig_input.current_velocity.at(joint)) / new_timestep;
        }

        ruckig_ptr = std::make_unique<ruckig::Ruckig<RUCKIG_DYNAMIC_DOF>>(num_dof, new_timestep);

        // Run Ruckig on this waypoint again
        ruckig_result = ruckig_ptr->update(ruckig_input, ruckig_output);
        // Reset Ruckig to the original timestep for the following waypoints
        ruckig_ptr = std::make_unique<ruckig::Ruckig<RUCKIG_DYNAMIC_DOF>>(num_dof, timestep);
        continue;
      }

      // If ruckig_result == Working, smoothing was working but it needs more time to reach the target.
      // Add another waypoint with the same target state.
      if (ruckig_result == ruckig::Result::Working)
      {
        // Insert a new waypoint after waypoint_idx+1
        trajectory.insertWayPoint(waypoint_idx + 2, *next_waypoint, timestep);
        ++num_waypoints;

        // Overwrite pos/vel/acc of waypoint_idx from previous Ruckig output
        setRobotStateFromRuckigOutput(ruckig_output, num_dof, idx, next_waypoint);

        // To the next waypoint_idx
        break;
      }

      // TODO(andyz): Drop consecutive waypoints with repeated positions.
      // They can cause "loopy motions".

      // If Ruckig failed completely, decrease target velocity
      decreaseTargetStateVelocity(num_dof, timestep, ruckig_input);

      // Run Ruckig
      ruckig_result = ruckig_ptr->update(ruckig_input, ruckig_output);
    }
  }

  if (ruckig_result != ruckig::Result::Finished)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Ruckig trajectory smoothing failed. Ruckig error: " << ruckig_result);
    return false;
  }

  return true;
}

void RuckigSmoothing::setRobotStateFromRuckigOutput(const ruckig::OutputParameter<RUCKIG_DYNAMIC_DOF> ruckig_output,
                                                    const size_t num_dof, const std::vector<int>& joint_idx,
                                                    moveit::core::RobotStatePtr state)
{
  for (size_t joint = 0; joint < num_dof; ++joint)
  {
    state->setVariablePosition(joint_idx.at(joint), ruckig_output.new_position.at(joint));
    state->setVariableVelocity(joint_idx.at(joint), ruckig_output.new_velocity.at(joint));
    state->setVariableAcceleration(joint_idx.at(joint), ruckig_output.new_acceleration.at(joint));
  }
  state->update();
}

void RuckigSmoothing::decreaseTargetStateVelocity(const size_t num_dof, const double timestep,
                                                  ruckig::InputParameter<RUCKIG_DYNAMIC_DOF>& ruckig_input)
{
  for (size_t joint = 0; joint < num_dof; ++joint)
  {
    ruckig_input.target_velocity.at(joint) *= 0.9;
    // Propagate the change in velocity to acceleration, too.
    // We don't change the position to ensure the exact target position is achieved.
    ruckig_input.target_acceleration.at(joint) =
        (ruckig_input.target_velocity.at(joint) - ruckig_input.current_velocity.at(joint)) / timestep;
  }
}

void RuckigSmoothing::initializeRuckigState(ruckig::InputParameter<RUCKIG_DYNAMIC_DOF>& ruckig_input,
                                            ruckig::OutputParameter<RUCKIG_DYNAMIC_DOF>& ruckig_output,
                                            const moveit::core::RobotState& first_waypoint, size_t num_dof,
                                            const std::vector<int>& idx)
{
  std::vector<double> current_positions_vector(num_dof);
  std::vector<double> current_velocities_vector(num_dof);
  std::vector<double> current_accelerations_vector(num_dof);

  for (size_t i = 0; i < num_dof; ++i)
  {
    current_positions_vector.at(i) = first_waypoint.getVariablePosition(idx.at(i));
    current_velocities_vector.at(i) = first_waypoint.getVariableVelocity(idx.at(i));
    current_accelerations_vector.at(i) = first_waypoint.getVariableAcceleration(idx.at(i));
  }
  std::copy_n(current_positions_vector.begin(), num_dof, ruckig_input.current_position.begin());
  std::copy_n(current_velocities_vector.begin(), num_dof, ruckig_input.current_velocity.begin());
  std::copy_n(current_accelerations_vector.begin(), num_dof, ruckig_input.current_acceleration.begin());
  // Initialize output data struct
  ruckig_output.new_position = ruckig_input.current_position;
  ruckig_output.new_velocity = ruckig_input.current_velocity;
  ruckig_output.new_acceleration = ruckig_input.current_acceleration;
}

bool RuckigSmoothing::checkForIdenticalWaypoints(const moveit::core::RobotState& prev_waypoint,
                                                 const moveit::core::RobotState& next_waypoint,
                                                 const moveit::core::JointModelGroup* joint_group)
{
  double magnitude_position_difference = prev_waypoint.distance(next_waypoint, joint_group);

  return (magnitude_position_difference <= IDENTICAL_POSITION_EPSILON);
}

double RuckigSmoothing::getTargetVelocityMagnitude(const ruckig::InputParameter<RUCKIG_DYNAMIC_DOF>& ruckig_input,
                                                   size_t num_dof)
{
  double vel_magnitude = 0;
  for (size_t joint = 0; joint < num_dof; ++joint)
  {
    vel_magnitude += ruckig_input.target_velocity.at(joint) * ruckig_input.target_velocity.at(joint);
  }
  return sqrt(vel_magnitude);
}

bool RuckigSmoothing::checkForBackwardMotion(const size_t num_dof,
                                             const ruckig::InputParameter<RUCKIG_DYNAMIC_DOF>& ruckig_input,
                                             const ruckig::OutputParameter<RUCKIG_DYNAMIC_DOF>& ruckig_output)
{
  // Check for backward motion of any joint
  for (size_t joint = 0; joint < num_dof; ++joint)
  {
    // Check for opposite velocity sign and significant magnitude
    if (((ruckig_output.new_velocity.at(joint) / ruckig_input.target_velocity.at(joint)) < 0) &&
        (fabs(ruckig_output.new_velocity.at(joint)) > BACKWARD_VELOCITY_THRESHOLD))
    {
      return true;
    }
  }
  return false;
}

void RuckigSmoothing::getNextRuckigInput(const ruckig::OutputParameter<RUCKIG_DYNAMIC_DOF>& ruckig_output,
                                         const moveit::core::RobotStatePtr& next_waypoint, size_t num_dof,
                                         const std::vector<int>& idx,
                                         ruckig::InputParameter<RUCKIG_DYNAMIC_DOF>& ruckig_input)
{
  // TODO(andyz): https://github.com/ros-planning/moveit2/issues/766
  // ruckig_output.pass_to_input(ruckig_input);

  for (size_t joint = 0; joint < num_dof; ++joint)
  {
    // Feed output from the previous timestep back as input
    ruckig_input.current_position.at(joint) = ruckig_output.new_position.at(joint);
    ruckig_input.current_velocity.at(joint) = ruckig_output.new_velocity.at(joint);
    ruckig_input.current_acceleration.at(joint) = ruckig_output.new_acceleration.at(joint);

    // Target state is the next waypoint
    ruckig_input.target_position.at(joint) = next_waypoint->getVariablePosition(idx.at(joint));
    ruckig_input.target_velocity.at(joint) = next_waypoint->getVariableVelocity(idx.at(joint));
    ruckig_input.target_acceleration.at(joint) = next_waypoint->getVariableAcceleration(idx.at(joint));
  }
}
}  // namespace trajectory_processing
