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

/** @file
 * @author Henning Kayser
 * @brief MoveIt-based cost functions that can be passed to STOMP via a ComposableTask.
 */

#pragma once

#include <Eigen/Geometry>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>

#include <stomp_moveit/stomp_moveit_task.hpp>
#include <stomp_moveit/conversion_functions.hpp>

namespace stomp_moveit
{
// Decides if the given state position vector is valid or not - example use cases are collision or constraint checking
using StateValidatorFn = std::function<bool(const Eigen::VectorXd& state_positions)>;

namespace costs
{

// Interpolation step size for collision checking (joint space, L2 norm)
constexpr double COL_CHECK_DISTANCE = 0.05;
constexpr double CONSTRAINT_CHECK_DISTANCE = 0.1;

/**
 * Creates a cost function from a binary robot state validation function.
 * This is used for computing smooth cost profiles for binary state conditions like collision checks and constraints.
 * The validator function is applied for all states in the validated path while also considering interpolated states.
 * If a waypoint or an interpolated state is invalid, a local penalty is being applied to the path.
 * Penalty costs are being smoothed out using a Gaussian so that valid neighboring states (near collisions) are
 * optimized as well.
 *
 * @param state_validator_fn      The validator function that tests for binary conditions
 * @param interpolation_step_size The L2 norm distance step used for interpolation
 * @param penalty                 The penalty cost value applied to invalid states
 *
 * @return                        Cost function that computes smooth costs for binary validity conditions
 */
CostFn get_cost_function_from_state_validator(const StateValidatorFn& state_validator_fn,
                                              double interpolation_step_size, double penalty)
{
  CostFn cost_fn = [=](const Eigen::MatrixXd& values, Eigen::VectorXd& costs, bool& validity) {
    costs.setZero(values.cols());

    validity = true;
    std::vector<std::pair<long, long>> invalid_windows;
    bool in_invalid_window = false;

    // Iterate over sample waypoint pairs and check for validity in each segment.
    // If an invalid state is found, weighted penalty costs are applied to both waypoints.
    // Subsequent invalid states are assumed to have the same cause, so we are keeping track
    // of "invalid windows" which are used for smoothing out the costs per violation cause
    // with a gaussian, penalizing neighboring valid states as well.
    for (int timestep = 0; timestep < values.cols() - 1; ++timestep)
    {
      Eigen::VectorXd current = values.col(timestep);
      Eigen::VectorXd next = values.col(timestep + 1);
      const double segment_distance = (next - current).norm();
      double interpolation_fraction = 0.0;
      const double interpolation_step = std::min(0.5, interpolation_step_size / segment_distance);
      bool found_invalid_state = false;
      while (!found_invalid_state && interpolation_fraction < 1.0)
      {
        Eigen::VectorXd sample_vec = (1 - interpolation_fraction) * current + interpolation_fraction * next;

        found_invalid_state = !state_validator_fn(sample_vec);
        interpolation_fraction += interpolation_step;
      }

      if (found_invalid_state)
      {
        // Apply weighted penalties -> This trajectory is definitely invalid
        costs(timestep) = (1 - interpolation_fraction) * penalty;
        costs(timestep + 1) = interpolation_fraction * penalty;
        validity = false;

        // Open new invalid window when this is the first detected invalid state in a group
        if (!in_invalid_window)
        {
          invalid_windows.emplace_back(timestep, values.cols());
          in_invalid_window = true;
        }
      }
      else
      {
        // Close current invalid window if the current state is valid
        if (in_invalid_window)
        {
          invalid_windows.back().second = timestep - 1;
          in_invalid_window = false;
        }
      }
    }

    // Smooth out cost of invalid segments using a gaussian
    // The standard deviation is picked so that neighboring states
    // before and after the violation are penalized as well.
    for (const auto& [start, end] : invalid_windows)
    {
      const double window_size = static_cast<double>(end - start) + 1;
      const double sigma = window_size / 5.0;
      const double mu = 0.5 * (start + end);
      // Iterate over waypoints in the range of +/-sigma (neighborhood)
      // and add a weighted and continuous cost value for each waypoint
      // based on a Gaussian distribution.
      for (auto j = std::max(0l, start - static_cast<long>(sigma));
           j <= std::min(values.cols() - 1, end + static_cast<long>(sigma)); ++j)
      {
        costs(j) +=
            std::exp(-std::pow(j - mu, 2) / (2 * std::pow(sigma, 2))) / (sigma * std::sqrt(2 * mu)) * window_size;
      }
    }

    return true;
  };

  return cost_fn;
}

/**
 * Creates a cost function for binary collisions of group states in the planning scene.
 * This function uses a StateValidatorFn for computing smooth penalty costs from binary
 * collision checks using get_cost_function_from_state_validator().
 *
 * @param planning_scene    The planning scene instance to use for collision checking
 * @param group             The group to use for computing link transforms from joint positions
 * @param collision_penalty The penalty cost value applied to colliding states
 *
 * @return                  Cost function that computes smooth costs for colliding path segments
 */
CostFn get_collision_cost_function(const std::shared_ptr<const planning_scene::PlanningScene>& planning_scene,
                                   const moveit::core::JointModelGroup* group, double collision_penalty)
{
  const auto& joints = group ? group->getActiveJointModels() : planning_scene->getRobotModel()->getActiveJointModels();
  const auto& group_name = group ? group->getName() : "";

  StateValidatorFn collision_validator_fn = [=](const Eigen::VectorXd& positions) {
    static moveit::core::RobotState state(planning_scene->getCurrentState());

    // Update robot state values
    set_joint_positions(positions, joints, state);
    state.update();

    return !planning_scene->isStateColliding(state, group_name);
  };

  return get_cost_function_from_state_validator(collision_validator_fn, COL_CHECK_DISTANCE, collision_penalty);
}

/**
 * Creates a cost function for binary constraint checks applied to group states.
 * This function uses a StateValidatorFn for computing smooth penalty costs from binary
 * constraint checks using get_cost_function_from_state_validator().
 *
 * @param planning_scene      The planning scene instance to use for computing transforms
 * @param group               The group to use for computing link transforms from joint positions
 * @param constraints_msg     The constraints used for validating group states
 * @param constraints_penalty The penalty cost value applied to invalid states
 *
 * @return                    Cost function that computes smooth costs for invalid path segments
 */
CostFn get_constraints_cost_function(const std::shared_ptr<const planning_scene::PlanningScene>& planning_scene,
                                     const moveit::core::JointModelGroup* group,
                                     const moveit_msgs::msg::Constraints& constraints_msg, double constraints_penalty)
{
  const auto& joints = group ? group->getActiveJointModels() : planning_scene->getRobotModel()->getActiveJointModels();

  kinematic_constraints::KinematicConstraintSet constraints(planning_scene->getRobotModel());
  constraints.add(constraints_msg, planning_scene->getTransforms());

  StateValidatorFn constraints_validator_fn = [=](const Eigen::VectorXd& positions) {
    static moveit::core::RobotState state(planning_scene->getCurrentState());

    // Update robot state values
    set_joint_positions(positions, joints, state);
    state.update();

    // NOTE: the returned ConstraintEvaluationResult also provides a `double distance` which might be used as an
    // actual cost gradient instead of the binary state penalty
    return constraints.decide(state).satisfied;
  };

  return get_cost_function_from_state_validator(constraints_validator_fn, CONSTRAINT_CHECK_DISTANCE,
                                                constraints_penalty);
}

/**
 * Creates a cost function that computes the summed waypoint penalites over a vector of cost functions.
 *
 * @param cost_functions A vector of cost functions
 *
 * @return               Cost function that computes the summed costs for each waypoint
 */
CostFn sum(const std::vector<CostFn>& cost_functions)
{
  return [=](const Eigen::MatrixXd& values, Eigen::VectorXd& overall_costs, bool& overall_validity) {
    overall_validity = true;
    overall_costs.setZero(values.cols());

    auto costs = overall_costs;
    for (const auto& cost_fn : cost_functions)
    {
      bool valid = true;
      cost_fn(values, costs, valid);

      // Sum results
      overall_validity = overall_validity && valid;
      overall_costs += costs;
    }
    return true;
  };
}
}  // namespace costs
}  // namespace stomp_moveit
