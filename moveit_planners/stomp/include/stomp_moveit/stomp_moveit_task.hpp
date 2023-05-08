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
 * @brief A STOMP task definition that allows injecting custom functions for planning.
 *
 * STOMP's task interface can be used for customizing the planning objective, in particular the code used for sampling
 * new random trajectories and for computing costs and validity of waypoint candidates. In order to allow building
 * generic planning tasks at runtime, the ComposableTask class enables combining generic function types for planning:
 *
 * - NoiseGeneratorFn: computes randomized paths
 * - CostFn: computes waypoint costs and path validity
 * - FilterFn: applies a filter to path waypoints
 * - PostIterationFn: reports on planning progress at each iteration (see STOMP documentation)
 * - DoneFn: reports on planning result when STOMP run terminates
 *
 * Each of these functions use Eigen types for representing path and waypoints.
 * The Eigen::MatrixXd 'values' refer to full path candidates where rows are the joint dimensions
 * and columns are the waypoints. Accordingly, Eigen::VectorXd is used for representing cost values,
 * one value for each waypoint.
 */

#pragma once

#include <stomp/task.h>

namespace stomp_moveit
{
// @brief A function that computes randomized noisy values for input trajectory values.
using NoiseGeneratorFn =
    std::function<bool(const Eigen::MatrixXd& values, Eigen::MatrixXd& noisy_values, Eigen::MatrixXd& noise)>;
// @brief A function that computes cost values and a validity result for input trajectory values
using CostFn = std::function<bool(const Eigen::MatrixXd& values, Eigen::VectorXd& costs, bool& validity)>;
// @brief A function that applies a filter (smoothing, clamping, ...) of an input trajectory
using FilterFn = std::function<bool(const Eigen::MatrixXd& values, Eigen::MatrixXd& filtered_values)>;
// @brief A function that is called with every iteration to report on the current trajectory candidate and cost
using PostIterationFn = std::function<void(int iteration_number, double cost, const Eigen::MatrixXd& values)>;
// @brief A function that is called when STOMP terminates, providing success flag, cost and solution trajectory
using DoneFn =
    std::function<void(bool success, int total_iterations, double final_cost, const Eigen::MatrixXd& values)>;

// @brief A STOMP task that allows injecting generic callbacks for a STOMP planning instance
//
// The ComposableTask stores custom functions for the most important callback types in STOMP and applies them during
// a motion planning run. This class is used for injecting MoveIt concepts and other custom features into STOMP.
class ComposableTask final : public stomp::Task
{
public:
  ComposableTask(NoiseGeneratorFn noise_generator_fn, CostFn cost_fn, FilterFn filter_fn,
                 PostIterationFn post_iteration_fn, DoneFn done_fn)
    : noise_generator_fn_(std::move(noise_generator_fn))
    , cost_fn_(std::move(cost_fn))
    , filter_fn_(std::move(filter_fn))
    , post_iteration_fn_(std::move(post_iteration_fn))
    , done_fn_(std::move(done_fn))
  {
  }

  ~ComposableTask() = default;

  /**
   * @brief Generates a noisy trajectory from the parameters.
   * @param parameters        A matrix [num_dimensions][num_parameters] of the current optimized parameters
   * @param start_timestep    The start index into the 'parameters' array, usually 0.
   * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param rollout_number    The index of the noisy trajectory.
   * @param parameters_noise  The parameters + noise
   * @param noise             The noise applied to the parameters
   * @return True if cost were properly computed, otherwise false
   */
  bool generateNoisyParameters(const Eigen::MatrixXd& parameters, std::size_t /*start_timestep*/,
                               std::size_t /*num_timesteps*/, int /*iteration_number*/, int /*rollout_number*/,
                               Eigen::MatrixXd& parameters_noise, Eigen::MatrixXd& noise) override
  {
    return noise_generator_fn_(parameters, parameters_noise, noise);
  }

  /**
   * @brief computes the state costs as a function of the distance from the bias parameters
   * @param parameters        A matrix [num_dimensions][num_parameters] of the policy parameters to execute
   * @param start_timestep    The start index into the 'parameters' array, usually 0.
   * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param costs             A vector containing the state costs per timestep.
   * @param validity          Whether or not the trajectory is valid
   * @return True if cost were properly computed, otherwise false
   */
  bool computeCosts(const Eigen::MatrixXd& parameters, std::size_t /*start_timestep*/, std::size_t /*num_timesteps*/,
                    int /*iteration_number*/, Eigen::VectorXd& costs, bool& validity) override
  {
    return cost_fn_(parameters, costs, validity);
  }

  /**
   * @brief computes the state costs as a function of the distance from the bias parameters
   * @param parameters        A matrix [num_dimensions][num_parameters] of the policy parameters to execute
   * @param start_timestep    The start index into the 'parameters' array, usually 0.
   * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param rollout_number    The index of the noisy trajectory.
   * @param costs             A vector containing the state costs per timestep.
   * @param validity          Whether or not the trajectory is valid
   * @return True if cost were properly computed, otherwise false
   */
  bool computeNoisyCosts(const Eigen::MatrixXd& parameters, std::size_t /*start_timestep*/,
                         std::size_t /*num_timesteps*/, int /*iteration_number*/, int /*rollout_number*/,
                         Eigen::VectorXd& costs, bool& validity) override
  {
    return cost_fn_(parameters, costs, validity);
  }

  /**
   * @brief Filters the given parameters which is applied after the update. It could be used for clipping of joint
   * limits or projecting into the null space of the Jacobian.
   *
   * @param start_timestep    The start index into the 'parameters' array, usually 0.
   * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param parameters        The optimized parameters
   * @param updates           The updates to the parameters
   * @return                  True if successful, otherwise false
   */
  bool filterParameterUpdates(std::size_t /*start_timestep*/, std::size_t /*num_timesteps*/, int /*iteration_number*/,
                              const Eigen::MatrixXd& parameters, Eigen::MatrixXd& updates) override
  {
    return filter_fn_(parameters, updates);
  }

  /**
   * @brief Called by STOMP at the end of each iteration.
   * @param start_timestep    The start index into the 'parameters' array, usually 0.
   * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param cost              The cost value for the current parameters.
   * @param parameters        The value of the parameters at the end of the current iteration [num_dimensions x
   * num_timesteps].
   */
  void postIteration(std::size_t /*start_timestep*/, std::size_t /*num_timesteps*/, int iteration_number, double cost,
                     const Eigen::MatrixXd& parameters) override
  {
    post_iteration_fn_(iteration_number, cost, parameters);
  }

  /**
   * @brief Called by Stomp at the end of the optimization process
   *
   * @param success           Whether the optimization succeeded
   * @param total_iterations  Number of iterations used
   * @param final_cost        The cost value after optimizing.
   * @param parameters        The parameters generated at the end of the optimization [num_dimensions x num_timesteps]
   */
  void done(bool success, int total_iterations, double final_cost, const Eigen::MatrixXd& parameters) override
  {
    done_fn_(success, total_iterations, final_cost, parameters);
  }

private:
  NoiseGeneratorFn noise_generator_fn_;
  CostFn cost_fn_;
  FilterFn filter_fn_;
  PostIterationFn post_iteration_fn_;
  DoneFn done_fn_;
};
}  // namespace stomp_moveit
