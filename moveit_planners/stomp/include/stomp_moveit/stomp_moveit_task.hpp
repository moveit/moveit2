#pragma once

#include <stomp/task.h>

namespace stomp_moveit
{
using NoiseGeneratorFn =
    std::function<bool(const Eigen::MatrixXd& values, Eigen::MatrixXd& noisy_values, Eigen::MatrixXd& noise)>;
using CostFn = std::function<bool(const Eigen::MatrixXd& values, Eigen::VectorXd& costs, bool& validity)>;
using FilterFn = std::function<bool(const Eigen::MatrixXd& values, Eigen::MatrixXd& filtered_values)>;
using PostIterationFn = std::function<void(int iteration_number, double cost, const Eigen::MatrixXd& values)>;
using DoneFn =
    std::function<void(bool success, int total_iterations, double final_cost, const Eigen::MatrixXd& values)>;

class ComposableTask final : public stomp::Task
{
public:
  ComposableTask(NoiseGeneratorFn noise_generator_fn, CostFn cost_fn, FilterFn filter_fn,
                 PostIterationFn post_iteration_fn, DoneFn done_fn)
    : noise_generator_fn_(noise_generator_fn)
    , cost_fn_(cost_fn)
    , filter_fn_(filter_fn)
    , post_iteration_fn_(post_iteration_fn)
    , done_fn_(done_fn)
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
