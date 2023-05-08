#pragma once

#include <stomp_moveit/stomp_moveit_task.hpp>  // Function definitions
#include <stomp_moveit/math/multivariate_gaussian.hpp>
#include <Eigen/Geometry>

namespace stomp_moveit
{
namespace noise
{
/**
 * Creates a noise generator function that applies noise sampled from a normal distribution.
 * The noise is applied over a matrix of size (num_timesteps, stddev.size()) which corresponds
 * to the matrix representation of a robot trajectory.
 *
 * @param num_timesteps the waypoint count of the trajectory
 * @param stddev the standard deviation for each variable dimension (number of joints)
 */
NoiseGeneratorFn get_normal_distribution_generator(size_t num_timesteps, std::vector<double> stddev)
{
  // Five-point stencil constants
  static const std::vector<double> ACC_MATRIX_DIAGONAL_VALUES = { -1.0 / 12.0, 16.0 / 12.0, -30.0 / 12.0, 16.0 / 12.0,
                                                                  -1.0 / 12.0 };
  static const std::vector<int> ACC_MATRIX_DIAGONAL_INDICES = { -2, -1, 0, 1, 2 };

  auto fill_diagonal = [](Eigen::MatrixXd& m, double coeff, int diag_index) {
    std::size_t size = m.rows() - std::abs(diag_index);
    m.diagonal(diag_index) = Eigen::VectorXd::Constant(size, coeff);
  };

  // creating finite difference acceleration matrix
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_timesteps, num_timesteps);
  for (auto i = 0u; i < ACC_MATRIX_DIAGONAL_INDICES.size(); i++)
  {
    fill_diagonal(A, ACC_MATRIX_DIAGONAL_VALUES[i], ACC_MATRIX_DIAGONAL_INDICES[i]);
  }

  // create and scale covariance matrix
  Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(num_timesteps, num_timesteps);
  covariance = A.transpose() * A;
  covariance = covariance.fullPivLu().inverse();
  covariance /= covariance.array().abs().matrix().maxCoeff();

  // create random generators
  std::vector<math::MultivariateGaussianPtr> rand_generators(stddev.size());
  for (auto& r : rand_generators)
  {
    r = std::make_shared<math::MultivariateGaussian>(Eigen::VectorXd::Zero(num_timesteps), covariance);
  }

  auto raw_noise = std::make_shared<Eigen::VectorXd>(num_timesteps);
  NoiseGeneratorFn noise_generator_fn = [=](const Eigen::MatrixXd& values, Eigen::MatrixXd& noisy_values,
                                            Eigen::MatrixXd& noise) {
    for (int i = 0; i < values.rows(); ++i)
    {
      rand_generators[i]->sample(*raw_noise);
      raw_noise->head(1).setZero();
      raw_noise->tail(1).setZero();  // zeroing out the start and end noise values
      noise.row(i).transpose() = stddev[i] * (*raw_noise);
      noisy_values.row(i) = values.row(i) + noise.row(i);
    }
    return true;
  };
  return noise_generator_fn;
}
}  // namespace noise
}  // namespace stomp_moveit
