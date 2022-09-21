#pragma once

#include <Eigen/Geometry>
#include <stomp_moveit/stomp_moveit_task.hpp>

#include <stomp/utils.h>

namespace stomp_moveit
{
namespace filters
{
const static FilterFn NoFilter = [](const Eigen::MatrixXd& values, Eigen::MatrixXd& filtered_values) { return true; };

FilterFn simple_smoothing_matrix(size_t num_timesteps)
{
  Eigen::MatrixXd smoothing_matrix;
  stomp::generateSmoothingMatrix(num_timesteps, 1.0, smoothing_matrix);
  return [=](const Eigen::MatrixXd& values, Eigen::MatrixXd& filtered_values) {
    for (int i = 0; i < filtered_values.rows(); ++i)
    {
      filtered_values.row(i).transpose() = smoothing_matrix * (filtered_values.row(i).transpose());
    }
    return true;
  };
}
}  // namespace filters
}  // namespace stomp_moveit
