#pragma once

#include <Eigen/Geometry>
#include <moveit/robot_model/joint_model_group.h>
#include <stomp_moveit/stomp_moveit_task.hpp>

#include <stomp/utils.h>

namespace stomp_moveit
{
namespace filters
{
// \brief An empty placeholder filter that doesn't apply any updates to the trajectory.
const static FilterFn NoFilter = [](const Eigen::MatrixXd& /*values*/, Eigen::MatrixXd& /*filtered_values*/) {
  return true;
};

/**
 * Creates a filter function that applies Stomp's smoothing matrix for the whole trajectory.
 *
 * @param num_timesteps The number of trajectory waypoints configured for STOMP
 * @return The smoothing filter function to be used for the STOMP task
 */
FilterFn simple_smoothing_matrix(size_t num_timesteps)
{
  Eigen::MatrixXd smoothing_matrix;
  stomp::generateSmoothingMatrix(num_timesteps, 1.0, smoothing_matrix);
  return [=](const Eigen::MatrixXd& /*values*/, Eigen::MatrixXd& filtered_values) {
    for (int i = 0; i < filtered_values.rows(); ++i)
    {
      filtered_values.row(i).transpose() = smoothing_matrix * (filtered_values.row(i).transpose());
    }
    return true;
  };
}

/**
 * Creates a filter function that clips all waypoint positions using the joint bounds of a given JointModelGroup.
 *
 * @param group The JointModelGroup providing the joint limits
 * @return The filter function for enforcing joint limits
 */
FilterFn enforce_position_bounds(const moveit::core::JointModelGroup* group)
{
  return [=](const Eigen::MatrixXd& values, Eigen::MatrixXd& filtered_values) {
    filtered_values = values;
    const auto& joints = group->getActiveJointModels();
    for (size_t i = 0; i < joints.size(); ++i)
    {
      for (int j = 0; j < filtered_values.cols(); ++j)
      {
        joints.at(i)->enforcePositionBounds(&filtered_values.coeffRef(i, j));
      }
    }
    return true;
  };
}

/**
 * Helper function for applying multiple trajectory update filters in sequential order.
 *
 * @param filter_functions The ordered vector of filter functions to apply
 * @return The filter function applying all the configured filters in sequence
 */
FilterFn chain(const std::vector<FilterFn>& filter_functions)
{
  return [=](const Eigen::MatrixXd& values, Eigen::MatrixXd& filtered_values) {
    Eigen::MatrixXd values_in = values;
    for (const auto& filter_fn : filter_functions)
    {
      filter_fn(values_in, filtered_values);
      values_in = filtered_values;
    }
    return true;
  };
}
}  // namespace filters
}  // namespace stomp_moveit
