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
 * @brief Filter functions that can be passed to STOMP via a ComposableTask.
 */

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
static const FilterFn NO_FILTER = [](const Eigen::MatrixXd& /*values*/, Eigen::MatrixXd& /*filtered_values*/) {
  return true;
};

/**
 * Creates a filter function that applies Stomp's smoothing matrix for the whole trajectory.
 *
 * @param num_timesteps The number of trajectory waypoints configured for STOMP
 * @return The smoothing filter function to be used for the STOMP task
 */
FilterFn simpleSmoothingMatrix(size_t num_timesteps)
{
  // Generates a smoothing matrix and applies it for each joint dimension in filtered_values
  // The 'dt' value is a placeholder timestep duration that is used for approximating the second order derivative
  // (acceleration) using a finite difference matrix. The actual timestep duration will be computed by a planner
  // adapter after solving the STOMP trajectory.
  Eigen::MatrixXd smoothing_matrix;
  stomp::generateSmoothingMatrix(num_timesteps, 1.0 /* dt */, smoothing_matrix);
  return [=](const Eigen::MatrixXd& /*values*/, Eigen::MatrixXd& filtered_values) {
    for (auto row : filtered_values.rowwise())
    {
      row.transpose() = smoothing_matrix * row.transpose();
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
FilterFn enforcePositionBounds(const moveit::core::JointModelGroup* group)
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
