/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, PickNik Inc.
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

#include <moveit/online_signal_smoothing/acceleration_filter.h>
#include <rclcpp/logging.hpp>

// Disable -Wold-style-cast because all _THROTTLE macros trigger this
#pragma GCC diagnostic ignored "-Wold-style-cast"

namespace online_signal_smoothing
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.core.acceleration_limited_plugin");
}

// The threshold below which any velocity or position difference is considered zero (rad and rad/s).
constexpr double COMMAND_DIFFERENCE_THRESHOLD = 1E-4;
// The scaling parameter alpha between the current point and commanded point must be less than 1.0
constexpr double ALPHA_UPPER_BOUND = 1.0;
// The scaling parameter alpha must also be greater than 0.0
constexpr double ALPHA_LOWER_BOUND = 0.0;

/** \brief Wrapper struct to make memory management easier for using osqp's C sparse_matrix types */
struct CSCWrapper
{
  /// row indices, size nzmax starting from 0
  std::vector<c_int> row_indices;
  /// column pointers (size n+1); col indices (size nzmax)
  std::vector<c_int> column_pointers;
  /// holds the non-zero values in Compressed Sparse Column (CSC) form
  std::vector<double> elements;
  /// osqp C sparse_matrix type
  csc csc_sparse_matrix;

  CSCWrapper(Eigen::SparseMatrix<double>& M)
  {
    M.makeCompressed();

    csc_sparse_matrix.n = M.cols();
    csc_sparse_matrix.m = M.rows();
    row_indices.assign(M.innerSize(), 0);
    csc_sparse_matrix.i = row_indices.data();
    column_pointers.assign(M.outerSize() + 1, 0);
    csc_sparse_matrix.p = column_pointers.data();
    csc_sparse_matrix.nzmax = M.nonZeros();
    csc_sparse_matrix.nz = -1;
    elements.assign(M.nonZeros(), 0.0);
    csc_sparse_matrix.x = elements.data();

    update(M);
  }

  /// Update the the data point to by sparse_matrix without reallocating memory
  void update(Eigen::SparseMatrix<double>& M)
  {
    for (size_t ind = 0; ind < row_indices.size(); ++ind)
    {
      row_indices[ind] = M.innerIndexPtr()[ind];
    }

    for (size_t ind = 0; ind < column_pointers.size(); ++ind)
    {
      column_pointers[ind] = M.outerIndexPtr()[ind];
    }
    for (size_t ind = 0; ind < elements.size(); ++ind)
    {
      elements[ind] = M.data().at(ind);
    }
  }
};

MOVEIT_STRUCT_FORWARD(OSQPDataWrapper);

/** \brief Wrapper struct to make memory management easier for using osqp's C API */
struct OSQPDataWrapper
{
  OSQPDataWrapper(Eigen::SparseMatrix<double>& objective_sparse, Eigen::SparseMatrix<double>& constraints_sparse)
    : P{ objective_sparse }, A{ constraints_sparse }
  {
    data.n = objective_sparse.rows();
    data.m = constraints_sparse.rows();
    data.P = &P.csc_sparse_matrix;
    q = Eigen::VectorXd::Zero(objective_sparse.rows());
    data.q = q.data();
    data.A = &A.csc_sparse_matrix;
    l = Eigen::VectorXd::Zero(constraints_sparse.rows());
    data.l = l.data();
    u = Eigen::VectorXd::Zero(constraints_sparse.rows());
    data.u = u.data();
  }

  /// Update the constraint matrix A without reallocating memory
  void updateA(OSQPWorkspace* work, Eigen::SparseMatrix<double>& constraints_sparse)
  {
    constraints_sparse.makeCompressed();
    A.update(constraints_sparse);
    osqp_update_A(work, A.elements.data(), OSQP_NULL, A.elements.size());
  }

  CSCWrapper P;
  CSCWrapper A;
  Eigen::VectorXd q;
  Eigen::VectorXd l;
  Eigen::VectorXd u;
  OSQPData data{};
};

bool AccelerationLimitedPlugin::initialize(rclcpp::Node::SharedPtr node, moveit::core::RobotModelConstPtr robot_model,
                                           size_t num_joints)
{
  // copy inputs into member variables
  node_ = node;
  num_joints_ = num_joints;
  robot_model_ = robot_model;
  cur_acceleration_ = Eigen::VectorXd::Zero(num_joints);

  // get node parameters and store in member variables
  auto param_listener = online_signal_smoothing::ParamListener(node_);
  params_ = param_listener.get_params();

  // get robot acceleration limits and store in member variables
  auto joint_model_group = robot_model_->getJointModelGroup(params_.planning_group_name);
  auto joint_bounds = joint_model_group->getActiveJointModelsBounds();
  min_acceleration_limits_ = Eigen::VectorXd::Zero(num_joints);
  max_acceleration_limits_ = Eigen::VectorXd::Zero(num_joints);
  size_t ind = 0;
  for (const auto& joint_bound : joint_bounds)
  {
    for (const auto& variable_bound : *joint_bound)
    {
      if (variable_bound.acceleration_bounded_)
      {
        min_acceleration_limits_[ind] = variable_bound.min_acceleration_;
        max_acceleration_limits_[ind] = variable_bound.max_acceleration_;
      }
      else
      {
        RCLCPP_ERROR(getLogger(), "The robot must have acceleration joint limits specified for all joints to "
                                  "use AccelerationLimitedPlugin.");
        return false;
      }
    }
    ind++;
  }

  // setup osqp optimization problem
  Eigen::SparseMatrix<double> objective_sparse(1, 1);
  objective_sparse.insert(0, 0) = 1.0;
  size_t num_constraints = num_joints + 1;
  constraints_sparse_ = Eigen::SparseMatrix<double>(num_constraints, 1);
  for (size_t i = 0; i < num_constraints - 1; ++i)
  {
    constraints_sparse_.insert(i, 0) = 0;
  }
  constraints_sparse_.insert(num_constraints - 1, 0) = 0;
  osqp_set_default_settings(&osqp_settings_);
  osqp_settings_.warm_start = 0;
  osqp_settings_.verbose = 0;
  osqp_data_ = std::make_shared<OSQPDataWrapper>(objective_sparse, constraints_sparse_);
  osqp_data_->q[0] = 0;

  if (osqp_setup(&osqp_workspace_, &osqp_data_->data, &osqp_settings_) != 0)
  {
    osqp_settings_.verbose = 1;
    // call setup again with verbose enables to trigger error message printing
    osqp_setup(&osqp_workspace_, &osqp_data_->data, &osqp_settings_);
    RCLCPP_ERROR(getLogger(), "Failed to initialize osqp problem.");
    return false;
  }

  return true;
}

double jointLimitAccelerationScalingFactor(const Eigen::VectorXd& accelerations,
                                           const moveit::core::JointBoundsVector& joint_bounds)
{
  double min_scaling_factor = 1.0;

  // Now get the scaling factor from joint limits.
  size_t idx = 0;
  for (const auto& joint_bound : joint_bounds)
  {
    for (const auto& variable_bound : *joint_bound)
    {
      const auto& target_accel = accelerations(idx);
      if (variable_bound.acceleration_bounded_ && target_accel != 0.0)
      {
        // Find the ratio of clamped acceleration to original acceleration
        const auto bounded_vel =
            std::clamp(target_accel, variable_bound.min_acceleration_, variable_bound.max_acceleration_);
        double joint_scaling_factor = bounded_vel / target_accel;
        min_scaling_factor = std::min(min_scaling_factor, joint_scaling_factor);
      }
      ++idx;
    }
  }

  return min_scaling_factor;
}

inline bool updateData(const OSQPDataWrapperPtr& data, OSQPWorkspace* work,
                       Eigen::SparseMatrix<double>& constraints_sparse, const Eigen::VectorXd& lower_bound,
                       const Eigen::VectorXd& upper_bound)
{
  data->updateA(work, constraints_sparse);
  size_t num_constraints = constraints_sparse.rows();
  data->u.block(0, 0, num_constraints - 1, 1) = upper_bound;
  data->l.block(0, 0, num_constraints - 1, 1) = lower_bound;
  data->u[num_constraints - 1] = ALPHA_UPPER_BOUND;
  data->l[num_constraints - 1] = ALPHA_LOWER_BOUND;
  return 0 == osqp_update_bounds(work, data->l.data(), data->u.data());
}

bool AccelerationLimitedPlugin::doSmoothing(Eigen::VectorXd& positions, Eigen::VectorXd& velocities,
                                            Eigen::VectorXd& /* unused */)
{
  const size_t num_positions = velocities.size();
  if (num_positions != num_joints_)
  {
    RCLCPP_ERROR_THROTTLE(
        getLogger(), *node_->get_clock(), 1000,
        "The length of the joint positions parameter is not equal to the number of joints, expected %zu got %zu.",
        num_joints_, num_positions);
    return false;
  }
  else if (last_positions_.size() != positions.size())
  {
    RCLCPP_ERROR_THROTTLE(getLogger(), *node_->get_clock(), 1000,
                          "The length of the last joint positions not equal to the current, expected %zu got %zu. Make "
                          "sure the reset was called.",
                          last_positions_.size(), positions.size());
    return false;
  }

  // formulate a quadratic program to find the best new reference point subject to the robot's acceleration limits
  // p_c: robot's current position
  // v_c: robot's current velocity
  // p_t: robot's target position
  // acc: acceleration to be applied
  // p_n: next position
  // dt: time step
  // p_n_hat: parameterize solution to be along the line from p_c to p_t
  // p_n_hat = p_t*alpha + p_c*(1-alpha)
  // define constraints
  // p_c + v_c*dt + acc_min*dt^2 < p_n_hat < p_c + v_c*dt + acc_max*dt^2
  // p_c + v_c*dt -p_t + acc_min*dt^2 < (p_c-p_t)alpha < p_c + v_c*dt -p_t + acc_max*dt^2
  // 0 < alpha < 1
  // define optimization
  // opt ||alpha||
  // s.t. constraints
  // p_n = p_t*alpha + p_c*(1-alpha)

  double& update_period = params_.update_period;
  size_t num_constraints = num_joints_ + 1;
  positions_offset_ = last_positions_ - positions;
  velocities_offset_ = last_velocities_ - velocities;
  for (size_t i = 0; i < num_constraints - 1; ++i)
  {
    constraints_sparse_.coeffRef(i, 0) = positions_offset_[i];
  }
  constraints_sparse_.coeffRef(num_constraints - 1, 0) = 1;
  Eigen::VectorXd vel_point = last_positions_ + last_velocities_ * update_period;
  Eigen::VectorXd upper_bound = vel_point - positions + max_acceleration_limits_ * (update_period * update_period);
  Eigen::VectorXd lower_bound = vel_point - positions + min_acceleration_limits_ * (update_period * update_period);
  if (!updateData(osqp_data_, osqp_workspace_, constraints_sparse_, lower_bound, upper_bound))
  {
    RCLCPP_ERROR_THROTTLE(getLogger(), *node_->get_clock(), 1000,
                          "failed to set osqp_update_bounds. Make sure the robot's acceleration limits are valid");
    return false;
  }

  if (positions_offset_.norm() < COMMAND_DIFFERENCE_THRESHOLD &&
      velocities_offset_.norm() < COMMAND_DIFFERENCE_THRESHOLD)
  {
    positions = last_positions_;
    velocities = last_velocities_;
  }
  else if (osqp_solve(osqp_workspace_) == 0 &&
           osqp_workspace_->solution->x[0] >= ALPHA_LOWER_BOUND - osqp_settings_.eps_abs &&
           osqp_workspace_->solution->x[0] <= ALPHA_UPPER_BOUND + osqp_settings_.eps_abs)
  {
    double alpha = osqp_workspace_->solution->x[0];
    positions = alpha * last_positions_ + (1.0 - alpha) * positions.eval();
    velocities = (positions - last_positions_) / update_period;
  }
  else
  {
    auto joint_model_group = robot_model_->getJointModelGroup(params_.planning_group_name);
    auto joint_bounds = joint_model_group->getActiveJointModelsBounds();
    cur_acceleration_ = -(last_velocities_) / update_period;
    cur_acceleration_ *= jointLimitAccelerationScalingFactor(cur_acceleration_, joint_bounds);
    velocities = last_velocities_ + cur_acceleration_ * update_period;
    positions = last_positions_ + velocities * update_period;
  }

  last_velocities_ = velocities;
  last_positions_ = positions;

  return true;
}

bool AccelerationLimitedPlugin::reset(const Eigen::VectorXd& positions, const Eigen::VectorXd& velocities,
                                      const Eigen::VectorXd& /* unused */)
{
  last_velocities_ = velocities;
  last_positions_ = positions;
  cur_acceleration_ = Eigen::VectorXd::Zero(num_joints_);

  return true;
}

}  // namespace online_signal_smoothing

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(online_signal_smoothing::AccelerationLimitedPlugin, online_signal_smoothing::SmoothingBaseClass)
