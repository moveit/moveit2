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

/* Author: Paul Gesel
   Description: applies smoothing by limiting the acceleration between consecutive commands
 */

#include <moveit/online_signal_smoothing/acceleration_filter.h>
#include <rclcpp/logging.hpp>
#include <Eigen/Sparse>

// Disable -Wold-style-cast because all _THROTTLE macros trigger this
#pragma GCC diagnostic ignored "-Wold-style-cast"

namespace online_signal_smoothing
{
// The threshold below which any velocity or position difference is considered zero.
constexpr double COMMAND_DIFFERENCE_THRESHOLD = 1E-4;
constexpr double ALPHA_UPPER_BOUND = 1.0;
constexpr double ALPHA_LOWER_BOUND = 0.0;

struct CSCWrapper
{
  std::vector<c_int> i;
  std::vector<c_int> j;
  std::vector<double> x;
  csc sparse_matrix;

  CSCWrapper(Eigen::SparseMatrix<double>& M)
  {
    M.makeCompressed();

    sparse_matrix.n = M.cols();
    sparse_matrix.m = M.rows();
    i.assign(M.innerSize(), 0);
    sparse_matrix.i = i.data();
    j.assign(M.outerSize() + 1, 0);
    sparse_matrix.p = j.data();
    sparse_matrix.nzmax = M.nonZeros();
    sparse_matrix.nz = -1;
    x.assign(M.nonZeros(), 0.0);
    sparse_matrix.x = x.data();

    update(M);
  }

  void update(Eigen::SparseMatrix<double>& M)
  {
    for (size_t ind = 0; ind < i.size(); ++ind)
    {
      i[ind] = M.innerIndexPtr()[ind];
    }

    for (size_t ind = 0; ind < j.size(); ++ind)
    {
      j[ind] = M.outerIndexPtr()[ind];
    }
    for (size_t ind = 0; ind < x.size(); ++ind)
    {
      x[ind] = M.data().at(ind);
    }
  }
};

MOVEIT_STRUCT_FORWARD(OSQPDataWrapper);

struct OSQPDataWrapper
{
  OSQPDataWrapper(Eigen::SparseMatrix<double>& P_sparse, Eigen::SparseMatrix<double>& A_sparse)
    : P{ P_sparse }, A{ A_sparse }
  {
    data.n = P_sparse.rows();
    data.m = A_sparse.rows();
    data.P = &P.sparse_matrix;
    q = Eigen::VectorXd::Zero(P_sparse.rows());
    data.q = q.data();
    data.A = &A.sparse_matrix;
    l = Eigen::VectorXd::Zero(A_sparse.rows());
    data.l = l.data();
    u = Eigen::VectorXd::Zero(A_sparse.rows());
    data.u = u.data();
  }

  void update_A(OSQPWorkspace* work, Eigen::SparseMatrix<double>& A_sparse)
  {
    A_sparse.makeCompressed();
    A.update(A_sparse);
    osqp_update_A(work, A.x.data(), OSQP_NULL, A.x.size());
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
  if (!node_->has_parameter("update_rate") || !node_->has_parameter("move_group_name"))
  {
    return false;
  }
  auto param_listener = online_signal_smoothing::ParamListener(node_);
  auto params = param_listener.get_params();
  update_rate_ = params.update_rate;
  move_group_name_ = params.move_group_name;

  // get robot acceleration limits and store in member variables
  auto joint_model_group = robot_model_->getJointModelGroup(move_group_name_);
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
        RCLCPP_ERROR(node_->get_logger(), "The robot must have acceleration joint limits specified for all joint to "
                                          "use AccelerationLimitedPlugin.");
        return false;
      }
    }
    ind++;
  }

  // setup osqp optimization problem
  Eigen::SparseMatrix<double> P_sparse(1, 1);
  P_sparse.insert(0, 0) = 1.0;
  size_t num_constraints = num_joints + 1;
  A_sparse_ = Eigen::SparseMatrix<double>(num_constraints, 1);
  for (size_t i = 0; i < num_constraints - 1; ++i)
  {
    A_sparse_.insert(i, 0) = 0;
  }
  A_sparse_.insert(num_constraints - 1, 0) = 0;
  osqp_set_default_settings(&settings_);
  settings_.warm_start = 0;
  settings_.verbose = 0;
  data_ = std::make_shared<OSQPDataWrapper>(P_sparse, A_sparse_);
  data_->q[0] = 0;

  if (osqp_setup(&work_, &data_->data, &settings_) != 0)
  {
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

inline bool update_data(const OSQPDataWrapperPtr& data, OSQPWorkspace* work, Eigen::SparseMatrix<double>& A_sparse,
                        const Eigen::VectorXd& lower_bound, const Eigen::VectorXd& upper_bound)
{
  data->update_A(work, A_sparse);
  size_t num_constraints = A_sparse.rows();
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
        node_->get_logger(), *node_->get_clock(), 1000,
        "The length of the joint positions parameter is not equal to the number of joints, expected %zu got %zu.",
        num_joints_, num_positions);
    return false;
  }
  else if (last_positions_.size() != positions.size())
  {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "The length of the last joint positions not equal to the current, expected %zu got %zu. Make "
                          "sure the reset was called.",
                          last_positions_.size(), positions.size());
    return false;
  }

  size_t num_constraints = num_joints_ + 1;
  Eigen::VectorXd positions_offset = last_positions_ - positions;
  Eigen::VectorXd velocities_offset = last_velocities_ - velocities;
  for (size_t i = 0; i < num_constraints - 1; ++i)
  {
    A_sparse_.coeffRef(i, 0) = positions_offset[i];
  }
  A_sparse_.coeffRef(num_constraints - 1, 0) = 1;
  Eigen::VectorXd vel_point = last_positions_ + last_velocities_ * update_rate_;
  Eigen::VectorXd upper_bound = vel_point - positions + max_acceleration_limits_ * (update_rate_ * update_rate_);
  Eigen::VectorXd lower_bound = vel_point - positions + min_acceleration_limits_ * (update_rate_ * update_rate_);
  if (!update_data(data_, work_, A_sparse_, lower_bound, upper_bound))
  {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "failed to set osqp_update_bounds. Make sure the robot's acceleration limits are valid");
    return false;
  }

  if (positions_offset.norm() < COMMAND_DIFFERENCE_THRESHOLD && velocities_offset.norm() < COMMAND_DIFFERENCE_THRESHOLD)
  {
    positions = last_positions_;
    velocities = last_velocities_;
  }
  else if (osqp_solve(work_) == 0 && work_->solution->x[0] >= ALPHA_LOWER_BOUND - settings_.eps_abs &&
           work_->solution->x[0] <= ALPHA_UPPER_BOUND + settings_.eps_abs)
  {
    double alpha = work_->solution->x[0];
    positions = alpha * last_positions_ + (1.0 - alpha) * positions.eval();
    velocities = (positions - last_positions_) / update_rate_;
  }
  else
  {
    auto joint_model_group = robot_model_->getJointModelGroup(move_group_name_);
    auto joint_bounds = joint_model_group->getActiveJointModelsBounds();
    cur_acceleration_ = -(last_velocities_) / update_rate_;
    cur_acceleration_ *= jointLimitAccelerationScalingFactor(cur_acceleration_, joint_bounds);
    velocities = last_velocities_ + cur_acceleration_ * update_rate_;
    positions = last_positions_ + velocities * update_rate_;
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
