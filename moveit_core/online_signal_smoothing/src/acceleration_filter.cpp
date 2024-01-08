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
   Description: Applies smoothing by limiting the  acceleration between consecutive commands
 */

#include <moveit/online_signal_smoothing/acceleration_filter.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <Eigen/Sparse>

// Disable -Wold-style-cast because all _THROTTLE macros trigger this
#pragma GCC diagnostic ignored "-Wold-style-cast"

namespace online_signal_smoothing
{
// The threshold above which `override_velocity_scaling_factor` will be used instead of computing the scaling from joint bounds.
constexpr double SCALING_OVERRIDE_THRESHOLD = 0.01;

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
    j.assign(M.outerSize(), 0);
    sparse_matrix.p = j.data();
    sparse_matrix.nzmax = M.nonZeros();
    sparse_matrix.nz = -1;
    x.assign(M.nonZeros(), 0.0);
    sparse_matrix.x = x.data();

    for (Eigen::Index ind = 0; ind < i.size(); ++ind)
    {
      i[ind] = M.innerIndexPtr()[ind];
    }
    for (Eigen::Index ind = 0; ind < j.size(); ++ind)
    {
      j[ind] = M.outerIndexPtr()[ind];
     }
    for (Eigen::Index ind = 0; ind < x.size(); ++ind)
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
    assert(A_sparse.nonZeros() == A.x.size());
    for (Eigen::Index ind = 0; ind < A.i.size(); ++ind)
    {
      A.i[ind] = A_sparse.innerIndexPtr()[ind];
    }
    for (Eigen::Index ind = 0; ind < A.j.size(); ++ind)
    {
      A.j[ind] = A_sparse.outerIndexPtr()[ind];
    }
    for (Eigen::Index ind = 0; ind < A.x.size(); ++ind)
    {
      A.x[ind] = A_sparse.data().at(ind);
    }
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
  auto param_listener = online_signal_smoothing::ParamListener(node_);
  auto params = param_listener.get_params();
  update_rate_ = params.update_rate;
  move_group_name_ = params.move_group_name;

  // access robot model information for given move group
  robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model);
  variable_names_ = robot_model->getJointModelGroup(move_group_name_)->getVariableNames();
  robot_model->getJointModelGroup(move_group_name_);
  auto joint_model_group = robot_model_->getJointModelGroup(move_group_name_);

  // get robot acceleration limits and store in member variables
  min_acceleration_limits_ = Eigen::VectorXd::Zero(variable_names_.size());
  max_acceleration_limits_ = Eigen::VectorXd::Zero(variable_names_.size());
  auto joint_bounds = joint_model_group->getActiveJointModelsBounds();
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
  size_t num_dims = variable_names_.size();
  size_t num_constraints = num_dims + 1;

  Eigen::SparseMatrix<double> P_sparse(1, 1);
  P_sparse.insert(0, 0) = 0.0;

  Eigen::SparseMatrix<double> A_sparse(num_constraints, 1);
  for (size_t i = 0; i < num_constraints - 1; ++i)
  {
    A_sparse.insert(i, 0) = 0;
  }
  A_sparse.insert(num_constraints - 1, 0) = 0;

  osqp_set_default_settings(&settings_);
  settings_.warm_start = 0;
  settings_.verbose = 0;
  data_ = std::make_shared<OSQPDataWrapper>(P_sparse, A_sparse);
  data_->q[0] = 1;

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

  auto update_data = [this](Eigen::SparseMatrix<double> &  A_sparse, const Eigen::VectorXd& lower_bound, const Eigen::VectorXd& upper_bound){
    data_->update_A(work_, A_sparse);
    size_t num_constraints  = A_sparse.rows();
    data_->u.block(0, 0, num_constraints - 1, 1) = upper_bound;
    data_->l.block(0, 0, num_constraints - 1, 1) = lower_bound;
    data_->u[num_constraints - 1] = 1.0;
    data_->l[num_constraints - 1] = 0.0;
    osqp_update_bounds(work_, data_->l.data(), data_->u.data());
  };

  size_t num_dims = variable_names_.size();
  size_t num_constraints = num_dims + 1;
  Eigen::SparseMatrix<double> A_sparse(num_constraints, 1);
  Eigen::VectorXd offset = last_positions_ - positions;
  for (int i = 0; i < num_constraints - 1; ++i)
  {
    A_sparse.insert(i, 0) = offset[i];
  }
  A_sparse.insert(num_constraints - 1, 0) = 1;
  Eigen::VectorXd vel_point = last_positions_ + last_velocities_ * update_rate_;
  Eigen::VectorXd upper_bound = vel_point - positions + 100*max_acceleration_limits_ * (update_rate_ * update_rate_);
  Eigen::VectorXd lower_bound = vel_point - positions + 100*min_acceleration_limits_ * (update_rate_ * update_rate_);
  update_data(A_sparse, lower_bound, upper_bound);

  if (osqp_solve(work_) == 0 && work_->solution->x[0] >= -0.0001 && work_->solution->x[0] <= 1.0001)
  {
    double alpha = work_->solution->x[0];
    RCLCPP_ERROR(node_->get_logger(), "alpha: %f", alpha);
    Eigen::VectorXd tmp = (1 - alpha) * positions;
    positions = alpha * last_positions_ + tmp;
    velocities = (positions - last_positions_) / update_rate_;
  }
  else
  {
    auto joint_model_group = robot_model_->getJointModelGroup(move_group_name_);
    Eigen::MatrixXd J = robot_state_->getJacobian(joint_model_group);
    Eigen::VectorXd target = J * last_velocities_;
    Eigen::MatrixXd Jinv = J.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::MatrixXd cartesian_point = Jinv * target;
    offset = 100*positions - 100*last_positions_;
    for (int i = 0; i < num_constraints - 1; ++i)
    {
      A_sparse.coeffRef(i, 0) = offset[i];
    }
    upper_bound = vel_point - last_positions_ + max_acceleration_limits_ * (update_rate_ * update_rate_);
    lower_bound = vel_point - last_positions_ + min_acceleration_limits_ * (update_rate_ * update_rate_);
    update_data(A_sparse, lower_bound, upper_bound);

    if (osqp_solve(work_) == 0 && work_->solution->x[0] >= -0.0001 && work_->solution->x[0] <= 1.0001)
    {
      double alpha = work_->solution->x[0];
      RCLCPP_ERROR(node_->get_logger(), "alpha: %f", alpha);
      Eigen::VectorXd tmp = (1 - alpha) * positions;
      positions = alpha * last_positions_ + tmp;
      velocities = (positions - last_positions_) / update_rate_;
    } else{
      RCLCPP_ERROR(node_->get_logger(), "Too fast!!");
    }
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
