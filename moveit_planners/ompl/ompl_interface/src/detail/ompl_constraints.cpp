/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, KU Leuven
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
 *   * Neither the name of KU Leuven nor the names of its
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

/* Author: Jeroen De Maeyer, Boston Cleek */

#include <algorithm>
#include <iterator>

#include <moveit/ompl_interface/detail/ompl_constraints.h>

#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_planners_ompl.ompl_constraints");

namespace ompl_interface
{
Bounds::Bounds() : size_(0)
{
}

Bounds::Bounds(const std::vector<double>& lower, const std::vector<double>& upper)
  : lower_(lower), upper_(upper), size_(lower.size())
{
  // how to report this in release mode??
  assert(lower_.size() == upper_.size());
}

Eigen::VectorXd Bounds::penalty(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  assert((long)lower_.size() == x.size());
  Eigen::VectorXd penalty(x.size());

  for (unsigned int i = 0; i < x.size(); i++)
  {
    if (x[i] < lower_.at(i))
    {
      penalty[i] = lower_.at(i) - x[i];
    }
    else if (x[i] > upper_.at(i))
    {
      penalty[i] = x[i] - upper_.at(i);
    }
    else
    {
      penalty[i] = 0.0;
    }
  }
  return penalty;
}

Eigen::VectorXd Bounds::derivative(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  assert((long)lower_.size() == x.size());
  Eigen::VectorXd derivative(x.size());

  for (unsigned int i = 0; i < x.size(); i++)
  {
    if (x[i] < lower_.at(i))
    {
      derivative[i] = -1.0;
    }
    else if (x[i] > upper_.at(i))
    {
      derivative[i] = 1.0;
    }
    else
    {
      derivative[i] = 0.0;
    }
  }
  return derivative;
}

std::size_t Bounds::size() const
{
  return size_;
}

std::ostream& operator<<(std::ostream& os, const ompl_interface::Bounds& bounds)
{
  os << "Bounds:\n";
  for (std::size_t i{ 0 }; i < bounds.size(); ++i)
  {
    os << "( " << bounds.lower_[i] << ", " << bounds.upper_[i] << " )\n";
  }
  return os;
}

/****************************
 * Base class for constraints
 * **************************/
BaseConstraint::BaseConstraint(const moveit::core::RobotModelConstPtr& robot_model, const std::string& group,
                               const unsigned int num_dofs, const unsigned int num_cons_)
  : ompl::base::Constraint(num_dofs, num_cons_)
  , state_storage_(robot_model)
  , joint_model_group_(robot_model->getJointModelGroup(group))

{
}

void BaseConstraint::init(const moveit_msgs::msg::Constraints& constraints)
{
  parseConstraintMsg(constraints);
}

void BaseConstraint::function(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                              Eigen::Ref<Eigen::VectorXd> out) const
{
  const Eigen::VectorXd current_values = calcError(joint_values);
  out = bounds_.penalty(current_values);
}

void BaseConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                              Eigen::Ref<Eigen::MatrixXd> out) const
{
  const Eigen::VectorXd constraint_error = calcError(joint_values);
  const Eigen::VectorXd constraint_derivative = bounds_.derivative(constraint_error);
  const Eigen::MatrixXd robot_jacobian = calcErrorJacobian(joint_values);
  for (std::size_t i = 0; i < bounds_.size(); i++)
  {
    out.row(i) = constraint_derivative[i] * robot_jacobian.row(i);
  }
}

Eigen::Isometry3d BaseConstraint::forwardKinematics(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  moveit::core::RobotState* robot_state = state_storage_.getStateStorage();
  robot_state->setJointGroupPositions(joint_model_group_, joint_values);
  return robot_state->getGlobalLinkTransform(link_name_);
}

Eigen::MatrixXd BaseConstraint::robotGeometricJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  moveit::core::RobotState* robot_state = state_storage_.getStateStorage();
  robot_state->setJointGroupPositions(joint_model_group_, joint_values);
  Eigen::MatrixXd jacobian;
  // return value (success) not used, could return a garbage jacobian.
  robot_state->getJacobian(joint_model_group_, joint_model_group_->getLinkModel(link_name_),
                           Eigen::Vector3d(0.0, 0.0, 0.0), jacobian);
  return jacobian;
}

Eigen::VectorXd BaseConstraint::calcError(const Eigen::Ref<const Eigen::VectorXd>& /*x*/) const
{
  RCLCPP_WARN_STREAM(LOGGER,
                     "BaseConstraint: Constraint method calcError was not overridden, so it should not be used.");
  return Eigen::VectorXd::Zero(getCoDimension());
}

Eigen::MatrixXd BaseConstraint::calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& /*x*/) const
{
  RCLCPP_WARN_STREAM(
      LOGGER, "BaseConstraint: Constraint method calcErrorJacobian was not overridden, so it should not be used.");
  return Eigen::MatrixXd::Zero(getCoDimension(), n_);
}

/******************************************
 * Position constraints
 * ****************************************/
BoxConstraint::BoxConstraint(const moveit::core::RobotModelConstPtr& robot_model, const std::string& group,
                             const unsigned int num_dofs)
  : BaseConstraint(robot_model, group, num_dofs)
{
}

void BoxConstraint::parseConstraintMsg(const moveit_msgs::msg::Constraints& constraints)
{
  RCLCPP_DEBUG(LOGGER, "Parsing box position constraint for OMPL constrained state space.");
  assert(bounds_.size() == 0);
  bounds_ = positionConstraintMsgToBoundVector(constraints.position_constraints.at(0));
  RCLCPP_DEBUG(LOGGER, "Parsed Box constraints");
  RCLCPP_DEBUG_STREAM(LOGGER, "Bounds: " << bounds_);

  // extract target position and orientation
  geometry_msgs::msg::Point position =
      constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).position;

  target_position_ << position.x, position.y, position.z;

  tf2::fromMsg(constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).orientation,
               target_orientation_);

  link_name_ = constraints.position_constraints.at(0).link_name;
  RCLCPP_DEBUG_STREAM(LOGGER, "Position constraints applied to link: " << link_name_);
}

Eigen::VectorXd BoxConstraint::calcError(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  return target_orientation_.matrix().transpose() * (forwardKinematics(x).translation() - target_position_);
}

Eigen::MatrixXd BoxConstraint::calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  return target_orientation_.matrix().transpose() * robotGeometricJacobian(x).topRows(3);
}

/******************************************
 * Equality constraints
 * ****************************************/
EqualityPositionConstraint::EqualityPositionConstraint(const moveit::core::RobotModelConstPtr& robot_model,
                                                       const std::string& group, const unsigned int num_dofs)
  : BaseConstraint(robot_model, group, num_dofs)
{
}

void EqualityPositionConstraint::parseConstraintMsg(const moveit_msgs::msg::Constraints& constraints)
{
  const auto dims = constraints.position_constraints.at(0).constraint_region.primitives.at(0).dimensions;

  is_dim_constrained_ = { false, false, false };
  for (std::size_t i = 0; i < dims.size(); i++)
  {
    if (dims.at(i) < EQUALITY_CONSTRAINT_THRESHOLD)
    {
      if (dims.at(i) < getTolerance())
      {
        RCLCPP_ERROR_STREAM(
            LOGGER,
            "Dimension: " << i
                          << " of position constraint is smaller than the tolerance used to evaluate the constraints. "
                             "This will make all states invalid and planning will fail. Please use a value between: "
                          << getTolerance() << " and " << EQUALITY_CONSTRAINT_THRESHOLD);
      }

      is_dim_constrained_.at(i) = true;
    }
  }

  // extract target position and orientation
  geometry_msgs::msg::Point position =
      constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).position;

  target_position_ << position.x, position.y, position.z;

  tf2::fromMsg(constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).orientation,
               target_orientation_);

  RCLCPP_DEBUG_STREAM(LOGGER, "Equality constraint on x-position? " << (is_dim_constrained_[0] ? "yes" : "no"));
  RCLCPP_DEBUG_STREAM(LOGGER, "Equality constraint on y-position? " << (is_dim_constrained_[1] ? "yes" : "no"));
  RCLCPP_DEBUG_STREAM(LOGGER, "Equality constraint on z-position? " << (is_dim_constrained_[2] ? "yes" : "no"));

  link_name_ = constraints.position_constraints.at(0).link_name;
  RCLCPP_DEBUG_STREAM(LOGGER, "Position constraints applied to link: " << link_name_);
}

void EqualityPositionConstraint::function(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                          Eigen::Ref<Eigen::VectorXd> out) const
{
  Eigen::Vector3d error =
      target_orientation_.matrix().transpose() * (forwardKinematics(joint_values).translation() - target_position_);
  for (std::size_t dim = 0; dim < 3; dim++)
  {
    if (is_dim_constrained_.at(dim))
    {
      out[dim] = error[dim];  // equality constraint dimension
    }
    else
    {
      out[dim] = 0.0;  // unbounded dimension
    }
  }
}

void EqualityPositionConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                          Eigen::Ref<Eigen::MatrixXd> out) const
{
  out.setZero();
  Eigen::MatrixXd jac = target_orientation_.matrix().transpose() * robotGeometricJacobian(joint_values).topRows(3);
  for (std::size_t dim = 0; dim < 3; dim++)
  {
    if (is_dim_constrained_.at(dim))
    {
      out.row(dim) = jac.row(dim);  // equality constraint dimension
    }
  }
}

/************************************
 * MoveIt constraint message parsing
 * **********************************/
Bounds positionConstraintMsgToBoundVector(const moveit_msgs::msg::PositionConstraint& pos_con)
{
  auto dims = pos_con.constraint_region.primitives.at(0).dimensions;

  // dimension of -1 signifies unconstrained parameter, so set to infinity
  for (auto& dim : dims)
  {
    if (dim == -1)
    {
      dim = std::numeric_limits<double>::infinity();
    }
  }

  return { { -dims.at(0) / 2.0, -dims.at(1) / 2.0, -dims.at(2) / 2.0 },
           { dims.at(0) / 2.0, dims.at(1) / 2.0, dims.at(2) / 2.0 } };
}

/******************************************
 * OMPL Constraints Factory
 * ****************************************/
std::shared_ptr<BaseConstraint> createOMPLConstraint(const moveit::core::RobotModelConstPtr& robot_model,
                                                     const std::string& group,
                                                     const moveit_msgs::msg::Constraints& constraints)
{
  // TODO(bostoncleek): does this reach the end w/o a return ?

  const std::size_t num_dofs = robot_model->getJointModelGroup(group)->getVariableCount();
  const std::size_t num_pos_con = constraints.position_constraints.size();
  const std::size_t num_ori_con = constraints.orientation_constraints.size();

  // This factory method contains template code to support different constraints, but only position constraints are
  // currently supported. The other options return a nullptr for now and should not be used.

  if (num_pos_con > 1)
  {
    RCLCPP_WARN(LOGGER, "Only a single position constraints supported. Using the first one.");
  }
  if (num_ori_con > 1)
  {
    RCLCPP_WARN(LOGGER, "Only a single orientation constraints supported. Using the first one.");
  }

  if (num_pos_con > 0 && num_ori_con > 0)
  {
    RCLCPP_ERROR(LOGGER, "Combining position and orientation constraints not implemented yet for OMPL's constrained "
                         "state space.");
    return nullptr;
  }
  else if (num_pos_con > 0)
  {
    RCLCPP_DEBUG_STREAM(LOGGER, "Constraint name: " << constraints.name);
    BaseConstraintPtr pos_con;
    if (constraints.name == "use_equality_constraints")
    {
      RCLCPP_DEBUG(LOGGER, "OMPL is using equality position constraints.");
      pos_con = std::make_shared<EqualityPositionConstraint>(robot_model, group, num_dofs);
    }
    else
    {
      RCLCPP_DEBUG(LOGGER, "OMPL is using box position constraints.");
      pos_con = std::make_shared<BoxConstraint>(robot_model, group, num_dofs);
    }
    pos_con->init(constraints);
    return pos_con;
  }
  else if (num_ori_con > 0)
  {
    RCLCPP_ERROR(LOGGER, "Orientation constraints are not yet supported.");
    return nullptr;
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "No path constraints found in planning request.");
    return nullptr;
  }
}
}  // namespace ompl_interface
