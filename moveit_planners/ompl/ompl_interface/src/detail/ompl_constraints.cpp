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

#include <tf2_eigen/tf2_eigen.hpp>

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
  assert(static_cast<long>(lower_.size()) == x.size());
  Eigen::VectorXd penalty(x.size());

  for (unsigned int i = 0; i < x.size(); ++i)
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
  assert(static_cast<long>(lower_.size()) == x.size());
  Eigen::VectorXd derivative(x.size());

  for (unsigned int i = 0; i < x.size(); ++i)
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
  for (std::size_t i = 0; i < bounds_.size(); ++i)
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
  assert(bounds_.size() == 0);
  bounds_ = positionConstraintMsgToBoundVector(constraints.position_constraints.at(0));

  // extract target position and orientation
  geometry_msgs::msg::Point position =
      constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).position;

  target_position_ << position.x, position.y, position.z;

  tf2::fromMsg(constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).orientation,
               target_orientation_);

  link_name_ = constraints.position_constraints.at(0).link_name;
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
  for (std::size_t i = 0; i < dims.size(); ++i)
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

  link_name_ = constraints.position_constraints.at(0).link_name;
}

void EqualityPositionConstraint::function(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                          Eigen::Ref<Eigen::VectorXd> out) const
{
  Eigen::Vector3d error =
      target_orientation_.matrix().transpose() * (forwardKinematics(joint_values).translation() - target_position_);
  for (std::size_t dim = 0; dim < 3; ++dim)
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
  for (std::size_t dim = 0; dim < 3; ++dim)
  {
    if (is_dim_constrained_.at(dim))
    {
      out.row(dim) = jac.row(dim);  // equality constraint dimension
    }
  }
}

/******************************************
 * Orientation constraints
 * ****************************************/
void OrientationConstraint::parseConstraintMsg(const moveit_msgs::msg::Constraints& constraints)
{
  bounds_ = orientationConstraintMsgToBoundVector(constraints.orientation_constraints.at(0));

  tf2::fromMsg(constraints.orientation_constraints.at(0).orientation, target_orientation_);

  link_name_ = constraints.orientation_constraints.at(0).link_name;
}

Eigen::VectorXd OrientationConstraint::calcError(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  Eigen::Matrix3d orientation_difference = forwardKinematics(x).linear().transpose() * target_orientation_;
  Eigen::AngleAxisd aa(orientation_difference);
  return aa.axis() * aa.angle();
}

Eigen::MatrixXd OrientationConstraint::calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  Eigen::Matrix3d orientation_difference = forwardKinematics(x).linear().transpose() * target_orientation_;
  Eigen::AngleAxisd aa{ orientation_difference };
  return -angularVelocityToAngleAxis(aa.angle(), aa.axis()) * robotGeometricJacobian(x).bottomRows(3);
}

/******************************************
 * Toolpath Constraint
 * ****************************************/
ToolPathConstraint::ToolPathConstraint(const moveit::core::RobotModelConstPtr& robot_model, const std::string& group,
                                       unsigned int num_dofs, const unsigned int num_cons)
  : BaseConstraint(robot_model, group, num_dofs, num_cons)
  , robot_model_{ robot_model }
  , group_{ group }
  , num_dofs_{ num_dofs }
{
  pose_nn_.setDistanceFunction([this](const EigenIsometry3dWrapper& pose1, const EigenIsometry3dWrapper& pose2) {
    const double translation_distance = (pose1.tform_.translation() - pose2.tform_.translation()).norm();
    const double rotation_distance = arcLength(pose1.tform_, pose2.tform_);
    return translation_distance + rotation_distance;
  });
}

void ToolPathConstraint::parseConstraintMsg(const moveit_msgs::msg::Constraints& constraints)
{
  const auto path_constraint = constraints.path_constraints.at(0);
  link_name_ = path_constraint.link_name;
  setPath(path_constraint);
  setTolerance(path_constraint.interp_distance * 2.5);
}

void ToolPathConstraint::setPath(const moveit_msgs::msg::PathConstraint& path_constraint)
{
  // Clear previous path
  path_.clear();
  pose_nn_.clear();
  box_constraints_.clear();

  // Create the new path
  const auto num_pts = path_constraint.path.size();
  if (num_pts > 1)
  {
    path_.reserve(num_pts);
    for (size_t i = 0; i < num_pts; i++)
    {
      const auto pose = path_constraint.path.at(i);
      Eigen::Isometry3d pose_eigen;
      tf2::fromMsg(pose, pose_eigen);
      path_.emplace_back(EigenIsometry3dWrapper(pose_eigen, std::min(i, num_pts - 2)));
    }

    // Add box constraints
    for (size_t i = 0; i < num_pts - 1; i++)
    {
      const auto pose = path_.at(i).tform_;
      const auto next_pose = path_.at(i + 1).tform_;

      auto pos_con = std::make_shared<BoxConstraint>(robot_model_, group_, num_dofs_);
      moveit_msgs::msg::PositionConstraint box_constraint;
      box_constraint.header = path_constraint.header;
      box_constraint.link_name = path_constraint.link_name;
      shape_msgs::msg::SolidPrimitive box;
      box.type = shape_msgs::msg::SolidPrimitive::BOX;
      box.dimensions = { 0.05, (pose.translation() - next_pose.translation()).norm() + 0.05,
                         0.005 };  // TODO: Set proper dimensions
      box_constraint.constraint_region.primitives.push_back(box);

      geometry_msgs::msg::Pose box_pose;
      box_pose.position.x = 0.5 * (pose.translation().x() + next_pose.translation().x());
      box_pose.position.y = 0.5 * (pose.translation().y() + next_pose.translation().y());
      box_pose.position.z = 0.5 * (pose.translation().z() + next_pose.translation().z());
      box_pose.orientation.w = 1.0;  // TODO: Orient the box
      box_constraint.constraint_region.primitive_poses.push_back(box_pose);
      box_constraint.weight = 1.0;

      moveit_msgs::msg::Constraints constr;
      constr.position_constraints.push_back(box_constraint);

      pos_con->init(constr);
      box_constraints_.emplace_back(pos_con);
      // std::cout << "Made box constraint index " << i << std::endl;
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Path constraint must specify at least 2 points!");
    return;
  }

  // Extract the interpolation distance
  if (path_constraint.interp_distance > 0.0)
  {
    interp_distance_ = path_constraint.interp_distance;
  }
  else
  {
    RCLCPP_WARN_STREAM(LOGGER, "Cannot use zero interpolation distance. Defaulting to 0.01.");
    interp_distance_ = 0.01;
  }

  // Interpolate along waypoints and insert into pose_nn_;
  for (size_t i = 0; i < num_pts - 1; i++)
  {
    // Insert waypoints
    pose_nn_.add(path_[i]);

    // TODO: Consider arc length between the poses to determine the number of interpolation steps
    // Linearly determine the number of steps
    const auto path_length = (path_[i].tform_.translation() - path_[i + 1].tform_.translation()).norm();
    const auto num_steps = static_cast<unsigned int>(path_length / interp_distance_);

    // Insert interpolated poses
    auto delta = interp_distance_;
    for (size_t j = 0; j < num_steps - 1; j++)
    {
      const auto pose = interpolatePose(path_[i].tform_, path_[i + 1].tform_, delta);
      pose_nn_.add(EigenIsometry3dWrapper(pose, i));
      delta += interp_distance_;
    }
  }

  // Insert last waypoint
  pose_nn_.add(path_[num_pts - 1]);
}

void ToolPathConstraint::function(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                  Eigen::Ref<Eigen::VectorXd> out) const
{
  const Eigen::Isometry3d eef_pose = forwardKinematics(joint_values);

  const auto eef_nearest = pose_nn_.nearest(EigenIsometry3dWrapper(eef_pose));
  // std::cout << "Function, nearest: " << eef_nearest.path_idx_ << std::endl;

  const auto bc = box_constraints_.at(eef_nearest.path_idx_);
  bc->function(joint_values, out);
}

void ToolPathConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                  Eigen::Ref<Eigen::MatrixXd> out) const
{
  const Eigen::Isometry3d eef_pose = forwardKinematics(joint_values);
  const auto eef_nearest = pose_nn_.nearest(EigenIsometry3dWrapper(eef_pose));
  // std::cout << "Jacobian, nearest: " << eef_nearest.path_idx_ << std::endl;

  const auto bc = box_constraints_.at(eef_nearest.path_idx_);
  bc->jacobian(joint_values, out);
}

double arcLength(const Eigen::Isometry3d& pose1, const Eigen::Isometry3d& pose2)
{
  const auto q1 = Eigen::Quaterniond(pose1.rotation());
  const auto q2 = Eigen::Quaterniond(pose2.rotation());
  return q1.angularDistance(q2);
}

Eigen::Isometry3d interpolatePose(const Eigen::Isometry3d& pose1, const Eigen::Isometry3d& pose2, double step)
{
  const Eigen::Translation3d interp_translation =
      Eigen::Translation3d((1.0 - step) * pose1.translation() + step * pose2.translation());

  const auto q1 = Eigen::Quaterniond(pose1.rotation());
  const auto q2 = Eigen::Quaterniond(pose2.rotation());
  const Eigen::Quaterniond interp_orientation = q1.slerp(step, q2);

  return interp_translation * interp_orientation;
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

Bounds orientationConstraintMsgToBoundVector(const moveit_msgs::msg::OrientationConstraint& ori_con)
{
  std::vector<double> dims = { ori_con.absolute_x_axis_tolerance * 2.0, ori_con.absolute_y_axis_tolerance * 2.0,
                               ori_con.absolute_z_axis_tolerance * 2.0 };

  // dimension of -1 signifies unconstrained parameter, so set to infinity
  for (auto& dim : dims)
  {
    if (dim == -1)
      dim = std::numeric_limits<double>::infinity();
  }
  return { { -dims[0], -dims[1], -dims[2] }, { dims[0], dims[1], dims[2] } };
}

/******************************************
 * OMPL Constraints Factory
 * ****************************************/
ompl::base::ConstraintPtr createOMPLConstraints(const moveit::core::RobotModelConstPtr& robot_model,
                                                const std::string& group,
                                                const moveit_msgs::msg::Constraints& constraints)
{
  // TODO(bostoncleek): does this reach the end w/o a return ?
  const std::size_t num_dofs = robot_model->getJointModelGroup(group)->getVariableCount();
  const std::size_t num_pos_con = constraints.position_constraints.size();
  const std::size_t num_ori_con = constraints.orientation_constraints.size();
  const std::size_t num_path_con = constraints.path_constraints.size();

  // This factory method contains template code to support constraints.
  // If the specified constraints are invalid, a nullptr is returned.
  std::vector<ompl::base::ConstraintPtr> ompl_constraints;
  if (num_pos_con > 1)
  {
    RCLCPP_WARN(LOGGER, "Only a single position constraint is supported. Using the first one.");
  }
  if (num_ori_con > 1)
  {
    RCLCPP_WARN(LOGGER, "Only a single orientation constraint is supported. Using the first one.");
  }
  if (num_path_con > 1)
  {
    RCLCPP_WARN(LOGGER, "Only a single path constraint is supported. Using the first one.");
  }

  if (num_pos_con > 0)
  {
    BaseConstraintPtr pos_con;
    if (constraints.name == "use_equality_constraints")
    {
      pos_con = std::make_shared<EqualityPositionConstraint>(robot_model, group, num_dofs);
    }
    else
    {
      pos_con = std::make_shared<BoxConstraint>(robot_model, group, num_dofs);
    }
    pos_con->init(constraints);
    ompl_constraints.emplace_back(pos_con);
  }
  if (num_ori_con > 0)
  {
    auto ori_con = std::make_shared<OrientationConstraint>(robot_model, group, num_dofs);
    ori_con->init(constraints);
    ompl_constraints.emplace_back(ori_con);
  }
  if (num_path_con > 0)
  {
    auto path_con = std::make_shared<ToolPathConstraint>(robot_model, group, num_dofs);
    path_con->init(constraints);
    ompl_constraints.emplace_back(path_con);
  }

  if (num_pos_con < 1 && num_ori_con < 1 && num_path_con < 1)
  {
    RCLCPP_ERROR(LOGGER, "No constraints found in planning request.");
    return nullptr;
  }
  return std::make_shared<ompl::base::ConstraintIntersection>(num_dofs, ompl_constraints);
}

}  // namespace ompl_interface
