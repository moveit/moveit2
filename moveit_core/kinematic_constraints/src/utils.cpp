/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#include <algorithm>

#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/utils/message_checks.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter_value.hpp>
#include <moveit/utils/logger.hpp>

#include <rclcpp/node.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

using namespace moveit::core;

namespace kinematic_constraints
{
namespace
{
rclcpp::Logger getLogger()
{
  static auto logger = moveit::makeChildLogger("moveit_kinematic_constraints");
  return logger;
}
}  // namespace

moveit_msgs::msg::Constraints mergeConstraints(const moveit_msgs::msg::Constraints& first,
                                               const moveit_msgs::msg::Constraints& second)
{
  moveit_msgs::msg::Constraints r;

  // add all joint constraints that are in first but not in second
  // and merge joint constraints that are for the same joint
  for (const moveit_msgs::msg::JointConstraint& jc_first : first.joint_constraints)
  {
    bool add = true;
    for (const moveit_msgs::msg::JointConstraint& jc_second : second.joint_constraints)
    {
      if (jc_second.joint_name == jc_first.joint_name)
      {
        add = false;
        // now we merge
        moveit_msgs::msg::JointConstraint m;
        const moveit_msgs::msg::JointConstraint& a = jc_first;
        const moveit_msgs::msg::JointConstraint& b = jc_second;
        double low = std::max(a.position - a.tolerance_below, b.position - b.tolerance_below);
        double high = std::min(a.position + a.tolerance_above, b.position + b.tolerance_above);
        if (low > high)
        {
          RCLCPP_ERROR(getLogger(),
                       "Attempted to merge incompatible constraints for joint '%s'. Discarding constraint.",
                       a.joint_name.c_str());
        }
        else
        {
          m.joint_name = a.joint_name;
          m.position =
              std::max(low, std::min((a.position * a.weight + b.position * b.weight) / (a.weight + b.weight), high));
          m.weight = (a.weight + b.weight) / 2.0;
          m.tolerance_above = std::max(0.0, high - m.position);
          m.tolerance_below = std::max(0.0, m.position - low);
          r.joint_constraints.push_back(m);
        }
        break;
      }
    }
    if (add)
      r.joint_constraints.push_back(jc_first);
  }

  // add all joint constraints that are in second but not in first
  for (const moveit_msgs::msg::JointConstraint& jc_second : second.joint_constraints)
  {
    bool add = true;
    for (const moveit_msgs::msg::JointConstraint& jc_first : first.joint_constraints)
    {
      if (jc_second.joint_name == jc_first.joint_name)
      {
        add = false;
        break;
      }
    }
    if (add)
      r.joint_constraints.push_back(jc_second);
  }

  // merge rest of constraints
  r.position_constraints = first.position_constraints;
  for (const moveit_msgs::msg::PositionConstraint& position_constraint : second.position_constraints)
    r.position_constraints.push_back(position_constraint);

  r.orientation_constraints = first.orientation_constraints;
  for (const moveit_msgs::msg::OrientationConstraint& orientation_constraint : second.orientation_constraints)
    r.orientation_constraints.push_back(orientation_constraint);

  r.visibility_constraints = first.visibility_constraints;
  for (const moveit_msgs::msg::VisibilityConstraint& visibility_constraint : second.visibility_constraints)
    r.visibility_constraints.push_back(visibility_constraint);

  return r;
}

std::size_t countIndividualConstraints(const moveit_msgs::msg::Constraints& constr)
{
  return constr.position_constraints.size() + constr.orientation_constraints.size() +
         constr.visibility_constraints.size() + constr.joint_constraints.size();
}

moveit_msgs::msg::Constraints constructGoalConstraints(const moveit::core::RobotState& state,
                                                       const moveit::core::JointModelGroup* jmg, double tolerance)
{
  return constructGoalConstraints(state, jmg, tolerance, tolerance);
}

moveit_msgs::msg::Constraints constructGoalConstraints(const moveit::core::RobotState& state,
                                                       const moveit::core::JointModelGroup* jmg, double tolerance_below,
                                                       double tolerance_above)
{
  moveit_msgs::msg::Constraints goal;
  std::vector<double> vals;
  state.copyJointGroupPositions(jmg, vals);
  goal.joint_constraints.resize(vals.size());
  for (std::size_t i = 0; i < vals.size(); ++i)
  {
    goal.joint_constraints[i].joint_name = jmg->getVariableNames()[i];
    goal.joint_constraints[i].position = vals[i];
    goal.joint_constraints[i].tolerance_above = tolerance_above;
    goal.joint_constraints[i].tolerance_below = tolerance_below;
    goal.joint_constraints[i].weight = 1.0;
  }

  return goal;
}

bool updateJointConstraints(moveit_msgs::msg::Constraints& constraints, const moveit::core::RobotState& state,
                            const moveit::core::JointModelGroup* jmg)
{
  const std::vector<std::string>& jmg_active_joints = jmg->getActiveJointModelNames();

  // For each constraint, update it if the joint is found within jmg
  for (auto& constraint : constraints.joint_constraints)
  {
    const auto itr = find(jmg_active_joints.begin(), jmg_active_joints.end(), constraint.joint_name);
    if (itr != jmg_active_joints.end())
    {
      constraint.position = state.getVariablePosition(constraint.joint_name);
    }
    // The joint was not found within jmg
    else
    {
      return false;
    }
  }
  return true;
}

moveit_msgs::msg::Constraints constructGoalConstraints(const std::string& link_name,
                                                       const geometry_msgs::msg::PoseStamped& pose,
                                                       double tolerance_pos, double tolerance_angle)
{
  moveit_msgs::msg::Constraints goal;

  goal.position_constraints.resize(1);
  moveit_msgs::msg::PositionConstraint& pcm = goal.position_constraints[0];
  pcm.link_name = link_name;
  pcm.target_point_offset.x = 0;
  pcm.target_point_offset.y = 0;
  pcm.target_point_offset.z = 0;
  pcm.constraint_region.primitives.resize(1);
  shape_msgs::msg::SolidPrimitive& bv = pcm.constraint_region.primitives[0];
  bv.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  bv.dimensions.resize(geometric_shapes::solidPrimitiveDimCount<shape_msgs::msg::SolidPrimitive::SPHERE>());
  bv.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = tolerance_pos;

  pcm.header = pose.header;
  pcm.constraint_region.primitive_poses.resize(1);
  // orientation of constraint region does not affect anything, since it is a sphere
  pcm.constraint_region.primitive_poses[0].position = pose.pose.position;
  pcm.weight = 1.0;

  goal.orientation_constraints.resize(1);
  moveit_msgs::msg::OrientationConstraint& ocm = goal.orientation_constraints[0];
  ocm.link_name = link_name;
  ocm.header = pose.header;
  ocm.orientation = pose.pose.orientation;
  ocm.absolute_x_axis_tolerance = tolerance_angle;
  ocm.absolute_y_axis_tolerance = tolerance_angle;
  ocm.absolute_z_axis_tolerance = tolerance_angle;
  ocm.weight = 1.0;

  return goal;
}

moveit_msgs::msg::Constraints constructGoalConstraints(const std::string& link_name,
                                                       const geometry_msgs::msg::PoseStamped& pose,
                                                       const std::vector<double>& tolerance_pos,
                                                       const std::vector<double>& tolerance_angle)
{
  moveit_msgs::msg::Constraints goal = constructGoalConstraints(link_name, pose);
  if (tolerance_pos.size() == 3)
  {
    shape_msgs::msg::SolidPrimitive& bv = goal.position_constraints[0].constraint_region.primitives[0];
    bv.type = shape_msgs::msg::SolidPrimitive::BOX;
    bv.dimensions.resize(geometric_shapes::solidPrimitiveDimCount<shape_msgs::msg::SolidPrimitive::BOX>());
    bv.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = tolerance_pos[0];
    bv.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = tolerance_pos[1];
    bv.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = tolerance_pos[2];
  }
  if (tolerance_angle.size() == 3)
  {
    moveit_msgs::msg::OrientationConstraint& ocm = goal.orientation_constraints[0];
    ocm.absolute_x_axis_tolerance = tolerance_angle[0];
    ocm.absolute_y_axis_tolerance = tolerance_angle[1];
    ocm.absolute_z_axis_tolerance = tolerance_angle[2];
  }
  return goal;
}

bool updatePoseConstraint(moveit_msgs::msg::Constraints& constraints, const std::string& link_name,
                          const geometry_msgs::msg::PoseStamped& pose)
{
  // Convert message types so the existing functions can be used
  geometry_msgs::msg::PointStamped point;
  point.header = pose.header;
  point.point.x = pose.pose.position.x;
  point.point.y = pose.pose.position.y;
  point.point.z = pose.pose.position.z;

  geometry_msgs::msg::QuaternionStamped quat_stamped;
  quat_stamped.header = pose.header;
  quat_stamped.quaternion = pose.pose.orientation;

  return updatePositionConstraint(constraints, link_name, point) &&
         updateOrientationConstraint(constraints, link_name, quat_stamped);
}

moveit_msgs::msg::Constraints constructGoalConstraints(const std::string& link_name,
                                                       const geometry_msgs::msg::QuaternionStamped& quat,
                                                       double tolerance)
{
  moveit_msgs::msg::Constraints goal;
  goal.orientation_constraints.resize(1);
  moveit_msgs::msg::OrientationConstraint& ocm = goal.orientation_constraints[0];
  ocm.link_name = link_name;
  ocm.header = quat.header;
  ocm.orientation = quat.quaternion;
  ocm.absolute_x_axis_tolerance = tolerance;
  ocm.absolute_y_axis_tolerance = tolerance;
  ocm.absolute_z_axis_tolerance = tolerance;
  ocm.weight = 1.0;
  return goal;
}

bool updateOrientationConstraint(moveit_msgs::msg::Constraints& constraints, const std::string& link_name,
                                 const geometry_msgs::msg::QuaternionStamped& quat)
{
  for (auto& constraint : constraints.orientation_constraints)
  {
    if (constraint.link_name == link_name)
    {
      if (quat.header.frame_id.empty())
      {
        RCLCPP_ERROR(getLogger(), "Cannot update orientation constraint, frame_id in the header is empty");
        return false;
      }
      constraint.header = quat.header;
      constraint.orientation = quat.quaternion;
      return true;
    }
  }
  return false;
}

moveit_msgs::msg::Constraints constructGoalConstraints(const std::string& link_name,
                                                       const geometry_msgs::msg::PointStamped& goal_point,
                                                       double tolerance)
{
  geometry_msgs::msg::Point p;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  return constructGoalConstraints(link_name, p, goal_point, tolerance);
}

bool updatePositionConstraint(moveit_msgs::msg::Constraints& constraints, const std::string& link_name,
                              const geometry_msgs::msg::PointStamped& goal_point)
{
  for (auto& constraint : constraints.position_constraints)
  {
    if (constraint.link_name == link_name)
    {
      if (goal_point.header.frame_id.empty())
      {
        RCLCPP_ERROR(getLogger(), "Cannot update position constraint, frame_id in the header is empty");
        return false;
      }
      constraint.header = goal_point.header;
      constraint.constraint_region.primitive_poses.at(0).position.x = goal_point.point.x;
      constraint.constraint_region.primitive_poses.at(0).position.y = goal_point.point.y;
      constraint.constraint_region.primitive_poses.at(0).position.z = goal_point.point.z;
      return true;
    }
  }
  return false;
}

moveit_msgs::msg::Constraints constructGoalConstraints(const std::string& link_name,
                                                       const geometry_msgs::msg::Point& reference_point,
                                                       const geometry_msgs::msg::PointStamped& goal_point,
                                                       double tolerance)
{
  moveit_msgs::msg::Constraints goal;
  goal.position_constraints.resize(1);
  moveit_msgs::msg::PositionConstraint& pcm = goal.position_constraints[0];
  pcm.link_name = link_name;
  pcm.target_point_offset.x = reference_point.x;
  pcm.target_point_offset.y = reference_point.y;
  pcm.target_point_offset.z = reference_point.z;
  pcm.constraint_region.primitives.resize(1);
  pcm.constraint_region.primitives[0].type = shape_msgs::msg::SolidPrimitive::SPHERE;
  pcm.constraint_region.primitives[0].dimensions.resize(
      geometric_shapes::solidPrimitiveDimCount<shape_msgs::msg::SolidPrimitive::SPHERE>());
  pcm.constraint_region.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = tolerance;

  pcm.header = goal_point.header;
  pcm.constraint_region.primitive_poses.resize(1);
  pcm.constraint_region.primitive_poses[0].position = goal_point.point;

  // orientation of constraint region does not affect anything, since it is a sphere
  pcm.constraint_region.primitive_poses[0].orientation.x = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.y = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.z = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.w = 1.0;
  pcm.weight = 1.0;

  return goal;
}

// Initialize a PoseStamped message from node parameters specified at pose_param.
static bool constructPoseStamped(const rclcpp::Node::SharedPtr& node, const std::string& pose_param,
                                 geometry_msgs::msg::PoseStamped& pose)
{
  if (!node->get_parameter(pose_param + ".frame_id", pose.header.frame_id))
    return false;

  std::vector<double> orientation;
  if (!node->get_parameter(pose_param + ".orientation", orientation) || orientation.size() != 3)
    return false;

  tf2::Quaternion q;
  q.setRPY(orientation[0], orientation[1], orientation[2]);
  pose.pose.orientation = toMsg(q);

  std::vector<double> position;
  if (!node->get_parameter(pose_param + ".position", position) || position.size() != 3)
    return false;

  pose.pose.position.x = position[0];
  pose.pose.position.y = position[1];
  pose.pose.position.z = position[2];

  return true;
}

// Initialize a JointConstraint message from node parameters specified at constraint_param.
static bool constructConstraint(const rclcpp::Node::SharedPtr& node, const std::string& constraint_param,
                                moveit_msgs::msg::JointConstraint& constraint)
{
  node->get_parameter(constraint_param + ".weight", constraint.weight);
  node->get_parameter(constraint_param + ".joint_name", constraint.joint_name);
  node->get_parameter(constraint_param + ".position", constraint.position);

  double tolerance;
  if (node->get_parameter(constraint_param + ".tolerance", tolerance))
  {
    constraint.tolerance_below = tolerance;
    constraint.tolerance_above = tolerance;
  }
  else if (node->has_parameter(constraint_param + ".tolerances"))
  {
    std::vector<double> tolerances;
    node->get_parameter(constraint_param + ".tolerances", tolerances);
    if (tolerances.size() != 2)
      return false;

    constraint.tolerance_below = tolerances[0];
    constraint.tolerance_above = tolerances[1];
  }
  else if (node->has_parameter(constraint_param + ".bounds"))
  {
    std::vector<double> bounds;
    node->get_parameter(constraint_param + ".bounds", bounds);
    if (bounds.size() != 2)
      return false;

    const double lower_bound = bounds[0];
    const double upper_bound = bounds[1];

    constraint.position = (lower_bound + upper_bound) / 2;
    constraint.tolerance_below = constraint.position - lower_bound;
    constraint.tolerance_above = upper_bound - constraint.position;
  }

  return true;
}

// Initialize a PositionConstraint message from node parameters specified at constraint_param.
static bool constructConstraint(const rclcpp::Node::SharedPtr& node, const std::string& constraint_param,
                                moveit_msgs::msg::PositionConstraint& constraint)
{
  node->get_parameter(constraint_param + ".frame_id", constraint.header.frame_id);
  node->get_parameter(constraint_param + ".weight", constraint.weight);
  node->get_parameter(constraint_param + ".link_name", constraint.link_name);

  std::vector<double> target_offset;
  if (node->get_parameter(constraint_param + ".target_offset", target_offset))
  {
    if (target_offset.size() != 3)
      return false;

    constraint.target_point_offset.x = target_offset[0];
    constraint.target_point_offset.y = target_offset[1];
    constraint.target_point_offset.z = target_offset[2];
  }
  if (!node->list_parameters({ constraint_param + ".region" }, 1).names.empty())  // TODO(henningkayser): specify depth
  {
    geometry_msgs::msg::Pose region_pose;
    region_pose.orientation.w = 1.0;

    shape_msgs::msg::SolidPrimitive region_primitive;
    region_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    region_primitive.dimensions.resize(3);

    const auto parse_dimension = [&](const std::string& dimension_param, double& center, double& dimension) -> bool {
      std::vector<double> dimension_limits;
      if (!node->get_parameter(constraint_param + ".region." + dimension_param, dimension_limits) ||
          dimension_limits.size() != 2)
        return false;

      center = (dimension_limits[0] + dimension_limits[1]) / 2;
      dimension = dimension_limits[1] - dimension_limits[2];
      return true;
    };

    if (!parse_dimension("x", region_pose.position.x,
                         region_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X]) ||
        !parse_dimension("y", region_pose.position.y,
                         region_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y]) ||
        !parse_dimension("z", region_pose.position.z,
                         region_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z]))
      return false;

    constraint.constraint_region.primitive_poses.push_back(region_pose);
    constraint.constraint_region.primitives.emplace_back(region_primitive);
  }

  return true;
}

// Initialize an OrientationConstraint message from node parameters specified at constraint_param.
static bool constructConstraint(const rclcpp::Node::SharedPtr& node, const std::string& constraint_param,
                                moveit_msgs::msg::OrientationConstraint& constraint)
{
  node->get_parameter(constraint_param + ".frame_id", constraint.header.frame_id);
  node->get_parameter(constraint_param + ".weight", constraint.weight);
  node->get_parameter(constraint_param + ".link_name", constraint.link_name);

  std::vector<double> orientation;
  if (node->get_parameter(constraint_param + ".orientation", orientation))
  {
    if (orientation.size() != 3)
      return false;

    tf2::Quaternion q;
    q.setRPY(orientation[0], orientation[1], orientation[2]);
    constraint.orientation = toMsg(q);
  }

  std::vector<double> tolerances;
  if (node->get_parameter(constraint_param + ".tolerances", tolerances))
  {
    if (tolerances.size() != 3)
      return false;

    constraint.absolute_x_axis_tolerance = tolerances[0];
    constraint.absolute_y_axis_tolerance = tolerances[1];
    constraint.absolute_z_axis_tolerance = tolerances[2];
  }

  return true;
}

// Initialize a VisibilityConstraint message from node parameters specified at constraint_param.
static bool constructConstraint(const rclcpp::Node::SharedPtr& node, const std::string& constraint_param,
                                moveit_msgs::msg::VisibilityConstraint& constraint)
{
  node->get_parameter(constraint_param + ".weight", constraint.weight);
  node->get_parameter(constraint_param + ".target_radius", constraint.target_radius);
  node->get_parameter(constraint_param + ".cone_sides", constraint.cone_sides);
  node->get_parameter(constraint_param + ".max_view_angle", constraint.max_view_angle);
  node->get_parameter(constraint_param + ".max_range_angle", constraint.max_range_angle);

  // TODO(henningkayser): specify depth
  if (!node->list_parameters({ constraint_param + ".target_pose" }, 1).names.empty())
  {
    if (!constructPoseStamped(node, constraint_param + ".target_pose", constraint.target_pose))
      return false;
  }
  // TODO(henningkayser): specify depth
  if (!node->list_parameters({ constraint_param + ".sensor_pose" }, 1).names.empty())
  {
    if (!constructPoseStamped(node, constraint_param + ".sensor_pose", constraint.sensor_pose))
      return false;
  }

  constraint.sensor_view_direction = moveit_msgs::msg::VisibilityConstraint::SENSOR_X;

  return true;
}

// Initialize a Constraints message containing constraints specified by node parameters under constraint_ids.
static bool collectConstraints(const rclcpp::Node::SharedPtr& node, const std::vector<std::string>& constraint_ids,
                               moveit_msgs::msg::Constraints& constraints)
{
  for (const auto& constraint_id : constraint_ids)
  {
    const auto constraint_param = "constraints." + constraint_id;
    if (!node->has_parameter(constraint_param + ".type"))
    {
      RCLCPP_ERROR(getLogger(), "constraint parameter \"%s\" does not specify its type", constraint_param.c_str());
      return false;
    }
    std::string constraint_type;
    node->get_parameter(constraint_param + ".type", constraint_type);
    if (constraint_type == "joint")
    {
      constraints.joint_constraints.emplace_back();
      if (!constructConstraint(node, constraint_param, constraints.joint_constraints.back()))
        return false;
    }
    else if (constraint_type == "position")
    {
      constraints.position_constraints.emplace_back();
      if (!constructConstraint(node, constraint_param, constraints.position_constraints.back()))
        return false;
    }
    else if (constraint_type == "orientation")
    {
      constraints.orientation_constraints.emplace_back();
      if (!constructConstraint(node, constraint_param, constraints.orientation_constraints.back()))
        return false;
    }
    else if (constraint_type == "visibility")
    {
      constraints.visibility_constraints.emplace_back();
      if (!constructConstraint(node, constraint_param, constraints.visibility_constraints.back()))
        return false;
    }
    else
    {
      RCLCPP_ERROR_STREAM(getLogger(), "Unable to process unknown constraint type: " << constraint_type);
      return false;
    }
  }

  return true;
}

bool constructConstraints(const rclcpp::Node::SharedPtr& node, const std::string& constraints_param,
                          moveit_msgs::msg::Constraints& constraints)
{
  if (!node->get_parameter(constraints_param + ".name", constraints.name))
    return false;

  std::vector<std::string> constraint_ids;
  if (!node->get_parameter(constraints_param + ".constraint_ids", constraint_ids))
    return false;

  for (auto& constraint_id : constraint_ids)
    constraint_id.insert(0, constraints_param + std::string("."));

  return collectConstraints(node, constraint_ids, constraints);
}

bool resolveConstraintFrames(const moveit::core::RobotState& state, moveit_msgs::msg::Constraints& constraints)
{
  for (auto& c : constraints.position_constraints)
  {
    bool frame_found;
    const moveit::core::LinkModel* robot_link;
    const Eigen::Isometry3d& transform = state.getFrameInfo(c.link_name, robot_link, frame_found);
    if (!frame_found)
      return false;

    // If the frame of the constraint is not part of the robot link model (but an attached body or subframe),
    // the constraint needs to be expressed in the frame of a robot link.
    if (c.link_name != robot_link->getName())
    {
      Eigen::Isometry3d robot_link_to_link_name = state.getGlobalLinkTransform(robot_link).inverse() * transform;
      Eigen::Vector3d offset_link_name(c.target_point_offset.x, c.target_point_offset.y, c.target_point_offset.z);
      Eigen::Vector3d offset_robot_link = robot_link_to_link_name * offset_link_name;

      c.link_name = robot_link->getName();
      tf2::toMsg(offset_robot_link, c.target_point_offset);
    }
  }

  for (auto& c : constraints.orientation_constraints)
  {
    bool frame_found;
    const moveit::core::LinkModel* robot_link;
    // getFrameInfo() returns a valid isometry by contract
    const Eigen::Isometry3d& transform = state.getFrameInfo(c.link_name, robot_link, frame_found);
    if (!frame_found)
      return false;

    // If the frame of the constraint is not part of the robot link model (but an attached body or subframe),
    // the constraint needs to be expressed in the frame of a robot link.
    if (c.link_name != robot_link->getName())
    {
      c.link_name = robot_link->getName();
      Eigen::Quaterniond link_name_to_robot_link(transform.linear().transpose() *
                                                 state.getGlobalLinkTransform(robot_link).linear());
      Eigen::Quaterniond quat_target;
      tf2::fromMsg(c.orientation, quat_target);
      c.orientation = tf2::toMsg(quat_target * link_name_to_robot_link);
    }
  }
  return true;
}
}  // namespace kinematic_constraints
