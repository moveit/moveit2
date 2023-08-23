// Copyright 2020 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \author Adolfo Rodriguez Tsouroukdissian

#pragma once

// TODO(henning): This file is copied from the DRAFT PR https://github.com/ros-controls/ros2_control/pull/462 since the
// current ros2_control implementation does not offer the desired joint limits API, yet. Remove when ros2_control has an
// upstream version of this.

#include <limits>
#include <string>

#include <joint_limits_copy/joint_limits.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
/** declare a parameter if not already declared. */
template <typename T>
void declare_parameter(const rclcpp::Node::SharedPtr& node, const std::string& name, T default_value)
{
  if (not node->has_parameter(name))
  {
    node->declare_parameter<T>(name, default_value);
  }
}
}  // namespace

namespace joint_limits
{
inline bool declare_parameters(const std::string& joint_name, const rclcpp::Node::SharedPtr& node,
                               const std::string& param_ns)
{
  const std::string param_base_name = (param_ns.empty() ? "" : param_ns + ".") + "joint_limits." + joint_name;

  try
  {
    declare_parameter(node, param_base_name + ".has_position_limits", false);
    declare_parameter(node, param_base_name + ".min_position", std::numeric_limits<double>::quiet_NaN());
    declare_parameter(node, param_base_name + ".max_position", std::numeric_limits<double>::quiet_NaN());
    declare_parameter(node, param_base_name + ".has_velocity_limits", false);
    declare_parameter(node, param_base_name + ".min_velocity", std::numeric_limits<double>::quiet_NaN());
    declare_parameter(node, param_base_name + ".max_velocity", std::numeric_limits<double>::quiet_NaN());
    declare_parameter(node, param_base_name + ".has_acceleration_limits", false);
    declare_parameter(node, param_base_name + ".max_acceleration", std::numeric_limits<double>::quiet_NaN());
    declare_parameter(node, param_base_name + ".has_jerk_limits", false);
    declare_parameter(node, param_base_name + ".max_jerk", std::numeric_limits<double>::quiet_NaN());
    declare_parameter(node, param_base_name + ".has_effort_limits", false);
    declare_parameter(node, param_base_name + ".max_effort", std::numeric_limits<double>::quiet_NaN());
    declare_parameter(node, param_base_name + ".angle_wraparound", false);
    declare_parameter(node, param_base_name + ".has_soft_limits", false);
    declare_parameter(node, param_base_name + ".k_position", std::numeric_limits<double>::quiet_NaN());
    declare_parameter(node, param_base_name + ".k_velocity", std::numeric_limits<double>::quiet_NaN());
    declare_parameter(node, param_base_name + ".soft_lower_limit", std::numeric_limits<double>::quiet_NaN());
    declare_parameter(node, param_base_name + ".soft_upper_limit", std::numeric_limits<double>::quiet_NaN());
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(node->get_logger(), "%s", ex.what());
    return false;
  }
  return true;
}

/// Populate a JointLimits instance from the ROS parameter server.
/**
 * It is assumed that the following parameter structure is followed on the provided NodeHandle. Unspecified parameters
 * are simply not added to the joint limits specification.
 * \code
 * joint_limits:
 *   foo_joint:
 *     has_position_limits: true
 *     min_position: 0.0
 *     max_position: 1.0
 *     has_velocity_limits: true
 *     max_velocity: 2.0
 *     has_acceleration_limits: true
 *     max_acceleration: 5.0
 *     has_jerk_limits: true
 *     max_jerk: 100.0
 *     has_effort_limits: true
 *     max_effort: 20.0
 *   bar_joint:
 *     has_position_limits: false # Continuous joint
 *     has_velocity_limits: true
 *     max_velocity: 4.0
 * \endcode
 *
 * This specification is similar to the one used by <a href="http://moveit.ros.org/wiki/MoveIt!">MoveIt!</a>,
 * but additionally supports jerk and effort limits.
 *
 * \param[in] joint_name Name of joint whose limits are to be fetched.
 * \param[in] node NodeHandle where the joint limits are specified.
 * \param[out] limits Where joint limit data gets written into. Limits specified in the parameter server will overwrite
 * existing values. Values in \p limits not specified in the parameter server remain unchanged.
 * \return True if a limits specification is found (ie. the \p joint_limits/joint_name parameter exists in \p node),
 * false otherwise.
 */
inline bool get_joint_limits(const std::string& joint_name, const rclcpp::Node::SharedPtr& node,
                             const std::string& param_ns, JointLimits& limits)
{
  const std::string param_base_name = (param_ns.empty() ? "" : param_ns + ".") + "joint_limits." + joint_name;
  try
  {
    if (!node->has_parameter(param_base_name + ".has_position_limits") &&
        !node->has_parameter(param_base_name + ".min_position") &&
        !node->has_parameter(param_base_name + ".max_position") &&
        !node->has_parameter(param_base_name + ".has_velocity_limits") &&
        !node->has_parameter(param_base_name + ".min_velocity") &&
        !node->has_parameter(param_base_name + ".max_velocity") &&
        !node->has_parameter(param_base_name + ".has_acceleration_limits") &&
        !node->has_parameter(param_base_name + ".max_acceleration") &&
        !node->has_parameter(param_base_name + ".has_jerk_limits") &&
        !node->has_parameter(param_base_name + ".max_jerk") &&
        !node->has_parameter(param_base_name + ".has_effort_limits") &&
        !node->has_parameter(param_base_name + ".max_effort") &&
        !node->has_parameter(param_base_name + ".angle_wraparound") &&
        !node->has_parameter(param_base_name + ".has_soft_limits") &&
        !node->has_parameter(param_base_name + ".k_position") &&
        !node->has_parameter(param_base_name + ".k_velocity") &&
        !node->has_parameter(param_base_name + ".soft_lower_limit") &&
        !node->has_parameter(param_base_name + ".soft_upper_limit"))
    {
      RCLCPP_ERROR(node->get_logger(),
                   "No joint limits specification found for joint '%s' in the parameter server "
                   "(node: %s param name: %s).",
                   joint_name.c_str(), node->get_name(), param_base_name.c_str());
      return false;
    }
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(node->get_logger(), "%s", ex.what());
    return false;
  }

  // Position limits
  bool has_position_limits = false;
  if (node->get_parameter(param_base_name + ".has_position_limits", has_position_limits))
  {
    if (!has_position_limits)
    {
      limits.has_position_limits = false;
    }
    double min_pos, max_pos;
    if (has_position_limits && node->get_parameter(param_base_name + ".min_position", min_pos) &&
        node->get_parameter(param_base_name + ".max_position", max_pos))
    {
      limits.has_position_limits = true;
      limits.min_position = min_pos;
      limits.max_position = max_pos;
    }

    bool angle_wraparound;
    if (!has_position_limits && node->get_parameter(param_base_name + ".angle_wraparound", angle_wraparound))
    {
      limits.angle_wraparound = angle_wraparound;
    }
  }

  // Velocity limits
  bool has_velocity_limits = false;
  if (node->get_parameter(param_base_name + ".has_velocity_limits", has_velocity_limits))
  {
    if (!has_velocity_limits)
    {
      limits.has_velocity_limits = false;
    }
    double max_vel;
    if (has_velocity_limits && node->get_parameter(param_base_name + ".max_velocity", max_vel))
    {
      limits.has_velocity_limits = true;
      limits.max_velocity = max_vel;
    }
  }

  // Acceleration limits
  bool has_acceleration_limits = false;
  if (node->get_parameter(param_base_name + ".has_acceleration_limits", has_acceleration_limits))
  {
    if (!has_acceleration_limits)
    {
      limits.has_acceleration_limits = false;
    }
    double max_acc;
    if (has_acceleration_limits && node->get_parameter(param_base_name + ".max_acceleration", max_acc))
    {
      limits.has_acceleration_limits = true;
      limits.max_acceleration = max_acc;
    }
  }

  // Jerk limits
  bool has_jerk_limits = false;
  if (node->get_parameter(param_base_name + ".has_jerk_limits", has_jerk_limits))
  {
    if (!has_jerk_limits)
    {
      limits.has_jerk_limits = false;
    }
    double max_jerk;
    if (has_jerk_limits && node->get_parameter(param_base_name + ".max_jerk", max_jerk))
    {
      limits.has_jerk_limits = true;
      limits.max_jerk = max_jerk;
    }
  }

  // Effort limits
  bool has_effort_limits = false;
  if (node->get_parameter(param_base_name + ".has_effort_limits", has_effort_limits))
  {
    if (!has_effort_limits)
    {
      limits.has_effort_limits = false;
    }
    double max_effort;
    if (has_effort_limits && node->get_parameter(param_base_name + ".max_effort", max_effort))
    {
      limits.has_effort_limits = true;
      limits.max_effort = max_effort;
    }
  }

  return true;
}

/// Populate a SoftJointLimits instance from the ROS parameter server.
/**
 * It is assumed that the following parameter structure is followed on the provided NodeHandle. Only completely
 * specified soft joint limits specifications will be considered valid. \code joint_limits: foo_joint: soft_lower_limit:
 * 0.0 soft_upper_limit: 1.0 k_position: 10.0 k_velocity: 10.0 \endcode
 *
 * This specification is similar to the specification of the safety_controller tag in the URDF, adapted to the parameter
 * server.
 *
 * \param[in] joint_name Name of joint whose limits are to be fetched.
 * \param[in] node NodeHandle where the joint limits are specified.
 * \param[out] soft_limits Where soft joint limit data gets written into. Limits specified in the parameter server will
 * overwrite existing values. \return True if a complete soft limits specification is found (ie. if all \p k_position,
 * \p k_velocity, \p soft_lower_limit and \p soft_upper_limit exist in \p joint_limits/joint_name namespace), false
 * otherwise.
 */
inline bool get_joint_limits(const std::string& joint_name, const rclcpp::Node::SharedPtr& node,
                             const std::string& param_ns, SoftJointLimits& soft_limits)
{
  const std::string param_base_name = (param_ns.empty() ? "" : param_ns + ".") + "joint_limits." + joint_name;
  try
  {
    if (!node->has_parameter(param_base_name + ".has_soft_limits") &&
        !node->has_parameter(param_base_name + ".k_velocity") &&
        !node->has_parameter(param_base_name + ".k_position") &&
        !node->has_parameter(param_base_name + ".soft_lower_limit") &&
        !node->has_parameter(param_base_name + ".soft_upper_limit"))
    {
      RCLCPP_DEBUG(node->get_logger(),
                   "No soft joint limits specification found for joint '%s' in the parameter server "
                   "(node: %s param name: %s).",
                   joint_name.c_str(), node->get_name(), param_base_name.c_str());
      return false;
    }
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(node->get_logger(), "%s", ex.what());
    return false;
  }

  // Override soft limits if complete specification is found
  bool has_soft_limits;
  if (node->get_parameter(param_base_name + ".has_soft_limits", has_soft_limits))
  {
    if (has_soft_limits && node->has_parameter(param_base_name + ".k_position") &&
        node->has_parameter(param_base_name + ".k_velocity") &&
        node->has_parameter(param_base_name + ".soft_lower_limit") &&
        node->has_parameter(param_base_name + ".soft_upper_limit"))
    {
      node->get_parameter(param_base_name + ".k_position", soft_limits.k_position);
      node->get_parameter(param_base_name + ".k_velocity", soft_limits.k_velocity);
      node->get_parameter(param_base_name + ".soft_lower_limit", soft_limits.min_position);
      node->get_parameter(param_base_name + ".soft_upper_limit", soft_limits.max_position);
      return true;
    }
  }

  return false;
}

}  // namespace joint_limits
