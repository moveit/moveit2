/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
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
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
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

#ifndef JOINT_LIMITS_INTERFACE_EXTENSION_H
#define JOINT_LIMITS_INTERFACE_EXTENSION_H

#include "pilz_industrial_motion_planner/joint_limits_extension.h"
#include <limits>

// TODO(henning): Re-include when this is available, until then the headers content is copied below
// TODO(sjahr): Current implementation does not offer the desired API (08/2021). Use draft from dstogl instead
// (https://github.com/ros-controls/ros2_control/pull/462/files) #include <joint_limits_interface/joint_limits_rosparam.hpp>
//////////////////////////////////////////////////////////////
// start of <joint_limits_interface/joint_limits_rosparam.hpp>
namespace joint_limits_interface
{
inline bool declare_parameters(const std::string& joint_name, const rclcpp::Node::SharedPtr& node)
{
  const std::string param_base_name = "joint_limits." + joint_name;
  try
  {
    node->declare_parameter<bool>(param_base_name + ".has_position_limits", false);
    node->declare_parameter<double>(param_base_name + ".min_position", std::numeric_limits<double>::quiet_NaN());
    node->declare_parameter<double>(param_base_name + ".max_position", std::numeric_limits<double>::quiet_NaN());
    node->declare_parameter<bool>(param_base_name + ".has_velocity_limits", false);
    node->declare_parameter<double>(param_base_name + ".min_velocity", std::numeric_limits<double>::quiet_NaN());
    node->declare_parameter<double>(param_base_name + ".max_velocity", std::numeric_limits<double>::quiet_NaN());
    node->declare_parameter<bool>(param_base_name + ".has_acceleration_limits", false);
    node->declare_parameter<double>(param_base_name + ".max_acceleration", std::numeric_limits<double>::quiet_NaN());
    node->declare_parameter<bool>(param_base_name + ".has_jerk_limits", false);
    node->declare_parameter<double>(param_base_name + ".max_jerk", std::numeric_limits<double>::quiet_NaN());
    node->declare_parameter<bool>(param_base_name + ".has_effort_limits", false);
    node->declare_parameter<double>(param_base_name + ".max_effort", std::numeric_limits<double>::quiet_NaN());
    node->declare_parameter<bool>(param_base_name + ".angle_wraparound", false);
    node->declare_parameter<bool>(param_base_name + ".has_soft_limits", false);
    node->declare_parameter<double>(param_base_name + ".k_position", std::numeric_limits<double>::quiet_NaN());
    node->declare_parameter<double>(param_base_name + ".k_velocity", std::numeric_limits<double>::quiet_NaN());
    node->declare_parameter<double>(param_base_name + ".soft_lower_limit", std::numeric_limits<double>::quiet_NaN());
    node->declare_parameter<double>(param_base_name + ".soft_upper_limit", std::numeric_limits<double>::quiet_NaN());
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(node->get_logger(), "%s", ex.what());
    return false;
  }
  return true;
}
//////////////////////////////////////////////////////////////
/// Populate a JointLimits instance from the ROS2 node parameters.
/**
 * It is assumed that the following parameter structure is followed on the provided NodeHandle to the node. Unspecified
 * parameters are simply not added to the joint limits specification.
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
 * \param[out] limits Where joint limit data gets written into. Limits specified in the node parameters will overwrite
 * existing values. Values in \p limits not specified in the node parameters remain unchanged.
 * \return True if a limits specification is found (ie. the \p joint_limits/joint_name parameter exists in \p node),
 * false otherwise.
 */
inline bool getJointLimits(const std::string& joint_name, const rclcpp::Node::SharedPtr& node, JointLimits& limits)
{
  const std::string param_base_name = "joint_limits." + joint_name;
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
      RCLCPP_ERROR_STREAM(node->get_logger(), "No joint limits specification found for joint '"
                                                  << joint_name << "' in the node parameter (node: "
                                                  << std::string(node->get_name()) + " param name: " + param_base_name
                                                  << ").");
      return false;
    }
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), ex.what());
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

/// Populate a SoftJointLimits instance from the ROS2 node parameters.
/**
 * It is assumed that the following parameter structure is followed on the provided NodeHandle. Only completely
 * specified soft joint limits specifications will be considered valid. \code joint_limits: foo_joint: soft_lower_limit:
 * 0.0 soft_upper_limit: 1.0 k_position: 10.0 k_velocity: 10.0 \endcode
 *
 * This specification is similar to the specification of the safety_controller tag in the URDF, adapted to the node paramters.
 *
 * \param[in] joint_name Name of joint whose limits are to be fetched.
 * \param[in] node NodeHandle where the joint limits are specified.
 * \param[out] soft_limits Where soft joint limit data gets written into. Limits specified in the node parameters will
 * overwrite existing values. \return True if a complete soft limits specification is found (ie. if all \p k_position,
 * \p k_velocity, \p soft_lower_limit and \p soft_upper_limit exist in \p joint_limits/joint_name namespace), false
 * otherwise.
 */
inline bool getSoftJointLimits(const std::string& joint_name, const rclcpp::Node::SharedPtr& node,
                               SoftJointLimits& soft_limits)
{
  const std::string param_base_name = "joint_limits." + joint_name;
  try
  {
    if (!node->has_parameter(param_base_name + ".has_soft_limits") &&
        !node->has_parameter(param_base_name + ".k_velocity") &&
        !node->has_parameter(param_base_name + ".k_position") &&
        !node->has_parameter(param_base_name + ".soft_lower_limit") &&
        !node->has_parameter(param_base_name + ".soft_upper_limit"))
    {
      RCLCPP_DEBUG_STREAM(node->get_logger(), "No soft joint limits specification found for joint '"
                                                  << joint_name << "' in the node parameter (node: "
                                                  << std::string(node->get_name()) + " param name: " + param_base_name
                                                  << ").");
      return false;
    }
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), ex.what());
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
}  // namespace joint_limits_interface
//////////////////////////////////////////////////////////////
// end of <joint_limits_interface/joint_limits_rosparam.hpp>
//////////////////////////////////////////////////////////////

namespace pilz_industrial_motion_planner
{
namespace joint_limits_interface
{
/**
 * @see joint_limits_inteface::getJointLimits(...)
 */
inline bool getJointLimits(const std::string& joint_name, const rclcpp::Node::SharedPtr& node,
                           joint_limits_interface::JointLimits& limits)
{
  // Set the existing limits
  if (!::joint_limits_interface::getJointLimits(joint_name, node, limits))
  {
    return false;  // LCOV_EXCL_LINE // The case where getJointLimits returns
                   // false is covered above.
  }
  try
  {
    // Deceleration limits
    const std::string limits_namespace = "joint_limits." + joint_name;
    limits.has_deceleration_limits = node->declare_parameter(limits_namespace + ".has_deceleration_limits", false);
    if (limits.has_deceleration_limits)
    {
      limits.max_deceleration =
          node->declare_parameter(limits_namespace + ".max_deceleration", std::numeric_limits<double>::quiet_NaN());
    }
  }
  catch (const std::exception& ex)
  {
    RCLCPP_WARN_STREAM(node->get_logger(), "Failed loading deceleration limits");
    limits.has_deceleration_limits = false;
  }

  return true;
}
}  // namespace joint_limits_interface
}  // namespace pilz_industrial_motion_planner

#endif  // JOINT_LIMITS_INTERFACE_EXTENSION_H
