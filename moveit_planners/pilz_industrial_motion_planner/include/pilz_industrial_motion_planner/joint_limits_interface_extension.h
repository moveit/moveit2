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

#pragma once

#include <pilz_industrial_motion_planner/joint_limits_extension.h>
#include <limits>

#include <joint_limits_copy/joint_limits_rosparam.hpp>

namespace pilz_industrial_motion_planner
{
namespace joint_limits_interface
{
/**
 * @see joint_limits::declareParameters(...)
 */
inline bool declareParameters(const std::string& joint_name, const std::string& param_ns,
                              const rclcpp::Node::SharedPtr& node)
{
  return joint_limits::declareParameters(joint_name, node, param_ns);
}
/**
 * @see joint_limits::getJointLimits(...)
 */
inline bool getJointLimits(const std::string& joint_name, const std::string& param_ns,
                           const rclcpp::Node::SharedPtr& node, joint_limits_interface::JointLimits& limits)
{
  // Load the existing limits
  if (!joint_limits::getJointLimits(joint_name, node, param_ns, limits))
  {
    return false;  // LCOV_EXCL_LINE // The case where getJointLimits returns
                   // false is covered above.
  }
  try
  {
    // Deceleration limits
    const std::string param_base_name = (param_ns.empty() ? "" : param_ns + ".") + "joint_limits." + joint_name;
    if (node->has_parameter(param_base_name + ".has_deceleration_limits"))
    {
      limits.has_deceleration_limits = node->get_parameter(param_base_name + ".has_deceleration_limits").as_bool();
    }
    else
    {
      limits.has_deceleration_limits = node->declare_parameter(param_base_name + ".has_deceleration_limits", false);
    }
    if (limits.has_deceleration_limits)
    {
      if (node->has_parameter(param_base_name + ".max_deceleration"))
      {
        limits.max_deceleration = node->get_parameter(param_base_name + ".max_deceleration").as_double();
      }
      else
      {
        limits.max_deceleration =
            node->declare_parameter(param_base_name + ".max_deceleration", std::numeric_limits<double>::quiet_NaN());
      }
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
