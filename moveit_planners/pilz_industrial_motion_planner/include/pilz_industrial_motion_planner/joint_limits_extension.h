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

#ifndef JOINT_LIMITS_EXTENSION_H
#define JOINT_LIMITS_EXTENSION_H

#include <limits>

// TODO(henning): Re-include when this is available, until then the headers content is copied below
// TODO(sjahr): Current implementation does not offer the desired API (08/2021). Use draft from dstogl instead
// (https://github.com/ros-controls/ros2_control/pull/462/files) #include <joint_limits_interface/joint_limits.h>
/////////////////////////////////////////////////////
// start of <joint_limits_interface/joint_limits.hpp>
// //////////////////////////////////////////////////
namespace joint_limits_interface
{
struct JointLimits
{
  JointLimits()
    : min_position(std::numeric_limits<double>::quiet_NaN())
    , max_position(std::numeric_limits<double>::quiet_NaN())
    , max_velocity(std::numeric_limits<double>::quiet_NaN())
    , max_acceleration(std::numeric_limits<double>::quiet_NaN())
    , max_jerk(std::numeric_limits<double>::quiet_NaN())
    , max_effort(std::numeric_limits<double>::quiet_NaN())
    , has_position_limits(false)
    , has_velocity_limits(false)
    , has_acceleration_limits(false)
    , has_jerk_limits(false)
    , has_effort_limits(false)
    , angle_wraparound(false)
  {
  }

  double min_position;
  double max_position;
  double max_velocity;
  double max_acceleration;
  double max_jerk;
  double max_effort;

  bool has_position_limits;
  bool has_velocity_limits;
  bool has_acceleration_limits;
  bool has_jerk_limits;
  bool has_effort_limits;
  bool angle_wraparound;
};

struct SoftJointLimits
{
  SoftJointLimits()
    : min_position(std::numeric_limits<double>::quiet_NaN())
    , max_position(std::numeric_limits<double>::quiet_NaN())
    , k_position(std::numeric_limits<double>::quiet_NaN())
    , k_velocity(std::numeric_limits<double>::quiet_NaN())
  {
  }

  double min_position;
  double max_position;
  double k_position;
  double k_velocity;
};
}  // namespace joint_limits_interface
///////////////////////////////////////////////////
// end of <joint_limits_interface/joint_limits.hpp>
// ////////////////////////////////////////////////

#include <map>
#include <string>

namespace pilz_industrial_motion_planner
{
namespace joint_limits_interface
{
/**
 * @brief Extends joint_limits_interface::JointLimits with a deceleration
 * parameter
 */
struct JointLimits : ::joint_limits_interface::JointLimits
{
  JointLimits() : max_deceleration(0.0), has_deceleration_limits(false)
  {
  }

  /// maximum deceleration MUST(!) be negative
  double max_deceleration;

  bool has_deceleration_limits;
};
}  // namespace joint_limits_interface

typedef joint_limits_interface::JointLimits JointLimit;
typedef std::map<std::string, JointLimit> JointLimitsMap;
}  // namespace pilz_industrial_motion_planner

#endif  // JOINT_LIMITS_EXTENSION_H
