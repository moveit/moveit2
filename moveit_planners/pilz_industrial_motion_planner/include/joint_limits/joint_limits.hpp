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

#ifndef JOINT_LIMITS__JOINT_LIMITS_HPP_
#define JOINT_LIMITS__JOINT_LIMITS_HPP_

// TODO(henning): This file is copied from the DRAFT PR https://github.com/ros-controls/ros2_control/pull/462 since the
// current ros2_control implementation does not offer the desired joint limits API, yet. Remove when ros2_control has an
// upstream version of this.

#include <limits>
#include <sstream>
#include <string>

namespace joint_limits
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

  std::string to_string()  // NOLINT: Ignore case style
  {
    std::stringstream ss_output;

    if (has_position_limits)
    {
      ss_output << "  position limits: "
                << "[" << min_position << ", " << max_position << "]\n";
    }
    if (has_velocity_limits)
    {
      ss_output << "  velocity limit: "
                << "[" << max_velocity << "]\n";
    }
    if (has_acceleration_limits)
    {
      ss_output << "  acceleration limit: "
                << "[" << max_acceleration << "]\n";
    }
    if (has_jerk_limits)
    {
      ss_output << "  jerk limit: "
                << "[" << max_acceleration << "]\n";
    }
    if (has_effort_limits)
    {
      ss_output << "  effort limit: "
                << "[" << max_acceleration << "]\n";
    }
    if (angle_wraparound)
    {
      ss_output << "  angle wraparound is active.";
    }

    return ss_output.str();
  }

  std::string debug_to_string()  // NOLINT: Ignore case style
  {
    std::stringstream ss_output;

    ss_output << "  has position limits: " << (has_position_limits ? "true" : "false") << "[" << min_position << ", "
              << max_position << "]\n";
    ss_output << "  has velocity limits: " << (has_velocity_limits ? "true" : "false") << "[" << max_velocity << "]\n";
    ss_output << "  has acceleration limits: " << (has_acceleration_limits ? "true" : "false") << " ["
              << max_acceleration << "]\n";
    ss_output << "  has jerk limits: " << (has_jerk_limits ? "true" : "false") << "[" << max_jerk << "]\n";
    ss_output << "  has effort limits: " << (has_effort_limits ? "true" : "false") << "[" << max_effort << "]\n";
    ss_output << "  angle wraparound: " << (angle_wraparound ? "true" : "false");

    return ss_output.str();
  }
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

}  // namespace joint_limits

#endif  // JOINT_LIMITS__JOINT_LIMITS_HPP_
