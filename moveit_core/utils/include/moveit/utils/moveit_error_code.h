/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik Inc.
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
 *   * Neither the name of the copyright holder nor the names of its
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

#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <string>

namespace moveit
{
namespace core
{
/**
 * @brief a wrapper around moveit_msgs::MoveItErrorCodes to make it easier to return an error code message from a function
 */
class MoveItErrorCode : public moveit_msgs::msg::MoveItErrorCodes
{
public:
  MoveItErrorCode(int code = 0)
  {
    val = code;
  }
  MoveItErrorCode(const moveit_msgs::msg::MoveItErrorCodes& code)
  {
    val = code.val;
  }
  explicit operator bool() const
  {
    return val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }
  bool operator==(const int c) const
  {
    return val == c;
  }
  bool operator!=(const int c) const
  {
    return val != c;
  }
};

/**
 * @brief Convenience function to translated error message into string
   @param error_code Error code to be translated to a string
   @return Error code string
 */
inline std::string error_code_to_string(MoveItErrorCode error_code)
{
  switch (error_code.val)
  {
    case moveit::core::MoveItErrorCode::SUCCESS:
      return std::string("SUCCESS");
    case moveit::core::MoveItErrorCode::FAILURE:
      return std::string("FAILURE");
    case moveit::core::MoveItErrorCode::PLANNING_FAILED:
      return std::string("PLANNING_FAILED");
    case moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN:
      return std::string("INVALID_MOTION_PLAN");
    case moveit::core::MoveItErrorCode::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
      return std::string("MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE");
    case moveit::core::MoveItErrorCode::CONTROL_FAILED:
      return std::string("CONTROL_FAILED");
    case moveit::core::MoveItErrorCode::UNABLE_TO_AQUIRE_SENSOR_DATA:
      return std::string("UNABLE_TO_AQUIRE_SENSOR_DATA");
    case moveit::core::MoveItErrorCode::TIMED_OUT:
      return std::string("TIMED_OUT");
    case moveit::core::MoveItErrorCode::PREEMPTED:
      return std::string("PREEMPTED");
    case moveit::core::MoveItErrorCode::START_STATE_IN_COLLISION:
      return std::string("START_STATE_IN_COLLISION");
    case moveit::core::MoveItErrorCode::START_STATE_VIOLATES_PATH_CONSTRAINTS:
      return std::string("START_STATE_VIOLATES_PATH_CONSTRAINTS");
    case moveit::core::MoveItErrorCode::START_STATE_INVALID:
      return std::string("START_STATE_INVALID");
    case moveit::core::MoveItErrorCode::GOAL_IN_COLLISION:
      return std::string("GOAL_IN_COLLISION");
    case moveit::core::MoveItErrorCode::GOAL_VIOLATES_PATH_CONSTRAINTS:
      return std::string("GOAL_VIOLATES_PATH_CONSTRAINTS");
    case moveit::core::MoveItErrorCode::GOAL_CONSTRAINTS_VIOLATED:
      return std::string("GOAL_CONSTRAINTS_VIOLATED");
    case moveit::core::MoveItErrorCode::GOAL_STATE_INVALID:
      return std::string("GOAL_STATE_INVALID");
    case moveit::core::MoveItErrorCode::UNRECOGNIZED_GOAL_TYPE:
      return std::string("UNRECOGNIZED_GOAL_TYPE");
    case moveit::core::MoveItErrorCode::INVALID_GROUP_NAME:
      return std::string("INVALID_GROUP_NAME");
    case moveit::core::MoveItErrorCode::INVALID_GOAL_CONSTRAINTS:
      return std::string("INVALID_GOAL_CONSTRAINTS");
    case moveit::core::MoveItErrorCode::INVALID_ROBOT_STATE:
      return std::string("INVALID_ROBOT_STATE");
    case moveit::core::MoveItErrorCode::INVALID_LINK_NAME:
      return std::string("INVALID_LINK_NAME");
    case moveit::core::MoveItErrorCode::INVALID_OBJECT_NAME:
      return std::string("INVALID_OBJECT_NAME");
    case moveit::core::MoveItErrorCode::FRAME_TRANSFORM_FAILURE:
      return std::string("FRAME_TRANSFORM_FAILURE");
    case moveit::core::MoveItErrorCode::COLLISION_CHECKING_UNAVAILABLE:
      return std::string("COLLISION_CHECKING_UNAVAILABLE");
    case moveit::core::MoveItErrorCode::ROBOT_STATE_STALE:
      return std::string("ROBOT_STATE_STALE");
    case moveit::core::MoveItErrorCode::SENSOR_INFO_STALE:
      return std::string("SENSOR_INFO_STALE");
    case moveit::core::MoveItErrorCode::COMMUNICATION_FAILURE:
      return std::string("COMMUNICATION_FAILURE");
    case moveit::core::MoveItErrorCode::CRASH:
      return std::string("CRASH");
    case moveit::core::MoveItErrorCode::ABORT:
      return std::string("ABORT");
    case moveit::core::MoveItErrorCode::NO_IK_SOLUTION:
      return std::string("NO_IK_SOLUTION");
  }
  return std::string("Unrecognized MoveItErrorCode. This should never happen!");
}

}  // namespace core
}  // namespace moveit
