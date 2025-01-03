/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

#pragma once

#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/utils/moveit_error_code.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>
#include <moveit_msgs/msg/motion_plan_detailed_response.hpp>

namespace planning_interface
{
/// \brief Response to a planning query
struct MotionPlanResponse
{
  /** \brief Constructor */
  MotionPlanResponse() : trajectory(nullptr), planning_time(0.0), error_code(moveit::core::MoveItErrorCode::FAILURE)
  {
  }

  /** \brief Construct a ROS message from struct data */
  void getMessage(moveit_msgs::msg::MotionPlanResponse& msg) const;

  // Trajectory generated by the queried planner
  robot_trajectory::RobotTrajectoryPtr trajectory;
  // Time to plan the response to the planning query
  double planning_time;
  // Result status of the query
  moveit::core::MoveItErrorCode error_code;
  /// The full starting state used for planning
  moveit_msgs::msg::RobotState start_state;
  std::string planner_id;

  // \brief Enable checking of query success or failure, for example if(response) ...
  explicit operator bool() const
  {
    return bool(error_code);
  }
};

struct MotionPlanDetailedResponse
{
  void getMessage(moveit_msgs::msg::MotionPlanDetailedResponse& msg) const;

  std::vector<robot_trajectory::RobotTrajectoryPtr> trajectory;
  std::vector<std::string> description;
  std::vector<double> processing_time;
  moveit_msgs::msg::MoveItErrorCodes error_code;
  std::string planner_id;
};

}  // namespace planning_interface
