/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
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
 *   * Neither the name of PickNik Inc. nor the names of its
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

/* Author: Sebastian Jahr
   Description: Defines an interface for a trajectory operator plugin implementation for the local planner component node.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/constraints.hpp>

#include <moveit_msgs/action/local_planner.hpp>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

namespace moveit::hybrid_planning
{
/**
 * Class TrajectoryOperatorInterface - Base class for a trajectory operator. The operator's task is manage the local
 * planner's global reference trajectory. This includes trajectory matching based on the current state to identify the
 * current planning problem and blending of new global trajectory updates into the currently processed reference
 * trajectory.
 */
class TrajectoryOperatorInterface
{
public:
  TrajectoryOperatorInterface() = default;
  TrajectoryOperatorInterface(const TrajectoryOperatorInterface&) = default;
  TrajectoryOperatorInterface(TrajectoryOperatorInterface&&) = default;
  TrajectoryOperatorInterface& operator=(const TrajectoryOperatorInterface&) = default;
  TrajectoryOperatorInterface& operator=(TrajectoryOperatorInterface&&) = default;
  virtual ~TrajectoryOperatorInterface() = default;
  /**
   * Initialize trajectory operator
   * @param node Node handle to access parameters
   * @param robot_model Robot model
   * @param group_name Name of the joint group the trajectory uses
   * @return True if initialization was successful
   */
  virtual bool initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelConstPtr& robot_model,
                          const std::string& group_name) = 0;

  /**
   * Add a new reference trajectory segment to the vector of global trajectory segments to process
   * @param new_trajectory New reference trajectory segment to add
   * @return True if segment was successfully added
   */
  virtual moveit_msgs::action::LocalPlanner::Feedback
  addTrajectorySegment(const robot_trajectory::RobotTrajectory& new_trajectory) = 0;

  /**
   * Return the current local constraints based on the newest robot state
   * @param current_state Current RobotState
   * @return Current local constraints that define the local planning goal
   */
  virtual moveit_msgs::action::LocalPlanner::Feedback
  getLocalTrajectory(const moveit::core::RobotState& current_state,
                     robot_trajectory::RobotTrajectory& local_trajectory) = 0;

  /**
   * Return the processing status of the reference trajectory's execution based on a user defined
   * metric.
   * @param current_state Current RobotState
   * @return A value between 0.0 (start) to 1.0 (completion).
   */
  virtual double getTrajectoryProgress(const moveit::core::RobotState& current_state) = 0;

  /**
   * Reset trajectory operator to some user-defined initial state
   * @return True if reset was successful
   */
  virtual bool reset() = 0;

protected:
  // Reference trajectory to be precessed
  robot_trajectory::RobotTrajectoryPtr reference_trajectory_;
  std::string group_;
};
}  // namespace moveit::hybrid_planning
