/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit_msgs/action/local_planner.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace moveit_hybrid_planning
{
// The possible hybrid planner states
// TODO Use lifecycle node?
enum class LocalPlannerState : int8_t
{
  ABORT = -1,
  UNKNOWN = 0,
  READY = 1,
  AWAIT_GLOBAL_TRAJECTORY = 2,
  LOCAL_PLANNING_ACTIVE = 3
};

// Component node containing the local planner
class LocalPlannerComponent : public rclcpp::Node
{
public:
  LocalPlannerComponent(const rclcpp::NodeOptions& options);

private:
  LocalPlannerState state_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::LocalPlanner>> local_planning_goal_handle_;
  moveit_msgs::msg::RobotTrajectory global_trajectory_;

  bool global_trajectory_received_{ false };

  // Global trajectory listener
  rclcpp::Subscription<moveit_msgs::msg::MotionPlanResponse>::SharedPtr global_trajectory_sub_;

  // Local planning request action server
  rclcpp_action::Server<moveit_msgs::action::LocalPlanner>::SharedPtr local_planning_request_server_;

  // Forward trajectory publisher
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;

  // Goal callback for local planning request action server
  void localPlanningLoop();
};
}  // namespace moveit_hybrid_planning