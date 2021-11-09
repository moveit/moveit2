/*******************************************************************************
 * Title     : collision_check.h
 * Project   : moveit_servo
 * Created   : 1/11/2019
 * Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#pragma once

#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include <moveit_servo/servo_parameters.h>

namespace moveit_servo
{
class CollisionCheck
{
public:
  /** \brief Constructor
   *  \param parameters: common settings of moveit_servo
   *  \param planning_scene_monitor: PSM should have scene monitor and state monitor
   *                                 already started when passed into this class
   */
  CollisionCheck(rclcpp::Node::SharedPtr node, const ServoParameters::SharedConstPtr& parameters,
                 const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);

  ~CollisionCheck()
  {
    if (timer_)
    {
      timer_->cancel();
    }
  }

  /** \brief start the Timer that regulates collision check rate */
  void start();

  /** \brief Pause or unpause processing servo commands while keeping the timers alive */
  void setPaused(bool paused);

private:
  /** \brief Run one iteration of collision checking */
  void run();

  /** \brief Get a read-only copy of the planning scene */
  planning_scene_monitor::LockedPlanningSceneRO getLockedPlanningSceneRO() const;

  /** \brief Callback for collision stopping time, from the thread that is aware of velocity and acceleration */
  void worstCaseStopTimeCB(const std_msgs::msg::Float64::SharedPtr msg);

  // Pointer to the ROS node
  const std::shared_ptr<rclcpp::Node> node_;

  // Parameters from yaml
  const ServoParameters::SharedConstPtr parameters_;

  // Pointer to the collision environment
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Robot state and collision matrix from planning scene
  std::shared_ptr<moveit::core::RobotState> current_state_;

  // Scale robot velocity according to collision proximity and user-defined thresholds.
  // I scaled exponentially (cubic power) so velocity drops off quickly after the threshold.
  // Proximity decreasing --> decelerate
  double velocity_scale_ = 1;
  double self_collision_distance_ = 0;
  double scene_collision_distance_ = 0;
  bool collision_detected_ = false;
  bool paused_ = false;

  // Variables for stop-distance-based collision checking
  double worst_case_stop_time_ = std::numeric_limits<double>::max();

  const double self_velocity_scale_coefficient_;
  const double scene_velocity_scale_coefficient_;

  // collision request
  collision_detection::CollisionRequest collision_request_;
  collision_detection::CollisionResult collision_result_;

  // ROS
  rclcpp::TimerBase::SharedPtr timer_;
  double period_;  // The loop period, in seconds
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr collision_velocity_scale_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr worst_case_stop_time_sub_;

  mutable std::mutex joint_state_mutex_;
  sensor_msgs::msg::JointState latest_joint_state_;
};
}  // namespace moveit_servo
