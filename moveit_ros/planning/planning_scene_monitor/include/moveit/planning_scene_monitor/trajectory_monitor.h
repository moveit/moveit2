/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#include <moveit/macros/class_forward.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <rclcpp/time.hpp>
#include <memory>
#include <functional>
#include <thread>

namespace planning_scene_monitor
{
using TrajectoryStateAddedCallback = std::function<void(const moveit::core::RobotStateConstPtr&, const rclcpp::Time&)>;

MOVEIT_CLASS_FORWARD(TrajectoryMonitor);  // Defines TrajectoryMonitorPtr, ConstPtr, WeakPtr... etc

/** @class TrajectoryMonitor
    @brief Monitors the joint_states topic and tf to record the trajectory of the robot. */
class TrajectoryMonitor
{
public:
  /**
   * @brief      This class contains the rcl interfaces for easier testing
   */
  class MiddlewareHandle
  {
  public:
    /**
     * @brief      Destroys the object.
     */
    virtual ~MiddlewareHandle() = default;

    /**
     * @brief      set Rate using sampling frequency
     */
    virtual void setRate(double sampling_frequency) = 0;

    /**
     * @brief      Sleep the handle for some prescribed amount of time.
     */
    virtual void sleep() = 0;
  };

  /** @brief Constructor
   *  @param[in]  state_monitor
   *  @param[in]  sampling_frequency
   */
  TrajectoryMonitor(const CurrentStateMonitorConstPtr& state_monitor, double sampling_frequency = 0.0);

  /** @brief Constructor with middleware handle as input parameter
   *  @param[in]  state_monitor
   *  @param[in]  sampling_frequency
   */
  TrajectoryMonitor(const CurrentStateMonitorConstPtr& state_monitor,
                    std::unique_ptr<MiddlewareHandle> middleware_handle, double sampling_frequency = 0.0);

  ~TrajectoryMonitor();

  void startTrajectoryMonitor();

  void stopTrajectoryMonitor();

  void clearTrajectory();

  bool isActive() const;

  double getSamplingFrequency() const
  {
    return sampling_frequency_;
  }

  void setSamplingFrequency(double sampling_frequency);

  /// Return the current maintained trajectory. This function is not thread safe (hence NOT const), because the
  /// trajectory could be modified.
  const robot_trajectory::RobotTrajectory& getTrajectory()
  {
    return trajectory_;
  }

  void swapTrajectory(robot_trajectory::RobotTrajectory& other)
  {
    trajectory_.swap(other);
  }

  void setOnStateAddCallback(const TrajectoryStateAddedCallback& callback)
  {
    state_add_callback_ = callback;
  }

private:
  void recordStates();

  // Samples robot states.
  CurrentStateMonitorConstPtr current_state_monitor_;
  // Interface for communicating with ROS.
  std::unique_ptr<MiddlewareHandle> middleware_handle_;
  double sampling_frequency_;

  robot_trajectory::RobotTrajectory trajectory_;
  rclcpp::Time trajectory_start_time_;
  rclcpp::Time last_recorded_state_time_;

  std::unique_ptr<std::thread> record_states_thread_;
  TrajectoryStateAddedCallback state_add_callback_;
};
}  // namespace planning_scene_monitor
