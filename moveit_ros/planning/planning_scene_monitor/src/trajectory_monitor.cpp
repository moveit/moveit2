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

#include <moveit/planning_scene_monitor/trajectory_monitor.h>
#include <moveit/planning_scene_monitor/trajectory_monitor_middleware_handle.hpp>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <limits>
#include <memory>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_ros.planning_scene_monitor.trajectory_monitor");

planning_scene_monitor::TrajectoryMonitor::TrajectoryMonitor(const CurrentStateMonitorConstPtr& state_monitor,
                                                             double sampling_frequency)
  : TrajectoryMonitor(state_monitor, std::make_unique<TrajectoryMonitorMiddlewareHandle>(sampling_frequency),
                      sampling_frequency)
{
}

planning_scene_monitor::TrajectoryMonitor::TrajectoryMonitor(
    const CurrentStateMonitorConstPtr& state_monitor,
    std::unique_ptr<TrajectoryMonitor::MiddlewareHandle> middleware_handle, double sampling_frequency)
  : current_state_monitor_(state_monitor)
  , middleware_handle_(std::move(middleware_handle))
  , sampling_frequency_(sampling_frequency)
  , trajectory_(current_state_monitor_->getRobotModel(), "")
{
  setSamplingFrequency(sampling_frequency);
}

planning_scene_monitor::TrajectoryMonitor::~TrajectoryMonitor()
{
  stopTrajectoryMonitor();
}

void planning_scene_monitor::TrajectoryMonitor::setSamplingFrequency(double sampling_frequency)
{
  if (sampling_frequency == sampling_frequency_)
    return;  // silently return if nothing changes

  if (sampling_frequency <= std::numeric_limits<double>::epsilon())
    RCLCPP_ERROR(LOGGER, "The sampling frequency for trajectory states should be positive");
  else
    RCLCPP_DEBUG(LOGGER, "Setting trajectory sampling frequency to %.1f", sampling_frequency);
  sampling_frequency_ = sampling_frequency;
}

bool planning_scene_monitor::TrajectoryMonitor::isActive() const
{
  return static_cast<bool>(record_states_thread_);
}

void planning_scene_monitor::TrajectoryMonitor::startTrajectoryMonitor()
{
  if (sampling_frequency_ > std::numeric_limits<double>::epsilon() && !record_states_thread_)
  {
    record_states_thread_ = std::make_unique<std::thread>([this] { recordStates(); });
    RCLCPP_DEBUG(LOGGER, "Started trajectory monitor");
  }
}

void planning_scene_monitor::TrajectoryMonitor::stopTrajectoryMonitor()
{
  if (record_states_thread_)
  {
    std::unique_ptr<std::thread> copy;
    copy.swap(record_states_thread_);
    copy->join();
    RCLCPP_DEBUG(LOGGER, "Stopped trajectory monitor");
  }
}

void planning_scene_monitor::TrajectoryMonitor::clearTrajectory()
{
  bool restart = isActive();
  if (restart)
    stopTrajectoryMonitor();
  trajectory_.clear();
  if (restart)
    startTrajectoryMonitor();
}

void planning_scene_monitor::TrajectoryMonitor::recordStates()
{
  if (!current_state_monitor_)
    return;

  middleware_handle_->setRate(sampling_frequency_);

  while (record_states_thread_)
  {
    middleware_handle_->sleep();
    std::pair<moveit::core::RobotStatePtr, rclcpp::Time> state = current_state_monitor_->getCurrentStateAndTime();
    if (trajectory_.empty())
    {
      trajectory_.addSuffixWayPoint(state.first, 0.0);
      trajectory_start_time_ = state.second;
      last_recorded_state_time_ = state.second;
    }
    else
    {
      trajectory_.addSuffixWayPoint(state.first, (state.second - last_recorded_state_time_).seconds());
      last_recorded_state_time_ = state.second;
    }
    if (state_add_callback_)
      state_add_callback_(state.first, state.second);
  }
}
