/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik, Inc.
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
 *   * Neither the name of PickNik nor the names of its
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

/* Author: Abishalini Sivaraman */

#pragma once

#include <moveit/planning_scene_monitor/trajectory_monitor.h>
#include <rclcpp/rclcpp.hpp>

namespace planning_scene_monitor
{
/**
 * @brief      This class contains the ROS2 interfaces for TrajectoryMonitor.
 *             This class is useful for testing by mocking the functions in the class below.
 */
class TrajectoryMonitorMiddlewareHandle : public TrajectoryMonitor::MiddlewareHandle
{
public:
  /**
   * @brief      Constructor for TrajectoryMonitor
   *
   * @param[in]  sampling_frequency  Used to create ROS2 Rate
   */
  TrajectoryMonitorMiddlewareHandle(double sampling_frequency);

  /**
   * @brief    Set Rate using sampling frequency
   */
  void setRate(double sampling_frequency) override;

  /**
   * @brief      Sleeps for time specified by @p sampling_frequency
   */
  void sleep() override;

private:
  std::unique_ptr<rclcpp::Rate> rate_;
};
}  // namespace planning_scene_monitor
