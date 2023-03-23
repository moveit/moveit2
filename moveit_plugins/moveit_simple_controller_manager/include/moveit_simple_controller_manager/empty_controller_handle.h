/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, PickNik Inc.
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
 *   * Neither the name of Fraunhofer IPA nor the names of its
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

/* Author: Paul Gesel */

#pragma once

#include <moveit_ros_control_interface/ControllerHandle.h>
#include <rclcpp/rclcpp.hpp>

namespace moveit_simple_controller_manager
{
/*
 * An interface for controllers that have no handle, e.g. chained controllers like an Admittance controller
 */
class EmptyControllerHandle : public moveit_controller_manager::MoveItControllerHandle
{
public:
  /* Topics will map to name/ns/goal, name/ns/result, etc */
  EmptyControllerHandle(const std::string& name, const std::string& logger_name)
    : moveit_controller_manager::MoveItControllerHandle(name), logger_(rclcpp::get_logger(logger_name))
  {
  }

  bool sendTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory) override
  {
    RCLCPP_ERROR_STREAM(logger_, "This controller handle does not support trajectory execution.");
    return false;
  }

  bool cancelExecution() override
  {
    return true;
  }

  /**
   * @brief Function called when the TrajectoryExecutionManager waits for a trajectory to finish.
   * @return Always returns true because a trajectory is never in progress.
   */
  bool waitForExecution(const rclcpp::Duration& /* timeout */) override
  {
    return true;
  }

  /**
   * @brief Gets the last trajectory execution status.
   * @return Always returns ExecutionStatus::FAILED.
   */
  moveit_controller_manager::ExecutionStatus getLastExecutionStatus() override
  {
    return moveit_controller_manager::ExecutionStatus::FAILED;
  }

private:
  const rclcpp::Logger logger_;
};

}  // end namespace moveit_simple_controller_manager
