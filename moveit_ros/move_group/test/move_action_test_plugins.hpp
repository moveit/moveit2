/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, MoveIt Contributors
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

#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>

#include <moveit/controller_manager/controller_manager.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

namespace move_group_test
{
struct PlannedBehavior
{
  bool succeed = true;
  int32_t error_code = 1;  // moveit_msgs::msg::MoveItErrorCodes::SUCCESS
  double planning_time = 0.0;
};

struct MockPlannerControl
{
  std::mutex mutex;
  std::deque<PlannedBehavior> queue;
  std::atomic<int> call_count{ 0 };

  void reset()
  {
    std::lock_guard<std::mutex> lock(mutex);
    queue.clear();
    call_count.store(0);
  }

  void push(const PlannedBehavior& b)
  {
    std::lock_guard<std::mutex> lock(mutex);
    queue.push_back(b);
  }

  PlannedBehavior popOrDefault()
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (queue.empty())
    {
      return PlannedBehavior{};
    }
    auto b = queue.front();
    queue.pop_front();
    return b;
  }
};

struct ExecutionBehavior
{
  moveit_controller_manager::ExecutionStatus status = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
  int delay_ms = 0;
  std::function<void()> on_send;
};

struct MockControllerControl
{
  std::mutex mutex;
  std::deque<ExecutionBehavior> queue;
  std::atomic<int> dispatch_count{ 0 };
  std::atomic<int> cancel_count{ 0 };
  moveit_msgs::msg::RobotTrajectory last_trajectory;

  void reset()
  {
    std::lock_guard<std::mutex> lock(mutex);
    queue.clear();
    dispatch_count.store(0);
    cancel_count.store(0);
    last_trajectory = moveit_msgs::msg::RobotTrajectory();
  }

  void push(const ExecutionBehavior& b)
  {
    std::lock_guard<std::mutex> lock(mutex);
    queue.push_back(b);
  }

  ExecutionBehavior popOrDefault()
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (queue.empty())
    {
      return ExecutionBehavior{};
    }
    auto b = queue.front();
    queue.pop_front();
    return b;
  }

  moveit_msgs::msg::RobotTrajectory getLastTrajectory()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return last_trajectory;
  }
};

MockPlannerControl& getMockPlannerControl();
MockControllerControl& getMockControllerControl();
}  // namespace move_group_test
