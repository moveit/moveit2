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

#include <memory>
#include <chrono>
#include <thread>
#include <atomic>
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "moveit/planning_scene_monitor/trajectory_monitor.h"
#include "moveit/planning_scene_monitor/current_state_monitor.h"
#include "moveit/utils/robot_model_test_utils.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

struct MockTrajectoryMonitorMiddlewareHandle : public planning_scene_monitor::TrajectoryMonitor::MiddlewareHandle
{
  MOCK_METHOD(void, setRate, (double sampling_frequency), (override));
  MOCK_METHOD(void, sleep, (), (override));
};

struct MockCurrentStateMonitorMiddlewareHandle : public planning_scene_monitor::CurrentStateMonitor::MiddlewareHandle
{
  MOCK_METHOD(rclcpp::Time, now, (), (const, override));
  MOCK_METHOD(void, createJointStateSubscription,
              (const std::string& topic, planning_scene_monitor::JointStateUpdateCallback callback), (override));
  MOCK_METHOD(void, resetJointStateSubscription, (), (override));
  MOCK_METHOD(std::string, getJointStateTopicName, (), (const, override));
  MOCK_METHOD(bool, sleepFor, (const std::chrono::nanoseconds& nanoseconds), (const, override));
  MOCK_METHOD(void, createStaticTfSubscription, (TfCallback callback), (override));
  MOCK_METHOD(void, createDynamicTfSubscription, (TfCallback callback), (override));
  MOCK_METHOD(std::string, getStaticTfTopicName, (), (const, override));
  MOCK_METHOD(std::string, getDynamicTfTopicName, (), (const, override));
  MOCK_METHOD(void, resetTfSubscriptions, (), (override));
};

void waitFor(std::chrono::seconds timeout, std::function<bool()> done)
{
  const auto start = std::chrono::steady_clock::now();
  while ((std::chrono::steady_clock::now() - start) < timeout)
  {
    if (done())
    {
      return;
    }
    std::this_thread::sleep_for(1ms);
  }
}

TEST(TrajectoryMonitorTests, SleepAtLeastOnce)
{
  auto mock_trajectory_monitor_middleware_handle = std::make_unique<MockTrajectoryMonitorMiddlewareHandle>();
  auto mock_current_state_monitor_middleware_handle = std::make_unique<MockCurrentStateMonitorMiddlewareHandle>();

  // THEN we expect it to call sleep function
  EXPECT_CALL(*mock_trajectory_monitor_middleware_handle, sleep).Times(::testing::AtLeast(1));

  // GIVEN a TrajectoryMonitor is started
  auto current_state_monitor = std::make_shared<planning_scene_monitor::CurrentStateMonitor>(
      std::move(mock_current_state_monitor_middleware_handle), moveit::core::loadTestingRobotModel("panda"),
      std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>()), false);

  planning_scene_monitor::TrajectoryMonitor trajectory_monitor{ current_state_monitor,
                                                                std::move(mock_trajectory_monitor_middleware_handle),
                                                                10.0 };

  // Set the trajectory monitor callback so we can block until at least one sample was taken.
  std::atomic<bool> callback_called{ false };
  trajectory_monitor.setOnStateAddCallback([&](const moveit::core::RobotStateConstPtr& /* unused */,
                                               const rclcpp::Time& /* unused */) { callback_called = true; });

  // WHEN we call the startTrajectoryMonitor function
  trajectory_monitor.startTrajectoryMonitor();
  waitFor(10s, [&]() { return static_cast<bool>(callback_called); });
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
