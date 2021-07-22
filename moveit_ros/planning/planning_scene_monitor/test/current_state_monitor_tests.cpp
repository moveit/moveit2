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

/* Author: Tyler Weaver */

#include <memory>
#include <string>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "moveit/planning_scene_monitor/current_state_monitor.h"
#include "moveit/utils/robot_model_test_utils.h"
#include "rclcpp/node_interfaces/node_clock_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

struct MockRclInterface : public planning_scene_monitor::CurrentStateMonitor::RclInterface
{
  MOCK_METHOD(rclcpp::Time, now, (), (const, override));
  MOCK_METHOD(void, createJointStateSubscription,
              (const std::string& topic, planning_scene_monitor::JointStateUpdateCallback callback), (override));
  MOCK_METHOD(void, resetJointStateSubscription, (), (override));
  MOCK_METHOD(std::string, getJointStateTopicName, (), (const, override));
};

TEST(StartStopTest, CurrentStateMonitorTests)
{
  auto mock_rcl_interface = std::make_unique<MockRclInterface>();
  EXPECT_CALL(*mock_rcl_interface, createJointStateSubscription);
  EXPECT_CALL(*mock_rcl_interface, resetJointStateSubscription);
  EXPECT_CALL(*mock_rcl_interface, now);

  planning_scene_monitor::CurrentStateMonitor current_state_monitor{
    std::move(mock_rcl_interface), moveit::core::loadTestingRobotModel("panda"),
    std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>())
  };

  current_state_monitor.startStateMonitor();

  EXPECT_TRUE(current_state_monitor.isActive());

  current_state_monitor.stopStateMonitor();

  EXPECT_FALSE(current_state_monitor.isActive());
}

TEST(NoModelTest, CurrentStateMonitorTests)
{
  EXPECT_THROW(planning_scene_monitor::CurrentStateMonitor _(
                   std::make_unique<MockRclInterface>(), nullptr,
                   std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>())),
               std::invalid_argument);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
