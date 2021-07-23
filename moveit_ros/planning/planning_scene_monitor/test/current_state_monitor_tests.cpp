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

struct MockMiddlewareHandle : public planning_scene_monitor::CurrentStateMonitor::MiddlewareHandle
{
  MOCK_METHOD(rclcpp::Time, now, (), (const, override));
  MOCK_METHOD(void, createJointStateSubscription,
              (const std::string& topic, planning_scene_monitor::JointStateUpdateCallback callback), (override));
  MOCK_METHOD(void, resetJointStateSubscription, (), (override));
  MOCK_METHOD(std::string, getJointStateTopicName, (), (const, override));
};

TEST(CurrentStateMonitorTests, StartCreateSubscriptionTest)
{
  auto mock_middleware_handle = std::make_unique<MockMiddlewareHandle>();
  // THEN we expect it to call createJointStateSubscription on the MiddlewareHandle
  EXPECT_CALL(*mock_middleware_handle, createJointStateSubscription);

  // GIVEN a CurrentStateMonitor
  planning_scene_monitor::CurrentStateMonitor current_state_monitor{
    std::move(mock_middleware_handle), moveit::core::loadTestingRobotModel("panda"),
    std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>())
  };

  // WHEN we start the current state monitor
  current_state_monitor.startStateMonitor();
}

TEST(CurrentStateMonitorTests, StartActiveTest)
{
  // GIVEN a CurrentStateMonitor
  planning_scene_monitor::CurrentStateMonitor current_state_monitor{
    std::make_unique<MockMiddlewareHandle>(), moveit::core::loadTestingRobotModel("panda"),
    std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>())
  };

  // WHEN we start the current state monitor
  current_state_monitor.startStateMonitor();

  // THEN we expect it to be active
  EXPECT_TRUE(current_state_monitor.isActive());
}

TEST(CurrentStateMonitorTests, StopResetSubscriptionTest)
{
  auto mock_middleware_handle = std::make_unique<MockMiddlewareHandle>();

  // THEN we expect it to call now and resetJointStateSubscription on the MiddlewareHandle
  EXPECT_CALL(*mock_middleware_handle, resetJointStateSubscription);
  EXPECT_CALL(*mock_middleware_handle, now);

  // GIVEN a CurrentStateMonitor that is started
  planning_scene_monitor::CurrentStateMonitor current_state_monitor{
    std::move(mock_middleware_handle), moveit::core::loadTestingRobotModel("panda"),
    std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>())
  };
  current_state_monitor.startStateMonitor();

  // WHEN we stop the current state monitor
  current_state_monitor.stopStateMonitor();
}

TEST(CurrentStateMonitorTests, StopNotActiveTest)
{
  auto mock_middleware_handle = std::make_unique<MockMiddlewareHandle>();

  // GIVEN a CurrentStateMonitor that is started
  planning_scene_monitor::CurrentStateMonitor current_state_monitor{
    std::move(mock_middleware_handle), moveit::core::loadTestingRobotModel("panda"),
    std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>())
  };
  current_state_monitor.startStateMonitor();

  // WHEN we stop the current state monitor
  current_state_monitor.stopStateMonitor();

  // THEN we expect it to not be active
  EXPECT_FALSE(current_state_monitor.isActive());
}

TEST(CurrentStateMonitorTests, DestructStopTest)
{
  auto mock_middleware_handle = std::make_unique<MockMiddlewareHandle>();

  // THEN we expect it to be stopped and resetJointStateSubscription and now to be called
  EXPECT_CALL(*mock_middleware_handle, resetJointStateSubscription);
  EXPECT_CALL(*mock_middleware_handle, now);

  // GIVEN a CurrentStateMonitor that we started
  {
    planning_scene_monitor::CurrentStateMonitor current_state_monitor{
      std::move(mock_middleware_handle), moveit::core::loadTestingRobotModel("panda"),
      std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>())
    };
    current_state_monitor.startStateMonitor();
    EXPECT_TRUE(current_state_monitor.isActive());
  }
  // WHEN it is  destructed
}

TEST(CurrentStateMonitorTests, NoModelTest)
{
  // GIVEN an unitialized robot model
  moveit::core::RobotModelPtr robot_model = nullptr;

  // WHEN the CurrentStateMonitor is constructed with it
  // THEN we expect the monitor to throw because of the invalid model
  EXPECT_THROW(planning_scene_monitor::CurrentStateMonitor _(
                   std::make_unique<MockMiddlewareHandle>(), robot_model,
                   std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>())),
               std::invalid_argument);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
