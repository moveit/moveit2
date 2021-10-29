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

#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <tf2_ros/buffer.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

struct MockMiddlewareHandle : public occupancy_map_monitor::OccupancyMapMonitor::MiddlewareHandle
{
  MOCK_METHOD(occupancy_map_monitor::OccupancyMapMonitor::Parameters, getParameters, (), (const, override));
  MOCK_METHOD(occupancy_map_monitor::OccupancyMapUpdaterPtr, loadOccupancyMapUpdater,
              (const std::string& sensor_plugin), (override));
  MOCK_METHOD(void, initializeOccupancyMapUpdater,
              (occupancy_map_monitor::OccupancyMapUpdaterPtr occupancy_map_updater), (override));
  MOCK_METHOD(void, createSaveMapService,
              (occupancy_map_monitor::OccupancyMapMonitor::MiddlewareHandle::SaveMapServiceCallback callback),
              (override));
  MOCK_METHOD(void, createLoadMapService,
              (occupancy_map_monitor::OccupancyMapMonitor::MiddlewareHandle::LoadMapServiceCallback callback),
              (override));
};

TEST(OccupancyMapMonitorTests, ConstructorTest)
{
  // GIVEN a mocked middleware handle
  auto mock_middleware_handle = std::make_unique<MockMiddlewareHandle>();

  // THEN we expect it to call these methods on the MiddlewareHandle
  EXPECT_CALL(*mock_middleware_handle, getParameters).Times(1);
  EXPECT_CALL(*mock_middleware_handle, createSaveMapService).Times(1);
  EXPECT_CALL(*mock_middleware_handle, createLoadMapService).Times(1);
  EXPECT_CALL(*mock_middleware_handle, loadOccupancyMapUpdater).Times(0);

  // WHEN we construct the occupancy map monitor
  occupancy_map_monitor::OccupancyMapMonitor occupancy_map_monitor{
    std::move(mock_middleware_handle), std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>())
  };
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
