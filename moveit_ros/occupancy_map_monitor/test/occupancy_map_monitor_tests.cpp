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

#include <moveit/occupancy_map_monitor/occupancy_map_monitor.hpp>
#include <moveit/collision_detection/occupancy_map.hpp>
#include <octomap/octomap.h>
#include <Eigen/Geometry>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <rclcpp/version.h>
// For Rolling, Kilted, and newer
#if RCLCPP_VERSION_GTE(29, 6, 0)
#include <tf2_ros/buffer.hpp>
// For Jazzy and older
#else
#include <tf2_ros/buffer.h>
#endif

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

/**
 * @class OctomapVoxelClearingTests
 * @brief Test fixtures to validate the instantaneous clearing of Octomap voxels
 *        when a collision object is added to the environment.
 */
class OctomapVoxelClearingTests : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // dummy ROS 2 node for the monitor
    auto node = std::make_shared<rclcpp::Node>("occupancy_map_monitor_test_node");
    monitor_ = std::make_unique<occupancy_map_monitor::OccupancyMapMonitor>(node, 0.05);
  }

  void populateRegion(const octomap::point3d& min, const octomap::point3d& max)
  {
    auto tree = monitor_->getOcTreePtr();
    double res = tree->getResolution();
    for (double x = min.x(); x <= max.x(); x += res)
      for (double y = min.y(); y <= max.y(); y += res)
        for (double z = min.z(); z <= max.z(); z += res)
          tree->updateNode(octomap::point3d(x, y, z), true);

    tree->updateInnerOccupancy();
  }

  void verifyVolumeIsClear(const collision_detection::OccMapTreePtr& tree, const Eigen::Vector3d& center, double range)
  {
    tree->lockRead();
    std::vector<Eigen::Vector3d> points = { center,
                                            center + Eigen::Vector3d(range, 0, 0),
                                            center - Eigen::Vector3d(range, 0, 0),
                                            center + Eigen::Vector3d(0, range, 0),
                                            center - Eigen::Vector3d(0, range, 0),
                                            center + Eigen::Vector3d(0, 0, range),
                                            center - Eigen::Vector3d(0, 0, range) };
    for (const auto& pt : points)
    {
      octomap::OcTreeNode* node = tree->search(pt.x(), pt.y(), pt.z());
      EXPECT_FALSE(node && tree->isNodeOccupied(node))
          << "Voxel at (" << pt.x() << ", " << pt.y() << ", " << pt.z() << ") was not cleared!";
    }
    tree->unlockRead();
  }

  std::unique_ptr<occupancy_map_monitor::OccupancyMapMonitor> monitor_;
};

TEST_F(OctomapVoxelClearingTests, VoxelManualClearingTriggered)
{
  auto tree = monitor_->getOcTreePtr();
  populateRegion(octomap::point3d(0, 0, 0), octomap::point3d(1, 1, 1));

  // Define a sub-region to clear
  Eigen::Vector3d clear_min(0.2, 0.2, 0.2);
  Eigen::Vector3d clear_max(0.8, 0.8, 0.8);

  // EXECUTE: Directly wipe the specified volume in the tree
  tree->clearRegion(clear_min, clear_max);

  // VERIFY: Check center of the box
  verifyVolumeIsClear(tree, Eigen::Vector3d(0.5, 0.5, 0.5), 0.3);
}

TEST_F(OctomapVoxelClearingTests, VoxelClearingOnObjectAddition)
{
  auto tree = monitor_->getOcTreePtr();

  // --- Test Case 1: Cylinder ---
  populateRegion(octomap::point3d(0, 0, 0), octomap::point3d(1, 1, 1));

  auto cylinder = std::make_shared<shapes::Cylinder>(0.2, 0.6);
  Eigen::Isometry3d cyl_pose = Eigen::Isometry3d::Identity();
  cyl_pose.translation() = Eigen::Vector3d(0.3, 0.3, 0.3);

  // EXECUTE
  monitor_->clearShape(cylinder, cyl_pose);

  // VERIFY: Check center and points around the cylinder center
  verifyVolumeIsClear(tree, Eigen::Vector3d(0.3, 0.3, 0.3), 0.1);

  // --- Test Case 2: Rotated Box ---
  populateRegion(octomap::point3d(0, 0, 0), octomap::point3d(1, 1, 1));

  auto box = std::make_shared<shapes::Box>(0.2, 0.2, 0.8);
  Eigen::Isometry3d box_pose = Eigen::Isometry3d::Identity();
  box_pose = Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitY());  // Rotate 45 deg
  box_pose.translation() = Eigen::Vector3d(0.6, 0.6, 0.6);

  // EXECUTE
  monitor_->clearShape(box, box_pose);

  // VERIFY: Check center of the rotated box
  verifyVolumeIsClear(tree, Eigen::Vector3d(0.6, 0.6, 0.6), 0.1);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
