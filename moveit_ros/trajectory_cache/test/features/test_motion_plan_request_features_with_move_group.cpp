// Copyright 2024 Intrinsic Innovation LLC.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/** @file
 * @author methylDragon
 *
 * Sanity tests for the moveit_msgs::msg::MotionPlanRequest feature extractors.
 *
 * @see motion_plan_request_features.hpp
 *
 * WARNING:
 *   These tests currently do not cover the implementation details, they are just the first sanity check for
 *   ensuring the most basic roundtrip functionality works.
 *
 *   For example, some features might not have any resulting changes to the metadata or query due to
 *   the nature of what is contained in the MotionPlanRequest passed to them.
 */

#include <memory>

#include <gtest/gtest.h>
#include <warehouse_ros/message_collection.h>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>

#include <moveit/trajectory_cache/features/features_interface.hpp>
#include <moveit/trajectory_cache/features/motion_plan_request_features.hpp>

#include "../fixtures/move_group_fixture.hpp"

namespace
{

using ::warehouse_ros::MessageCollection;
using ::warehouse_ros::Metadata;
using ::warehouse_ros::Query;

using ::moveit_msgs::msg::MotionPlanRequest;
using ::moveit_ros::trajectory_cache::FeaturesInterface;

using ::moveit_ros::trajectory_cache::GoalConstraintsFeatures;
using ::moveit_ros::trajectory_cache::MaxSpeedAndAccelerationFeatures;
using ::moveit_ros::trajectory_cache::PathConstraintsFeatures;
using ::moveit_ros::trajectory_cache::StartStateJointStateFeatures;
using ::moveit_ros::trajectory_cache::TrajectoryConstraintsFeatures;
using ::moveit_ros::trajectory_cache::WorkspaceFeatures;

TEST_F(MoveGroupFixture, MotionPlanRequestRoundTrip)
{
  // Test cases.
  double match_tolerance = 0.001;

  std::vector<std::unique_ptr<FeaturesInterface<MotionPlanRequest>>> features_under_test;

  features_under_test.push_back(std::make_unique<GoalConstraintsFeatures>(match_tolerance));
  features_under_test.push_back(std::make_unique<MaxSpeedAndAccelerationFeatures>());
  features_under_test.push_back(std::make_unique<PathConstraintsFeatures>(match_tolerance));
  features_under_test.push_back(std::make_unique<StartStateJointStateFeatures>(match_tolerance));
  features_under_test.push_back(std::make_unique<TrajectoryConstraintsFeatures>(match_tolerance));
  features_under_test.push_back(std::make_unique<WorkspaceFeatures>());

  // Setup.
  ASSERT_TRUE(move_group_->setPoseTarget(move_group_->getRandomPose()));
  MotionPlanRequest msg;
  move_group_->constructMotionPlanRequest(msg);

  // Core test.
  for (auto& feature : features_under_test)
  {
    MessageCollection<MotionPlanRequest> coll = db_->openCollection<MotionPlanRequest>("test_db", feature->getName());

    SCOPED_TRACE(feature->getName());

    Query::Ptr fuzzy_query = coll.createQuery();
    Query::Ptr exact_query = coll.createQuery();
    Metadata::Ptr metadata = coll.createMetadata();

    EXPECT_TRUE(feature->appendFeaturesAsInsertMetadata(*metadata, msg, *move_group_));
    coll.insert(msg, metadata);

    EXPECT_TRUE(
        feature->appendFeaturesAsFuzzyFetchQuery(*fuzzy_query, msg, *move_group_, /*exact_match_precision=*/0.0001));
    EXPECT_TRUE(
        feature->appendFeaturesAsExactFetchQuery(*exact_query, msg, *move_group_, /*exact_match_precision=*/0.0001));

    EXPECT_EQ(coll.queryList(fuzzy_query).size(), 1);
    EXPECT_EQ(coll.queryList(exact_query).size(), 1);
  }
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
