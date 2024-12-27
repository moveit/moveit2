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
 * Sanity tests for the moveit_msgs::srv::GetCartesianPath::Request feature extractors.
 *
 * @see get_cartesian_path_request_features.hpp
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
#include <moveit_msgs/srv/get_cartesian_path.hpp>

#include <moveit/trajectory_cache/features/features_interface.hpp>
#include <moveit/trajectory_cache/features/get_cartesian_path_request_features.hpp>
#include <moveit/trajectory_cache/utils/utils.hpp>

#include "../fixtures/move_group_fixture.hpp"

namespace
{

using ::warehouse_ros::MessageCollection;
using ::warehouse_ros::Metadata;
using ::warehouse_ros::Query;

using ::moveit_msgs::srv::GetCartesianPath;

using ::moveit_ros::trajectory_cache::constructGetCartesianPathRequest;
using ::moveit_ros::trajectory_cache::FeaturesInterface;

using ::moveit_ros::trajectory_cache::CartesianMaxSpeedAndAccelerationFeatures;
using ::moveit_ros::trajectory_cache::CartesianMaxStepAndJumpThresholdFeatures;
using ::moveit_ros::trajectory_cache::CartesianPathConstraintsFeatures;
using ::moveit_ros::trajectory_cache::CartesianStartStateJointStateFeatures;
using ::moveit_ros::trajectory_cache::CartesianWaypointsFeatures;
using ::moveit_ros::trajectory_cache::CartesianWorkspaceFeatures;

TEST_F(MoveGroupFixture, GetCartesianPathRequestRoundTrip)
{
  // Test cases.
  double match_tolerance = 0.001;

  std::vector<std::unique_ptr<FeaturesInterface<GetCartesianPath::Request>>> features_under_test;

  features_under_test.push_back(std::make_unique<CartesianMaxSpeedAndAccelerationFeatures>());
  features_under_test.push_back(std::make_unique<CartesianMaxStepAndJumpThresholdFeatures>());
  features_under_test.push_back(std::make_unique<CartesianPathConstraintsFeatures>(match_tolerance));
  features_under_test.push_back(std::make_unique<CartesianStartStateJointStateFeatures>(match_tolerance));
  features_under_test.push_back(std::make_unique<CartesianWaypointsFeatures>(match_tolerance));
  features_under_test.push_back(std::make_unique<CartesianWorkspaceFeatures>());

  // Setup.
  std::vector<geometry_msgs::msg::Pose> waypoints = { move_group_->getRandomPose().pose,
                                                      move_group_->getRandomPose().pose };

  GetCartesianPath::Request msg = constructGetCartesianPathRequest(*move_group_, waypoints, /*max_step=*/1.0,
                                                                   /*jump_threshold=*/0.0, /*avoid_collisions=*/false);

  // Core test.
  for (auto& feature : features_under_test)
  {
    MessageCollection<GetCartesianPath::Request> coll =
        db_->openCollection<GetCartesianPath::Request>("test_db", feature->getName());

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
