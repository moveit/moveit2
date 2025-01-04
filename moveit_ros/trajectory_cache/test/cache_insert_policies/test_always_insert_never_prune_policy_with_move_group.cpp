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
 */

#include <memory>
#include <utility>

#include <gtest/gtest.h>
#include <warehouse_ros/message_collection.h>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>

#include <moveit/trajectory_cache/cache_insert_policies/always_insert_never_prune_policy.hpp>
#include <moveit/trajectory_cache/features/features_interface.hpp>
#include <moveit/trajectory_cache/utils/utils.hpp>

#include "../fixtures/move_group_fixture.hpp"

namespace
{

using ::warehouse_ros::MessageCollection;
using ::warehouse_ros::MessageWithMetadata;
using ::warehouse_ros::Metadata;
using ::warehouse_ros::Query;

using ::moveit::core::MoveItErrorCode;
using ::moveit::planning_interface::MoveGroupInterface;

using ::moveit_msgs::msg::MotionPlanRequest;
using ::moveit_msgs::msg::RobotTrajectory;
using ::moveit_msgs::srv::GetCartesianPath;

using ::moveit_ros::trajectory_cache::AlwaysInsertNeverPrunePolicy;
using ::moveit_ros::trajectory_cache::CartesianAlwaysInsertNeverPrunePolicy;
using ::moveit_ros::trajectory_cache::constructGetCartesianPathRequest;
using ::moveit_ros::trajectory_cache::FeaturesInterface;

// =================================================================================================
// AlwaysInsertNeverPrunePolicy.
// =================================================================================================

TEST_F(MoveGroupFixture, AlwaysInsertNeverPrunePolicyChecks)
{
  // Setup.
  AlwaysInsertNeverPrunePolicy policy;

  MessageCollection<RobotTrajectory> coll = db_->openCollection<RobotTrajectory>("test_db", policy.getName());

  MotionPlanRequest valid_msg;
  MoveGroupInterface::Plan valid_plan;

  // Valid case, as control.
  {
    valid_msg.workspace_parameters.header.frame_id = "panda_link0";
    valid_msg.goal_constraints.emplace_back();

    valid_plan.trajectory.joint_trajectory.header.frame_id = "panda_link0";
    valid_plan.trajectory.joint_trajectory.joint_names.emplace_back();
    valid_plan.trajectory.joint_trajectory.points.emplace_back();

    MoveItErrorCode ret = policy.checkCacheInsertInputs(*move_group_, coll, valid_msg, valid_plan);
    ASSERT_TRUE(ret) << ret.message;
  }

  // We can't test workspace ID frame empty.
  // But it technically should be unreachable as long as the robot description is correct.

  // No goal.
  {
    MotionPlanRequest msg = valid_msg;
    msg.goal_constraints.clear();
    EXPECT_FALSE(policy.checkCacheInsertInputs(*move_group_, coll, msg, valid_plan));
  }

  // Empty joint trajectory points.
  {
    MoveGroupInterface::Plan plan = valid_plan;
    plan.trajectory.joint_trajectory.points.clear();
    EXPECT_FALSE(policy.checkCacheInsertInputs(*move_group_, coll, valid_msg, plan));
  }

  // Empty joint trajectory names.
  {
    MoveGroupInterface::Plan plan = valid_plan;
    plan.trajectory.joint_trajectory.joint_names.clear();
    EXPECT_FALSE(policy.checkCacheInsertInputs(*move_group_, coll, valid_msg, plan));
  }

  // Multi-DOF trajectory plan.
  {
    MoveGroupInterface::Plan plan = valid_plan;
    plan.trajectory.multi_dof_joint_trajectory.points.emplace_back();
    EXPECT_FALSE(policy.checkCacheInsertInputs(*move_group_, coll, valid_msg, plan));
  }

  // Trajectory frame ID empty.
  {
    MoveGroupInterface::Plan plan = valid_plan;
    plan.trajectory.joint_trajectory.header.frame_id = "";
    EXPECT_FALSE(policy.checkCacheInsertInputs(*move_group_, coll, valid_msg, plan));
  }

  // Mismatched frames.
  {
    MotionPlanRequest msg = valid_msg;
    MoveGroupInterface::Plan plan = valid_plan;
    msg.workspace_parameters.header.frame_id = "panda_link0";
    plan.trajectory.joint_trajectory.header.frame_id = "clearly_a_different_frame";
    EXPECT_FALSE(policy.checkCacheInsertInputs(*move_group_, coll, msg, plan));
  }
}

TEST_F(MoveGroupFixture, AlwaysInsertNeverPrunePolicyWorks)
{
  AlwaysInsertNeverPrunePolicy policy;

  MessageCollection<RobotTrajectory> coll = db_->openCollection<RobotTrajectory>("test_db", policy.getName());
  ASSERT_EQ(coll.count(), 0);

  // Setup. Get valid entries to insert.
  MotionPlanRequest msg;
  MoveGroupInterface::Plan plan;
  MoveItErrorCode ret = MoveItErrorCode::FAILURE;

  MotionPlanRequest another_msg;
  MoveGroupInterface::Plan another_plan;
  MoveItErrorCode another_ret = MoveItErrorCode::FAILURE;

  do
  {
    ASSERT_TRUE(move_group_->setPoseTarget(move_group_->getRandomPose()));
    move_group_->constructMotionPlanRequest(msg);
    ret = move_group_->plan(plan);
  } while (!ret || plan.trajectory.joint_trajectory.points.empty());  // Sometimes the plan fails with the random pose.

  do
  {
    ASSERT_TRUE(move_group_->setPoseTarget(move_group_->getRandomPose()));
    move_group_->constructMotionPlanRequest(another_msg);
    another_ret = move_group_->plan(another_plan);
  } while (another_msg == msg || !another_ret ||
           another_plan.trajectory.joint_trajectory.points
               .empty());  // Also, sometimes the random pose happens to be the same.

  // Ensure that the entries are valid.
  {
    MoveItErrorCode ret = policy.checkCacheInsertInputs(*move_group_, coll, msg, plan);
    ASSERT_TRUE(ret) << ret.message;
  }
  {
    MoveItErrorCode ret = policy.checkCacheInsertInputs(*move_group_, coll, another_msg, another_plan);
    ASSERT_TRUE(ret) << ret.message;
  }

  // Core test. ====================================================================================
  // NOTE: Be mindful that the policy is stateful.

  // Insert messages and check if policy-specific additional metadata are added.
  size_t count = 0;
  for (const auto& msg_plan_pair : { std::make_pair(msg, plan), std::make_pair(another_msg, another_plan) })
  {
    Metadata::Ptr metadata = coll.createMetadata();
    EXPECT_TRUE(policy.appendInsertMetadata(*metadata, *move_group_, msg_plan_pair.first, msg_plan_pair.second));
    EXPECT_TRUE(metadata->lookupField("execution_time_s"));
    EXPECT_TRUE(metadata->lookupField("planning_time_s"));

    // We add two to test the prune predicate, as appropriate.
    coll.insert(msg_plan_pair.second.trajectory, metadata);
    coll.insert(msg_plan_pair.second.trajectory, metadata);
    count += 2;
    ASSERT_EQ(coll.count(), count);
  }

  // Fetch with features from getSupportedFeatures and fetchMatchingEntries.
  // In this case the results should also match with fetchMatchingEntries.
  //
  // We also test the predicates here.
  std::vector<std::unique_ptr<FeaturesInterface<MotionPlanRequest>>> features =
      AlwaysInsertNeverPrunePolicy::getSupportedFeatures(/*start_tolerance=*/0.025, /*goal_tolerance=*/0.001);

  for (const auto& msg_plan_pair : { std::make_pair(msg, plan), std::make_pair(another_msg, another_plan) })
  {
    Query::Ptr query = coll.createQuery();
    for (const auto& feature : features)
    {
      ASSERT_TRUE(feature->appendFeaturesAsExactFetchQuery(*query, msg_plan_pair.first, *move_group_,
                                                           /*exact_match_precision=*/0.0001));
    }

    std::vector<MessageWithMetadata<RobotTrajectory>::ConstPtr> feature_fetch =
        coll.queryList(query, /*metadata_only=*/true);
    std::vector<MessageWithMetadata<RobotTrajectory>::ConstPtr> policy_fetch = policy.fetchMatchingEntries(
        *move_group_, coll, msg_plan_pair.first, msg_plan_pair.second, /*exact_match_precision=*/0.0001);

    ASSERT_EQ(feature_fetch.size(), 2);
    ASSERT_EQ(policy_fetch.size(), 2);
    for (size_t i = 0; i < feature_fetch.size(); ++i)
    {
      ASSERT_EQ(*feature_fetch[i], *policy_fetch[i]);

      // Policy is never prune.
      std::string prune_reason;
      EXPECT_FALSE(policy.shouldPruneMatchingEntry(*move_group_, msg_plan_pair.first, msg_plan_pair.second,
                                                   policy_fetch[i], &prune_reason));
      EXPECT_FALSE(prune_reason.empty());
    }

    // Policy is always insert.
    std::string insert_reason;
    EXPECT_TRUE(policy.shouldInsert(*move_group_, msg_plan_pair.first, msg_plan_pair.second, &insert_reason));
    EXPECT_FALSE(insert_reason.empty());

    policy.reset();
  }
}

// =================================================================================================
// CartesianAlwaysInsertNeverPrunePolicy.
// =================================================================================================

TEST_F(MoveGroupFixture, CartesianAlwaysInsertNeverPrunePolicyChecks)
{
  // Setup.
  CartesianAlwaysInsertNeverPrunePolicy policy;

  MessageCollection<RobotTrajectory> coll = db_->openCollection<RobotTrajectory>("test_db", policy.getName());

  GetCartesianPath::Request valid_msg;
  GetCartesianPath::Response valid_plan;
  valid_plan.fraction = -1;

  do
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(move_group_->getCurrentPose().pose);
    waypoints.push_back(move_group_->getRandomPose().pose);

    valid_msg = constructGetCartesianPathRequest(*move_group_, waypoints, /*max_step=*/0.01, /*jump_threshold=*/0.0);
// TODO: Swap this over to the new computeCartesianPath API.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    valid_plan.fraction = move_group_->computeCartesianPath(valid_msg.waypoints, valid_msg.max_step,
                                                            valid_msg.jump_threshold, valid_plan.solution);
#pragma GCC diagnostic pop
  } while (valid_plan.fraction <= 0);  // Sometimes the plan fails with the random pose.

  // Valid case, as control.
  {
    MoveItErrorCode ret = policy.checkCacheInsertInputs(*move_group_, coll, valid_msg, valid_plan);
    ASSERT_TRUE(ret) << ret.message;
  }

  // We can't test workspace ID frame empty.
  // But it technically should be unreachable as long as the robot description is correct.

  // No waypoints.
  {
    GetCartesianPath::Request msg = valid_msg;
    msg.waypoints.clear();
    EXPECT_FALSE(policy.checkCacheInsertInputs(*move_group_, coll, msg, valid_plan));
  }

  // Empty joint trajectory points.
  {
    GetCartesianPath::Response plan = valid_plan;
    plan.solution.joint_trajectory.points.clear();
    EXPECT_FALSE(policy.checkCacheInsertInputs(*move_group_, coll, valid_msg, plan));
  }

  // Empty joint trajectory names.
  {
    GetCartesianPath::Response plan = valid_plan;
    plan.solution.joint_trajectory.joint_names.clear();
    EXPECT_FALSE(policy.checkCacheInsertInputs(*move_group_, coll, valid_msg, plan));
  }

  // Multi-DOF trajectory plan.
  {
    GetCartesianPath::Response plan = valid_plan;
    plan.solution.multi_dof_joint_trajectory.points.emplace_back();
    EXPECT_FALSE(policy.checkCacheInsertInputs(*move_group_, coll, valid_msg, plan));
  }

  // Trajectory frame ID empty.
  {
    GetCartesianPath::Response plan = valid_plan;
    plan.solution.joint_trajectory.header.frame_id = "";
    EXPECT_FALSE(policy.checkCacheInsertInputs(*move_group_, coll, valid_msg, plan));
  }

  // Mismatched frames.
  {
    GetCartesianPath::Request msg = valid_msg;
    GetCartesianPath::Response plan = valid_plan;
    msg.header.frame_id = "panda_link0";
    plan.solution.joint_trajectory.header.frame_id = "clearly_a_different_frame";
    EXPECT_FALSE(policy.checkCacheInsertInputs(*move_group_, coll, msg, plan));
  }
}

TEST_F(MoveGroupFixture, CartesianAlwaysInsertNeverPrunePolicyWorks)
{
  CartesianAlwaysInsertNeverPrunePolicy policy;

  MessageCollection<RobotTrajectory> coll = db_->openCollection<RobotTrajectory>("test_db", policy.getName());
  ASSERT_EQ(coll.count(), 0);

  // Setup. Get valid entries to insert.
  GetCartesianPath::Request msg;
  GetCartesianPath::Response plan;
  plan.fraction = -1;

  GetCartesianPath::Request another_msg;
  GetCartesianPath::Response another_plan;
  another_plan.fraction = -1;

  do
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(move_group_->getCurrentPose().pose);
    waypoints.push_back(move_group_->getRandomPose().pose);

    msg = constructGetCartesianPathRequest(*move_group_, waypoints, /*max_step=*/0.01, /*jump_threshold=*/0.0);
// TODO: Swap this over to the new computeCartesianPath API.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    plan.fraction = move_group_->computeCartesianPath(msg.waypoints, msg.max_step, msg.jump_threshold, plan.solution);
#pragma GCC diagnostic pop
  } while (plan.fraction <= -1 &&
           plan.solution.joint_trajectory.points.size() < 2);  // Sometimes the plan fails with the random pose.

  do
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(move_group_->getCurrentPose().pose);
    waypoints.push_back(move_group_->getRandomPose().pose);

    another_msg = constructGetCartesianPathRequest(*move_group_, waypoints, /*max_step=*/0.01, /*jump_threshold=*/0.0);
// TODO: Swap this over to the new computeCartesianPath API.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    another_plan.fraction = move_group_->computeCartesianPath(another_msg.waypoints, another_msg.max_step,
                                                              another_msg.jump_threshold, another_plan.solution);
#pragma GCC diagnostic pop
  } while (another_plan.fraction <= -1 &&
           plan.solution.joint_trajectory.points.size() < 2);  // Sometimes the plan fails with the random pose.

  // Ensure that the entries are valid.
  {
    MoveItErrorCode ret = policy.checkCacheInsertInputs(*move_group_, coll, msg, plan);
    ASSERT_TRUE(ret) << ret.message;
  }
  {
    MoveItErrorCode ret = policy.checkCacheInsertInputs(*move_group_, coll, another_msg, another_plan);
    ASSERT_TRUE(ret) << ret.message;
  }

  // Core test. ====================================================================================
  // NOTE: Be mindful that the policy is stateful.

  // Insert messages and check if policy-specific additional metadata are added.
  size_t count = 0;
  for (const auto& msg_plan_pair : { std::make_pair(msg, plan), std::make_pair(another_msg, another_plan) })
  {
    Metadata::Ptr metadata = coll.createMetadata();
    EXPECT_TRUE(policy.appendInsertMetadata(*metadata, *move_group_, msg_plan_pair.first, msg_plan_pair.second));
    EXPECT_TRUE(metadata->lookupField("execution_time_s"));
    EXPECT_TRUE(metadata->lookupField("fraction"));

    // We add two to test the prune predicate, as appropriate.
    coll.insert(msg_plan_pair.second.solution, metadata);
    coll.insert(msg_plan_pair.second.solution, metadata);
    count += 2;
    ASSERT_EQ(coll.count(), count);
  }

  // Fetch with features from getSupportedFeatures and fetchMatchingEntries.
  // In this case the results should also match with fetchMatchingEntries.
  //
  // We also test the predicates here.
  std::vector<std::unique_ptr<FeaturesInterface<GetCartesianPath::Request>>> features =
      CartesianAlwaysInsertNeverPrunePolicy::getSupportedFeatures(/*start_tolerance=*/0.025,
                                                                  /*goal_tolerance=*/0.001,
                                                                  /*min_fraction=*/0.0);

  for (const auto& msg_plan_pair : { std::make_pair(msg, plan), std::make_pair(another_msg, another_plan) })
  {
    Query::Ptr query = coll.createQuery();
    for (const auto& feature : features)
    {
      ASSERT_TRUE(feature->appendFeaturesAsExactFetchQuery(*query, msg_plan_pair.first, *move_group_,
                                                           /*exact_match_precision=*/0.0001));
    }

    std::vector<MessageWithMetadata<RobotTrajectory>::ConstPtr> feature_fetch =
        coll.queryList(query, /*metadata_only=*/true);
    std::vector<MessageWithMetadata<RobotTrajectory>::ConstPtr> policy_fetch = policy.fetchMatchingEntries(
        *move_group_, coll, msg_plan_pair.first, msg_plan_pair.second, /*exact_match_precision=*/0.0001);

    ASSERT_EQ(feature_fetch.size(), 2);
    ASSERT_EQ(policy_fetch.size(), 2);
    for (size_t i = 0; i < feature_fetch.size(); ++i)
    {
      ASSERT_EQ(*feature_fetch[i], *policy_fetch[i]);

      // Policy is never prune.
      std::string prune_reason;
      EXPECT_FALSE(policy.shouldPruneMatchingEntry(*move_group_, msg_plan_pair.first, msg_plan_pair.second,
                                                   policy_fetch[i], &prune_reason));
      EXPECT_FALSE(prune_reason.empty());
    }

    // Policy is always insert.
    std::string insert_reason;
    EXPECT_TRUE(policy.shouldInsert(*move_group_, msg_plan_pair.first, msg_plan_pair.second, &insert_reason));
    EXPECT_FALSE(insert_reason.empty());

    policy.reset();
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
