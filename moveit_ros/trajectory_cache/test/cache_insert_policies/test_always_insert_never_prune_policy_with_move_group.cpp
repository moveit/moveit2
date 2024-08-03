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

#include <gtest/gtest.h>
#include <warehouse_ros/message_collection.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/motion_plan_request.hpp>

#include <moveit/trajectory_cache/cache_insert_policies/always_insert_never_prune_policy.hpp>
#include <moveit/trajectory_cache/features/features_interface.hpp>

#include "../move_group_fixture.hpp"

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
using ::moveit_ros::trajectory_cache::AlwaysInsertNeverPrunePolicy;
using ::moveit_ros::trajectory_cache::FeaturesInterface;

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
    MoveGroupInterface::Plan plan = valid_plan;
    msg.goal_constraints.clear();
    EXPECT_FALSE(policy.checkCacheInsertInputs(*move_group_, coll, msg, plan));
  }

  // Empty joint trajectory points.
  {
    MotionPlanRequest msg = valid_msg;
    MoveGroupInterface::Plan plan = valid_plan;
    plan.trajectory.joint_trajectory.points.clear();
    EXPECT_FALSE(policy.checkCacheInsertInputs(*move_group_, coll, msg, plan));
  }

  // Empty joint trajectory names.
  {
    MotionPlanRequest msg = valid_msg;
    MoveGroupInterface::Plan plan = valid_plan;
    plan.trajectory.joint_trajectory.joint_names.clear();
    EXPECT_FALSE(policy.checkCacheInsertInputs(*move_group_, coll, msg, plan));
  }

  // Multi-DOF trajectory plan.
  {
    MotionPlanRequest msg = valid_msg;
    MoveGroupInterface::Plan plan = valid_plan;
    plan.trajectory.multi_dof_joint_trajectory.points.emplace_back();
    EXPECT_FALSE(policy.checkCacheInsertInputs(*move_group_, coll, msg, plan));
  }

  // Trajectory frame ID empty.
  {
    MotionPlanRequest msg = valid_msg;
    MoveGroupInterface::Plan plan = valid_plan;
    plan.trajectory.joint_trajectory.header.frame_id = "";
    EXPECT_FALSE(policy.checkCacheInsertInputs(*move_group_, coll, msg, plan));
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

  MessageCollection<RobotTrajectory> coll =
      db_->openCollection<RobotTrajectory>("test_db", policy.getName());
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
  } while (!ret);  // Sometimes the plan fails with the random pose.

  do
  {
    ASSERT_TRUE(move_group_->setPoseTarget(move_group_->getRandomPose()));
    move_group_->constructMotionPlanRequest(another_msg);
    another_ret = move_group_->plan(another_plan);
  } while (another_msg == msg && !another_ret);  // Also, sometimes the random pose happens to be the same.

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

    std::vector<MessageWithMetadata<RobotTrajectory>::ConstPtr> feature_fetch = coll.queryList(query);
    std::vector<MessageWithMetadata<RobotTrajectory>::ConstPtr> policy_fetch = policy.fetchMatchingEntries(
        *move_group_, coll, msg_plan_pair.first, msg_plan_pair.second, /*exact_match_precision=*/0.0001);

    ASSERT_EQ(feature_fetch.size(), 2);
    ASSERT_EQ(policy_fetch.size(), 2);
    for (size_t i = 0; i < feature_fetch.size(); ++i)
    {
      ASSERT_EQ(*feature_fetch[i], *policy_fetch[i]);

      // Policy is never prune.
      EXPECT_FALSE(policy.prunePredicate(*move_group_, msg_plan_pair.first, msg_plan_pair.second, policy_fetch[i]));
    }

    // Policy is always insert.
    EXPECT_TRUE(policy.insertPredicate(*move_group_, msg_plan_pair.first, msg_plan_pair.second));

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