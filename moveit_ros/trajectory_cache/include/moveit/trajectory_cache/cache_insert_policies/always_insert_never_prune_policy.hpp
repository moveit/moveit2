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
 * @brief A cache insertion policy that always decides to insert and never decides to prune.
 *
 * This is mainly for explanatory purposes.
 * @see CacheInsertPolicyInterface<KeyT, ValueT, CacheEntryT>
 *
 * @author methylDragon
 */

#pragma once

#include <memory>

#include <warehouse_ros/message_collection.h>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>

#include <moveit/trajectory_cache/cache_insert_policies/cache_insert_policy_interface.hpp>
#include <moveit/trajectory_cache/features/features_interface.hpp>

namespace moveit_ros
{
namespace trajectory_cache
{

// =================================================================================================
// AlwaysInsertNeverPrunePolicy.
// =================================================================================================
// moveit_msgs::msg::MotionPlanRequest <=> moveit::planning_interface::MoveGroupInterface::Plan

/** @class AlwaysInsertNeverPrunePolicy
 *
 * @brief A cache insertion policy that always decides to insert and never decides to prune for motion plan requests.
 *
 * Supported Metadata and Features
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 * Appends the following additional metadata, which can be used for querying and sorting:
 *   - execution_time_s
 *   - planning_time_s
 *
 * Usable with the motion plan request features:
 *   - WorkspaceFeatures
 *   - StartStateJointStateFeatures
 *   - MaxSpeedAndAccelerationFeatures
 *   - GoalConstraintsFeatures
 *   - PathConstraintsFeatures
 *   - TrajectoryConstraintsFeatures
 *
 * @see motion_plan_request_features.hpp
 * @see FeaturesInterface<FeatureSourceT>
 *
 * Matches, Pruning, and Insertion
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 * A matching cache entry is one that exactly matches on every one of the features above.
 *
 * The sort order is ordered on execution_time_s in ascending order (so loweest execution time first).
 * This policy never indicates that pruning should happen, and always indicates that insertion should happen.
 */
class AlwaysInsertNeverPrunePolicy final
  : public CacheInsertPolicyInterface<moveit_msgs::msg::MotionPlanRequest,
                                      moveit::planning_interface::MoveGroupInterface::Plan,
                                      moveit_msgs::msg::RobotTrajectory>
{
public:
  /** @brief Configures and returns a vector of feature extractors that can be used with this policy. */
  static std::vector<std::unique_ptr<FeaturesInterface<moveit_msgs::msg::MotionPlanRequest>>>
  getSupportedFeatures(double start_tolerance, double goal_tolerance);

  AlwaysInsertNeverPrunePolicy();

  std::string getName() const override;

  moveit::core::MoveItErrorCode
  checkCacheInsertInputs(const moveit::planning_interface::MoveGroupInterface& move_group,
                         const warehouse_ros::MessageCollection<moveit_msgs::msg::RobotTrajectory>& coll,
                         const moveit_msgs::msg::MotionPlanRequest& key,
                         const moveit::planning_interface::MoveGroupInterface::Plan& value) override;

  std::vector<warehouse_ros::MessageWithMetadata<moveit_msgs::msg::RobotTrajectory>::ConstPtr>
  fetchMatchingEntries(const moveit::planning_interface::MoveGroupInterface& move_group,
                       const warehouse_ros::MessageCollection<moveit_msgs::msg::RobotTrajectory>& coll,
                       const moveit_msgs::msg::MotionPlanRequest& key,
                       const moveit::planning_interface::MoveGroupInterface::Plan& value,
                       double exact_match_precision) override;

  bool shouldPruneMatchingEntry(
      const moveit::planning_interface::MoveGroupInterface& move_group, const moveit_msgs::msg::MotionPlanRequest& key,
      const moveit::planning_interface::MoveGroupInterface::Plan& value,
      const warehouse_ros::MessageWithMetadata<moveit_msgs::msg::RobotTrajectory>::ConstPtr& matching_entry,
      std::string* reason = nullptr) override;

  bool shouldInsert(const moveit::planning_interface::MoveGroupInterface& move_group,
                    const moveit_msgs::msg::MotionPlanRequest& key,
                    const moveit::planning_interface::MoveGroupInterface::Plan& value,
                    std::string* reason = nullptr) override;

  moveit::core::MoveItErrorCode
  appendInsertMetadata(warehouse_ros::Metadata& metadata,
                       const moveit::planning_interface::MoveGroupInterface& move_group,
                       const moveit_msgs::msg::MotionPlanRequest& key,
                       const moveit::planning_interface::MoveGroupInterface::Plan& value) override;

  void reset() override;

private:
  const std::string name_;
  std::vector<std::unique_ptr<FeaturesInterface<moveit_msgs::msg::MotionPlanRequest>>> exact_matching_supported_features_;
};

// =================================================================================================
// CartesianAlwaysInsertNeverPrunePolicy.
// =================================================================================================
// moveit_msgs::srv::GetCartesianPath::Request <=> moveit_msgs::srv::GetCartesianPath::Response

/** @class CartesianAlwaysInsertNeverPrunePolicy
 *
 * @brief A cache insertion policy that always decides to insert and never decides to prune for cartesian path requests.
 *
 * Supported Metadata and Features
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 * Appends the following additional metadata, which can be used for querying and sorting:
 *   - fraction
 *   - execution_time_s
 *
 * NOTE:
 *   Planning time is not available. If you want to use it, add it as an additional MetadataOnly
 *   feature in the cache insert call.
 *
 * Compatible with the get cartesian path request features:
 *   - CartesianWorkspaceFeatures
 *   - CartesianStartStateJointStateFeatures
 *   - CartesianMaxSpeedAndAccelerationFeatures
 *   - CartesianMaxStepAndJumpThresholdFeatures
 *   - CartesianWaypointsFeatures
 *   - CartesianPathConstraintsFeatures
 *
 * @see get_cartesian_path_request_features.hpp
 * @see constant_features.hpp
 * @see FeaturesInterface<FeatureSourceT>
 *
 * Matches, Pruning, and Insertion
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 * A matching cache entry is one that exactly matches on every one of the features above, and
 * fraction.
 *
 * The sort order is ordered on execution_time_s in ascending order (so loweest execution time first).
 * This policy never indicates that pruning should happen, and always indicates that insertion should happen.
 */
class CartesianAlwaysInsertNeverPrunePolicy final
  : public CacheInsertPolicyInterface<moveit_msgs::srv::GetCartesianPath::Request,
                                      moveit_msgs::srv::GetCartesianPath::Response, moveit_msgs::msg::RobotTrajectory>
{
public:
  /** @brief Configures and returns a vector of feature extractors that can be used with this policy. */
  static std::vector<std::unique_ptr<FeaturesInterface<moveit_msgs::srv::GetCartesianPath::Request>>>
  getSupportedFeatures(double start_tolerance, double goal_tolerance, double min_fraction);

  CartesianAlwaysInsertNeverPrunePolicy();

  std::string getName() const override;

  moveit::core::MoveItErrorCode
  checkCacheInsertInputs(const moveit::planning_interface::MoveGroupInterface& move_group,
                         const warehouse_ros::MessageCollection<moveit_msgs::msg::RobotTrajectory>& coll,
                         const moveit_msgs::srv::GetCartesianPath::Request& key,
                         const moveit_msgs::srv::GetCartesianPath::Response& value) override;

  std::vector<warehouse_ros::MessageWithMetadata<moveit_msgs::msg::RobotTrajectory>::ConstPtr>
  fetchMatchingEntries(const moveit::planning_interface::MoveGroupInterface& move_group,
                       const warehouse_ros::MessageCollection<moveit_msgs::msg::RobotTrajectory>& coll,
                       const moveit_msgs::srv::GetCartesianPath::Request& key,
                       const moveit_msgs::srv::GetCartesianPath::Response& value,
                       double exact_match_precision) override;

  bool shouldPruneMatchingEntry(
      const moveit::planning_interface::MoveGroupInterface& move_group,
      const moveit_msgs::srv::GetCartesianPath::Request& key, const moveit_msgs::srv::GetCartesianPath::Response& value,
      const warehouse_ros::MessageWithMetadata<moveit_msgs::msg::RobotTrajectory>::ConstPtr& matching_entry,
      std::string* reason = nullptr) override;

  bool shouldInsert(const moveit::planning_interface::MoveGroupInterface& move_group,
                    const moveit_msgs::srv::GetCartesianPath::Request& key,
                    const moveit_msgs::srv::GetCartesianPath::Response& value, std::string* reason = nullptr) override;

  moveit::core::MoveItErrorCode appendInsertMetadata(warehouse_ros::Metadata& metadata,
                                                     const moveit::planning_interface::MoveGroupInterface& move_group,
                                                     const moveit_msgs::srv::GetCartesianPath::Request& key,
                                                     const moveit_msgs::srv::GetCartesianPath::Response& value) override;

  void reset() override;

private:
  const std::string name_;
  std::vector<std::unique_ptr<FeaturesInterface<moveit_msgs::srv::GetCartesianPath::Request>>>
      exact_matching_supported_features_;
};

}  // namespace trajectory_cache
}  // namespace moveit_ros
