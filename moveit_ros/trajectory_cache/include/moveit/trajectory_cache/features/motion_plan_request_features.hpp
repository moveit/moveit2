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
 * @brief moveit_msgs::msg::MotionPlanRequest features to key the trajectory cache on.
 * @see FeaturesInterface<FeatureSourceT>
 *
 * @author methylDragon
 */

#pragma once

#include <warehouse_ros/message_collection.h>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>

#include <moveit/trajectory_cache/features/features_interface.hpp>

namespace moveit_ros
{
namespace trajectory_cache
{

// "Start" features. ===============================================================================

/** @class WorkspaceFeatures
 * @brief Extracts group name and details of the `workspace_parameters` field in the plan request.
 *
 * A cache hit with this feature is valid if the requested planning constraints has a workspace that
 * completely subsumes a cached entry's workspace.
 *
 * For example: (We ignore z for simplicity)
 *   If workspace is defined by the extrema (x_min, y_min, x_max, y_max),
 *
 *   Potential valid match if other constraints fulfilled:
 *     Request: (-1, -1, 1, 1)
 *     Plan in cache: (-0.5, -0.5, 0.5, 0.5)
 *
 *   No match, since this plan might cause the end effector to go out of bounds.:
 *     Request: (-1, -1, 1, 1)
 *     Plan in cache: (-2, -0.5, 0.5, 0.5)
 */
class WorkspaceFeatures final : public FeaturesInterface<moveit_msgs::msg::MotionPlanRequest>
{
public:
  WorkspaceFeatures();

  std::string getName() const override;

  moveit::core::MoveItErrorCode
  appendFeaturesAsFuzzyFetchQuery(warehouse_ros::Query& query, const moveit_msgs::msg::MotionPlanRequest& source,
                                  const moveit::planning_interface::MoveGroupInterface& move_group,
                                  double exact_match_precision) const override;

  moveit::core::MoveItErrorCode
  appendFeaturesAsExactFetchQuery(warehouse_ros::Query& query, const moveit_msgs::msg::MotionPlanRequest& source,
                                  const moveit::planning_interface::MoveGroupInterface& move_group,
                                  double exact_match_precision) const override;

  moveit::core::MoveItErrorCode
  appendFeaturesAsInsertMetadata(warehouse_ros::Metadata& metadata, const moveit_msgs::msg::MotionPlanRequest& source,
                                 const moveit::planning_interface::MoveGroupInterface& move_group) const override;

private:
  const std::string name_;
};

/** @class StartStateJointStateFeatures
 * @brief Extracts details of the joint state from the `start_state` field in the plan request.
 *
 * The start state will always be re-interpreted into explicit joint state positions.
 *
 * WARNING:
 *   MultiDOF joints and attached collision objects are not supported.
 *
 * @see appendRobotStateJointStateAsFetchQueryWithTolerance
 * @see appendRobotStateJointStateAsInsertMetadata
 */
class StartStateJointStateFeatures final : public FeaturesInterface<moveit_msgs::msg::MotionPlanRequest>
{
public:
  StartStateJointStateFeatures(double match_tolerance);

  std::string getName() const override;

  moveit::core::MoveItErrorCode
  appendFeaturesAsFuzzyFetchQuery(warehouse_ros::Query& query, const moveit_msgs::msg::MotionPlanRequest& source,
                                  const moveit::planning_interface::MoveGroupInterface& move_group,
                                  double exact_match_precision) const override;

  moveit::core::MoveItErrorCode
  appendFeaturesAsExactFetchQuery(warehouse_ros::Query& query, const moveit_msgs::msg::MotionPlanRequest& source,
                                  const moveit::planning_interface::MoveGroupInterface& move_group,
                                  double exact_match_precision) const override;

  moveit::core::MoveItErrorCode
  appendFeaturesAsInsertMetadata(warehouse_ros::Metadata& metadata, const moveit_msgs::msg::MotionPlanRequest& source,
                                 const moveit::planning_interface::MoveGroupInterface& move_group) const override;

private:
  moveit::core::MoveItErrorCode appendFeaturesAsFetchQueryWithTolerance(
      warehouse_ros::Query& query, const moveit_msgs::msg::MotionPlanRequest& source,
      const moveit::planning_interface::MoveGroupInterface& move_group, double match_tolerance) const;

  const std::string name_;
  const double match_tolerance_;
};

// "Goal" features. ================================================================================

/** @class MaxSpeedAndAccelerationFeatures
 * @brief Extracts max velocity and acceleration scaling, and cartesian speed limits from the plan request.
 *
 * These features will be extracted as less-than-or-equal features.
 *
 * NOTE: In accordance with the source message's field descriptions:
 *   If the max scaling factors are outside the range of (0, 1], they will be set to 1.
 *   If max_cartesian_speed is <= 0, it will be ignored instead.
 */
class MaxSpeedAndAccelerationFeatures final : public FeaturesInterface<moveit_msgs::msg::MotionPlanRequest>
{
public:
  MaxSpeedAndAccelerationFeatures();

  std::string getName() const override;

  moveit::core::MoveItErrorCode
  appendFeaturesAsFuzzyFetchQuery(warehouse_ros::Query& query, const moveit_msgs::msg::MotionPlanRequest& source,
                                  const moveit::planning_interface::MoveGroupInterface& move_group,
                                  double exact_match_precision) const override;

  moveit::core::MoveItErrorCode
  appendFeaturesAsExactFetchQuery(warehouse_ros::Query& query, const moveit_msgs::msg::MotionPlanRequest& source,
                                  const moveit::planning_interface::MoveGroupInterface& move_group,
                                  double exact_match_precision) const override;

  moveit::core::MoveItErrorCode
  appendFeaturesAsInsertMetadata(warehouse_ros::Metadata& metadata, const moveit_msgs::msg::MotionPlanRequest& source,
                                 const moveit::planning_interface::MoveGroupInterface& move_group) const override;

private:
  const std::string name_;
};

/** @class GoalConstraintsFeatures
 * @brief Extracts features from the `goal_constraints` field in the plan request.
 *
 * @see appendConstraintsAsFetchQueryWithTolerance
 * @see appendConstraintsAsInsertMetadata
 */
class GoalConstraintsFeatures final : public FeaturesInterface<moveit_msgs::msg::MotionPlanRequest>
{
public:
  GoalConstraintsFeatures(double match_tolerance);

  std::string getName() const override;

  moveit::core::MoveItErrorCode
  appendFeaturesAsFuzzyFetchQuery(warehouse_ros::Query& query, const moveit_msgs::msg::MotionPlanRequest& source,
                                  const moveit::planning_interface::MoveGroupInterface& move_group,
                                  double exact_match_precision) const override;

  moveit::core::MoveItErrorCode
  appendFeaturesAsExactFetchQuery(warehouse_ros::Query& query, const moveit_msgs::msg::MotionPlanRequest& source,
                                  const moveit::planning_interface::MoveGroupInterface& move_group,
                                  double exact_match_precision) const override;

  moveit::core::MoveItErrorCode
  appendFeaturesAsInsertMetadata(warehouse_ros::Metadata& metadata, const moveit_msgs::msg::MotionPlanRequest& source,
                                 const moveit::planning_interface::MoveGroupInterface& move_group) const override;

private:
  moveit::core::MoveItErrorCode appendFeaturesAsFetchQueryWithTolerance(
      warehouse_ros::Query& query, const moveit_msgs::msg::MotionPlanRequest& source,
      const moveit::planning_interface::MoveGroupInterface& move_group, double match_tolerance) const;

  const std::string name_;
  const double match_tolerance_;
};

/** @class PathConstraintsFeatures
 * @brief Extracts features from the `path_constraints` field in the plan request.
 *
 * @see appendConstraintsAsFetchQueryWithTolerance
 * @see appendConstraintsAsInsertMetadata
 */
class PathConstraintsFeatures final : public FeaturesInterface<moveit_msgs::msg::MotionPlanRequest>
{
public:
  PathConstraintsFeatures(double match_tolerance);

  std::string getName() const override;

  moveit::core::MoveItErrorCode
  appendFeaturesAsFuzzyFetchQuery(warehouse_ros::Query& query, const moveit_msgs::msg::MotionPlanRequest& source,
                                  const moveit::planning_interface::MoveGroupInterface& move_group,
                                  double exact_match_precision) const override;

  moveit::core::MoveItErrorCode
  appendFeaturesAsExactFetchQuery(warehouse_ros::Query& query, const moveit_msgs::msg::MotionPlanRequest& source,
                                  const moveit::planning_interface::MoveGroupInterface& move_group,
                                  double exact_match_precision) const override;

  moveit::core::MoveItErrorCode
  appendFeaturesAsInsertMetadata(warehouse_ros::Metadata& metadata, const moveit_msgs::msg::MotionPlanRequest& source,
                                 const moveit::planning_interface::MoveGroupInterface& move_group) const override;

private:
  moveit::core::MoveItErrorCode appendFeaturesAsFetchQueryWithTolerance(
      warehouse_ros::Query& query, const moveit_msgs::msg::MotionPlanRequest& source,
      const moveit::planning_interface::MoveGroupInterface& move_group, double match_tolerance) const;

  const std::string name_;
  const double match_tolerance_;
};

/** @class TrajectoryConstraintsFeatures
 * @brief Extracts features from the `trajectory_constraints` field in the plan request.
 *
 * @see appendConstraintsAsFetchQueryWithTolerance
 * @see appendConstraintsAsInsertMetadata
 */
class TrajectoryConstraintsFeatures final : public FeaturesInterface<moveit_msgs::msg::MotionPlanRequest>
{
public:
  TrajectoryConstraintsFeatures(double match_tolerance);

  std::string getName() const override;

  moveit::core::MoveItErrorCode
  appendFeaturesAsFuzzyFetchQuery(warehouse_ros::Query& query, const moveit_msgs::msg::MotionPlanRequest& source,
                                  const moveit::planning_interface::MoveGroupInterface& move_group,
                                  double exact_match_precision) const override;

  moveit::core::MoveItErrorCode
  appendFeaturesAsExactFetchQuery(warehouse_ros::Query& query, const moveit_msgs::msg::MotionPlanRequest& source,
                                  const moveit::planning_interface::MoveGroupInterface& move_group,
                                  double exact_match_precision) const override;

  moveit::core::MoveItErrorCode
  appendFeaturesAsInsertMetadata(warehouse_ros::Metadata& metadata, const moveit_msgs::msg::MotionPlanRequest& source,
                                 const moveit::planning_interface::MoveGroupInterface& move_group) const override;

private:
  moveit::core::MoveItErrorCode appendFeaturesAsFetchQueryWithTolerance(
      warehouse_ros::Query& query, const moveit_msgs::msg::MotionPlanRequest& source,
      const moveit::planning_interface::MoveGroupInterface& move_group, double match_tolerance) const;

  const std::string name_;
  const double match_tolerance_;
};

}  // namespace trajectory_cache
}  // namespace moveit_ros
