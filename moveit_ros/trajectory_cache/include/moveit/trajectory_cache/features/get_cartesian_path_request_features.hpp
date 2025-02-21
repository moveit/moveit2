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
 * @brief moveit_msgs::srv::GetCartesianPath::Request features to key the trajectory cache on.
 * @see FeaturesInterface<FeatureSourceT>
 *
 * @author methylDragon
 */

#pragma once

#include <warehouse_ros/message_collection.h>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>

#include <moveit/trajectory_cache/features/features_interface.hpp>

namespace moveit_ros
{
namespace trajectory_cache
{

// "Start" features. ===============================================================================

/** @class CartesianWorkspaceFeatures
 * @brief Extracts group name and frame ID from the plan request.
 */
class CartesianWorkspaceFeatures final : public FeaturesInterface<moveit_msgs::srv::GetCartesianPath::Request>
{
public:
  CartesianWorkspaceFeatures();

  std::string getName() const override;

  moveit::core::MoveItErrorCode appendFeaturesAsFuzzyFetchQuery(
      warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
      const moveit::planning_interface::MoveGroupInterface& move_group, double exact_match_precision) const override;

  moveit::core::MoveItErrorCode appendFeaturesAsExactFetchQuery(
      warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
      const moveit::planning_interface::MoveGroupInterface& move_group, double exact_match_precision) const override;

  moveit::core::MoveItErrorCode
  appendFeaturesAsInsertMetadata(warehouse_ros::Metadata& metadata,
                                 const moveit_msgs::srv::GetCartesianPath::Request& source,
                                 const moveit::planning_interface::MoveGroupInterface& move_group) const override;

private:
  const std::string name_;
};

/** @class CartesianStartStateJointStateFeatures
 * @brief Extracts details of the joint state from the `start_state` field in the plan request.
 *
 * The start state will always be re-interpreted into explicit joint state positions.
 *
 * WARNING:
 *   MultiDOF joints and attached collision objects are not supported.
 */
class CartesianStartStateJointStateFeatures final
  : public FeaturesInterface<moveit_msgs::srv::GetCartesianPath::Request>
{
public:
  CartesianStartStateJointStateFeatures(double match_tolerance);

  std::string getName() const override;

  moveit::core::MoveItErrorCode appendFeaturesAsFuzzyFetchQuery(
      warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
      const moveit::planning_interface::MoveGroupInterface& move_group, double exact_match_precision) const override;

  moveit::core::MoveItErrorCode appendFeaturesAsExactFetchQuery(
      warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
      const moveit::planning_interface::MoveGroupInterface& move_group, double exact_match_precision) const override;

  moveit::core::MoveItErrorCode
  appendFeaturesAsInsertMetadata(warehouse_ros::Metadata& metadata,
                                 const moveit_msgs::srv::GetCartesianPath::Request& source,
                                 const moveit::planning_interface::MoveGroupInterface& move_group) const override;

private:
  moveit::core::MoveItErrorCode appendFeaturesAsFetchQueryWithTolerance(
      warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
      const moveit::planning_interface::MoveGroupInterface& move_group, double match_tolerance) const;

  const std::string name_;
  const double match_tolerance_;
};

// "Goal" features. ================================================================================

/** @class CartesianMaxSpeedAndAccelerationFeatures
 * @brief Extracts max velocity and acceleration scaling, and cartesian speed limits from the plan request.
 *
 * These features will be extracted as less-than-or-equal features.
 *
 * NOTE: In accordance with the source message's field descriptions:
 *   If the max scaling factors are outside the range of (0, 1], they will be set to 1.
 *   If max_cartesian_speed is <= 0, it will be ignored instead.
 */
class CartesianMaxSpeedAndAccelerationFeatures final
  : public FeaturesInterface<moveit_msgs::srv::GetCartesianPath::Request>
{
public:
  CartesianMaxSpeedAndAccelerationFeatures();

  std::string getName() const override;

  moveit::core::MoveItErrorCode appendFeaturesAsFuzzyFetchQuery(
      warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
      const moveit::planning_interface::MoveGroupInterface& move_group, double exact_match_precision) const override;

  moveit::core::MoveItErrorCode appendFeaturesAsExactFetchQuery(
      warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
      const moveit::planning_interface::MoveGroupInterface& move_group, double exact_match_precision) const override;

  moveit::core::MoveItErrorCode
  appendFeaturesAsInsertMetadata(warehouse_ros::Metadata& metadata,
                                 const moveit_msgs::srv::GetCartesianPath::Request& source,
                                 const moveit::planning_interface::MoveGroupInterface& move_group) const override;

private:
  const std::string name_;
};

/** @class CartesianMaxStepAndJumpThresholdFeatures
 * @brief Extracts max step and jump thresholds from the plan request.
 *
 * These features will be extracted as less-than-or-equal features.
 *
 * NOTE: In accordance with the source message's field descriptions:
 *   If a jump threshold is set to 0, it will be ignored instead.
 */
class CartesianMaxStepAndJumpThresholdFeatures final
  : public FeaturesInterface<moveit_msgs::srv::GetCartesianPath::Request>
{
public:
  CartesianMaxStepAndJumpThresholdFeatures();

  std::string getName() const override;

  moveit::core::MoveItErrorCode appendFeaturesAsFuzzyFetchQuery(
      warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
      const moveit::planning_interface::MoveGroupInterface& move_group, double exact_match_precision) const override;

  moveit::core::MoveItErrorCode appendFeaturesAsExactFetchQuery(
      warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
      const moveit::planning_interface::MoveGroupInterface& move_group, double exact_match_precision) const override;

  moveit::core::MoveItErrorCode
  appendFeaturesAsInsertMetadata(warehouse_ros::Metadata& metadata,
                                 const moveit_msgs::srv::GetCartesianPath::Request& source,
                                 const moveit::planning_interface::MoveGroupInterface& move_group) const override;

private:
  const std::string name_;
};

/** @class CartesianWaypointsFeatures
 * @brief Extracts features from the `waypoints` and `link_name` field in the plan request.
 *
 * `link_name` is extracted here because it is what the waypoints are stated with reference to.
 * Additionally, the waypoints will be restated in the robot's model frame.
 *
 * NOTE: In accordance with the source message's field descriptions:
 *   If link_name is empty, uses move_group.getEndEffectorLink().
 *
 * @see appendConstraintsAsFetchQueryWithTolerance
 * @see appendConstraintsAsInsertMetadata
 */
class CartesianWaypointsFeatures final : public FeaturesInterface<moveit_msgs::srv::GetCartesianPath::Request>
{
public:
  CartesianWaypointsFeatures(double match_tolerance);

  std::string getName() const override;

  moveit::core::MoveItErrorCode appendFeaturesAsFuzzyFetchQuery(
      warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
      const moveit::planning_interface::MoveGroupInterface& move_group, double exact_match_precision) const override;

  moveit::core::MoveItErrorCode appendFeaturesAsExactFetchQuery(
      warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
      const moveit::planning_interface::MoveGroupInterface& move_group, double exact_match_precision) const override;

  moveit::core::MoveItErrorCode
  appendFeaturesAsInsertMetadata(warehouse_ros::Metadata& metadata,
                                 const moveit_msgs::srv::GetCartesianPath::Request& source,
                                 const moveit::planning_interface::MoveGroupInterface& move_group) const override;

private:
  moveit::core::MoveItErrorCode appendFeaturesAsFetchQueryWithTolerance(
      warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
      const moveit::planning_interface::MoveGroupInterface& move_group, double match_tolerance) const;

  const std::string name_;
  const double match_tolerance_;
};

/** @class CartesianPathConstraintsFeatures
 * @brief Extracts features from the `path_constraints` field in the plan request.
 *
 * @see appendConstraintsAsFetchQueryWithTolerance
 * @see appendConstraintsAsInsertMetadata
 */
class CartesianPathConstraintsFeatures final : public FeaturesInterface<moveit_msgs::srv::GetCartesianPath::Request>
{
public:
  CartesianPathConstraintsFeatures(double match_tolerance);

  std::string getName() const override;

  moveit::core::MoveItErrorCode appendFeaturesAsFuzzyFetchQuery(
      warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
      const moveit::planning_interface::MoveGroupInterface& move_group, double exact_match_precision) const override;

  moveit::core::MoveItErrorCode appendFeaturesAsExactFetchQuery(
      warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
      const moveit::planning_interface::MoveGroupInterface& move_group, double exact_match_precision) const override;

  moveit::core::MoveItErrorCode
  appendFeaturesAsInsertMetadata(warehouse_ros::Metadata& metadata,
                                 const moveit_msgs::srv::GetCartesianPath::Request& source,
                                 const moveit::planning_interface::MoveGroupInterface& move_group) const override;

private:
  moveit::core::MoveItErrorCode appendFeaturesAsFetchQueryWithTolerance(
      warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
      const moveit::planning_interface::MoveGroupInterface& move_group, double match_tolerance) const;

  const std::string name_;
  const double match_tolerance_;
};

}  // namespace trajectory_cache
}  // namespace moveit_ros
