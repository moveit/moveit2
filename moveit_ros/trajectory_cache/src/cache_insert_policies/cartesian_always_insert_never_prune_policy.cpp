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
 * @brief Implementation of a cache insertion policy that always decides to insert and never decides
 * to prune for motion cartesian path requests.
 *
 * @see CacheInsertPolicyInterface<KeyT, ValueT, CacheEntryT>
 *
 * @author methylDragon
 */

#include <memory>
#include <sstream>
#include <string>

#include <warehouse_ros/message_collection.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/motion_plan_request.hpp>

#include <moveit/trajectory_cache/features/constant_features.hpp>
#include <moveit/trajectory_cache/features/get_cartesian_path_request_features.hpp>
#include <moveit/trajectory_cache/cache_insert_policies/cartesian_always_insert_never_prune_policy.hpp>
#include <moveit/trajectory_cache/utils/utils.hpp>

namespace moveit_ros
{
namespace trajectory_cache
{

using ::warehouse_ros::MessageCollection;
using ::warehouse_ros::MessageWithMetadata;
using ::warehouse_ros::Metadata;
using ::warehouse_ros::Query;

using ::moveit::core::MoveItErrorCode;
using ::moveit::planning_interface::MoveGroupInterface;

using ::moveit_msgs::msg::RobotTrajectory;
using ::moveit_msgs::srv::GetCartesianPath;
using ::moveit_ros::trajectory_cache::FeaturesInterface;

namespace
{

static const std::string FRACTION = "fraction";
static const std::string EXECUTION_TIME = "execution_time_s";

}  // namespace

// moveit_msgs::srv::GetCartesianPath::Request <=> moveit_msgs::srv::GetCartesianPath::Response ====

CartesianAlwaysInsertNeverPrunePolicy::CartesianAlwaysInsertNeverPrunePolicy()
  : name_("CartesianAlwaysInsertNeverPrunePolicy")
{
}

std::vector<std::unique_ptr<FeaturesInterface<GetCartesianPath::Request>>>
CartesianAlwaysInsertNeverPrunePolicy::getSupportedFeatures(double start_tolerance, double goal_tolerance,
                                                            double min_fraction)
{
  std::vector<std::unique_ptr<FeaturesInterface<GetCartesianPath::Request>>> out;

  // Start.
  out.push_back(std::make_unique<CartesianWorkspaceFeatures>());
  out.push_back(std::make_unique<CartesianStartStateJointStateFeatures>(start_tolerance));

  // Goal.
  out.push_back(std::make_unique<CartesianMaxSpeedAndAccelerationFeatures>());
  out.push_back(std::make_unique<CartesianMaxStepAndJumpThresholdFeatures>());
  out.push_back(std::make_unique<CartesianWaypointsFeatures>(goal_tolerance));
  out.push_back(std::make_unique<CartesianPathConstraintsFeatures>(goal_tolerance));

  // Min fraction.
  out.push_back(std::make_unique<QueryOnlyGTEFeature<double, GetCartesianPath::Request>>(FRACTION, min_fraction));

  return out;
}

std::string CartesianAlwaysInsertNeverPrunePolicy::getName() const
{
  return name_;
}

MoveItErrorCode CartesianAlwaysInsertNeverPrunePolicy::checkCacheInsertInputs(
    const MoveGroupInterface& move_group, const MessageCollection<RobotTrajectory>& /*coll*/,
    const GetCartesianPath::Request& key, const GetCartesianPath::Response& value)
{
  std::string workspace_frame_id = getCartesianPathRequestFrameId(move_group, key);

  // Check key.
  if (workspace_frame_id.empty())
  {
    return MoveItErrorCode(MoveItErrorCode::INVALID_MOTION_PLAN,
                           name_ + ": Skipping insert: Workspace frame ID cannot be empty.");
  }
  if (key.waypoints.empty())
  {
    return MoveItErrorCode(MoveItErrorCode::INVALID_MOTION_PLAN, name_ + ": Skipping insert: No waypoints.");
  }

  // Check value.
  if (value.solution.joint_trajectory.points.empty())
  {
    return MoveItErrorCode(MoveItErrorCode::INVALID_MOTION_PLAN, name_ + ": Empty joint trajectory points.");
  }
  if (value.solution.joint_trajectory.joint_names.empty())
  {
    return MoveItErrorCode(MoveItErrorCode::INVALID_MOTION_PLAN,
                           name_ + ": Skipping insert: Empty joint trajectory joint names.");
  }
  if (!value.solution.multi_dof_joint_trajectory.points.empty())
  {
    return MoveItErrorCode(MoveItErrorCode::INVALID_MOTION_PLAN,
                           name_ + ": Skipping insert: Multi-DOF trajectory plans are not supported.");
  }
  if (value.solution.joint_trajectory.header.frame_id.empty())
  {
    return MoveItErrorCode(MoveItErrorCode::INVALID_MOTION_PLAN,
                           name_ + ": Skipping insert: Trajectory frame ID cannot be empty.");
  }
  if (workspace_frame_id != value.solution.joint_trajectory.header.frame_id)
  {
    std::stringstream ss;
    ss << "Skipping insert: Plan request frame (" << workspace_frame_id << ") does not match plan frame ("
       << value.solution.joint_trajectory.header.frame_id << ").";
    return MoveItErrorCode(MoveItErrorCode::INVALID_MOTION_PLAN, ss.str());
  }

  return MoveItErrorCode::SUCCESS;
}

std::vector<MessageWithMetadata<RobotTrajectory>::ConstPtr> CartesianAlwaysInsertNeverPrunePolicy::fetchMatchingEntries(
    const MoveGroupInterface& move_group, const MessageCollection<RobotTrajectory>& coll,
    const GetCartesianPath::Request& key, const GetCartesianPath::Response& /*value*/, double exact_match_precision)
{
  Query::Ptr query = coll.createQuery();

  // We avoid the heap allocation from getSupportedFeatures() for performance reasons.
  // Furthermore, we set match_precision to zero because we want "exact" matches.

  // Start.
  if (MoveItErrorCode ret =
          CartesianWorkspaceFeatures().appendFeaturesAsExactFetchQuery(*query, key, move_group, exact_match_precision);
      !ret)
  {
    return {};
  }
  if (MoveItErrorCode ret = CartesianStartStateJointStateFeatures(/*match_precision=*/0.0)
                                .appendFeaturesAsExactFetchQuery(*query, key, move_group, exact_match_precision);
      !ret)
  {
    return {};
  }

  // Goal.
  if (MoveItErrorCode ret = CartesianMaxSpeedAndAccelerationFeatures().appendFeaturesAsExactFetchQuery(
          *query, key, move_group, exact_match_precision);
      !ret)
  {
    return {};
  }
  if (MoveItErrorCode ret = CartesianMaxStepAndJumpThresholdFeatures().appendFeaturesAsExactFetchQuery(
          *query, key, move_group, exact_match_precision);
      !ret)
  {
    return {};
  }
  if (MoveItErrorCode ret = CartesianWaypointsFeatures(/*match_precision=*/0.0)
                                .appendFeaturesAsExactFetchQuery(*query, key, move_group, exact_match_precision);
      !ret)
  {
    return {};
  }
  if (MoveItErrorCode ret = CartesianPathConstraintsFeatures(/*match_precision=*/0.0)
                                .appendFeaturesAsExactFetchQuery(*query, key, move_group, exact_match_precision);
      !ret)
  {
    return {};
  }

  return coll.queryList(query);
}

bool CartesianAlwaysInsertNeverPrunePolicy::prunePredicate(
    const MoveGroupInterface& /*move_group*/, const GetCartesianPath::Request& /*key*/,
    const GetCartesianPath::Response& /*value*/,
    const MessageWithMetadata<RobotTrajectory>::ConstPtr& /*matched_entry*/)
{
  return false;  // Never prune.
}

bool CartesianAlwaysInsertNeverPrunePolicy::insertPredicate(const MoveGroupInterface& /*move_group*/,
                                                            const GetCartesianPath::Request& /*key*/,
                                                            const GetCartesianPath::Response& /*value*/)
{
  return true;  // Always insert.
}

MoveItErrorCode CartesianAlwaysInsertNeverPrunePolicy::appendInsertMetadata(Metadata& metadata,
                                                                            const MoveGroupInterface& move_group,
                                                                            const GetCartesianPath::Request& key,
                                                                            const GetCartesianPath::Response& value)
{
  // Extract and append features.
  // We avoid the heap allocation from getSupportedFeatures() for performance reasons.

  // Start features.
  if (MoveItErrorCode ret = CartesianWorkspaceFeatures().appendFeaturesAsInsertMetadata(metadata, key, move_group); !ret)
  {
    return ret;
  }
  if (MoveItErrorCode ret = CartesianStartStateJointStateFeatures(/*match_tolerance=*/0)
                                .appendFeaturesAsInsertMetadata(metadata, key, move_group);
      !ret)
  {
    return ret;
  }

  // Goal features.
  if (MoveItErrorCode ret =
          CartesianMaxSpeedAndAccelerationFeatures().appendFeaturesAsInsertMetadata(metadata, key, move_group);
      !ret)
  {
    return ret;
  }
  if (MoveItErrorCode ret =
          CartesianMaxStepAndJumpThresholdFeatures().appendFeaturesAsInsertMetadata(metadata, key, move_group);
      !ret)
  {
    return ret;
  }
  if (MoveItErrorCode ret =
          CartesianWaypointsFeatures(/*match_tolerance=*/0).appendFeaturesAsInsertMetadata(metadata, key, move_group);
      !ret)
  {
    return ret;
  }
  if (MoveItErrorCode ret = CartesianPathConstraintsFeatures(/*match_tolerance=*/0)
                                .appendFeaturesAsInsertMetadata(metadata, key, move_group);
      !ret)
  {
    return ret;
  }

  // Append ValueT metadata.
  metadata.append(EXECUTION_TIME,
                  rclcpp::Duration(value.solution.joint_trajectory.points.back().time_from_start).seconds());
  metadata.append(FRACTION, value.fraction);

  return MoveItErrorCode::SUCCESS;
}

void CartesianAlwaysInsertNeverPrunePolicy::reset()
{
  return;
}

}  // namespace trajectory_cache
}  // namespace moveit_ros
