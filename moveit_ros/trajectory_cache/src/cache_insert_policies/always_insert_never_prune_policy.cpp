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
 * to prune for motion plan requests.
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

#include <moveit/trajectory_cache/features/motion_plan_request_features.hpp>
#include <moveit/trajectory_cache/cache_insert_policies/always_insert_never_prune_policy.hpp>
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

using ::moveit_msgs::msg::MotionPlanRequest;
using ::moveit_msgs::msg::RobotTrajectory;
using ::moveit_ros::trajectory_cache::FeaturesInterface;

namespace
{

static const std::string EXECUTION_TIME = "execution_time_s";
static const std::string PLANNING_TIME = "planning_time_s";

}  // namespace

// moveit_msgs::msg::MotionPlanRequest <=> moveit::planning_interface::MoveGroupInterface::Plan ====

AlwaysInsertNeverPrunePolicy::AlwaysInsertNeverPrunePolicy() : name_("AlwaysInsertNeverPrunePolicy")
{
}

std::vector<std::unique_ptr<FeaturesInterface<MotionPlanRequest>>>
AlwaysInsertNeverPrunePolicy::getSupportedFeatures(double start_tolerance, double goal_tolerance)
{
  std::vector<std::unique_ptr<FeaturesInterface<MotionPlanRequest>>> out;

  // Start.
  out.push_back(std::make_unique<WorkspaceFeatures>());
  out.push_back(std::make_unique<StartStateJointStateFeatures>(start_tolerance));

  // Goal.
  out.push_back(std::make_unique<MaxSpeedAndAccelerationFeatures>());
  out.push_back(std::make_unique<GoalConstraintsFeatures>(goal_tolerance));
  out.push_back(std::make_unique<PathConstraintsFeatures>(goal_tolerance));
  out.push_back(std::make_unique<TrajectoryConstraintsFeatures>(goal_tolerance));

  return out;
}

std::string AlwaysInsertNeverPrunePolicy::getName() const
{
  return name_;
}

MoveItErrorCode AlwaysInsertNeverPrunePolicy::checkCacheInsertInputs(const MoveGroupInterface& move_group,
                                                                     const MessageCollection<RobotTrajectory>& /*coll*/,
                                                                     const MotionPlanRequest& key,
                                                                     const MoveGroupInterface::Plan& value)
{
  std::string workspace_frame_id = getWorkspaceFrameId(move_group, key.workspace_parameters);

  // Check key.
  if (workspace_frame_id.empty())
  {
    return MoveItErrorCode(MoveItErrorCode::INVALID_MOTION_PLAN,
                           name_ + ": Skipping insert: Workspace frame ID cannot be empty.");
  }
  if (key.goal_constraints.empty())
  {
    return MoveItErrorCode(MoveItErrorCode::INVALID_MOTION_PLAN, name_ + ": Skipping insert: No goal.");
  }

  // Check value.
  if (value.trajectory.joint_trajectory.points.empty())
  {
    return MoveItErrorCode(MoveItErrorCode::INVALID_MOTION_PLAN, name_ + ": Empty joint trajectory points.");
  }
  if (value.trajectory.joint_trajectory.joint_names.empty())
  {
    return MoveItErrorCode(MoveItErrorCode::INVALID_MOTION_PLAN,
                           name_ + ": Skipping insert: Empty joint trajectory joint names.");
  }
  if (!value.trajectory.multi_dof_joint_trajectory.points.empty())
  {
    return MoveItErrorCode(MoveItErrorCode::INVALID_MOTION_PLAN,
                           name_ + ": Skipping insert: Multi-DOF trajectory plans are not supported.");
  }
  if (value.trajectory.joint_trajectory.header.frame_id.empty())
  {
    return MoveItErrorCode(MoveItErrorCode::INVALID_MOTION_PLAN,
                           name_ + ": Skipping insert: Trajectory frame ID cannot be empty.");
  }
  if (workspace_frame_id != value.trajectory.joint_trajectory.header.frame_id)
  {
    std::stringstream ss;
    ss << "Skipping insert: Plan request frame (" << workspace_frame_id << ") does not match plan frame ("
       << value.trajectory.joint_trajectory.header.frame_id << ").";
    return MoveItErrorCode(MoveItErrorCode::INVALID_MOTION_PLAN, ss.str());
  }

  return MoveItErrorCode::SUCCESS;
}

std::vector<MessageWithMetadata<RobotTrajectory>::ConstPtr> AlwaysInsertNeverPrunePolicy::fetchMatchingEntries(
    const MoveGroupInterface& move_group, const MessageCollection<RobotTrajectory>& coll, const MotionPlanRequest& key,
    const MoveGroupInterface::Plan& /*value*/, double exact_match_precision)
{
  Query::Ptr query = coll.createQuery();

  // We avoid the heap allocation from getSupportedFeatures() for performance reasons.
  // Furthermore, we set match_precision to zero because we want "exact" matches.

  // Start.
  if (MoveItErrorCode ret =
          WorkspaceFeatures().appendFeaturesAsExactFetchQuery(*query, key, move_group, exact_match_precision);
      !ret)
  {
    return {};
  };
  if (MoveItErrorCode ret = StartStateJointStateFeatures(/*match_precision=*/0.0)
                                .appendFeaturesAsExactFetchQuery(*query, key, move_group, exact_match_precision);
      !ret)
  {
    return {};
  };

  // Goal.
  if (MoveItErrorCode ret = MaxSpeedAndAccelerationFeatures().appendFeaturesAsExactFetchQuery(*query, key, move_group,
                                                                                              exact_match_precision);
      !ret)
  {
    return {};
  };
  if (MoveItErrorCode ret = GoalConstraintsFeatures(/*match_precision=*/0.0)
                                .appendFeaturesAsExactFetchQuery(*query, key, move_group, exact_match_precision);
      !ret)
  {
    return {};
  };
  if (MoveItErrorCode ret = PathConstraintsFeatures(/*match_precision=*/0.0)
                                .appendFeaturesAsExactFetchQuery(*query, key, move_group, exact_match_precision);
      !ret)
  {
    return {};
  };
  if (MoveItErrorCode ret = TrajectoryConstraintsFeatures(/*match_precision=*/0.0)
                                .appendFeaturesAsExactFetchQuery(*query, key, move_group, exact_match_precision);
      !ret)
  {
    return {};
  };

  return coll.queryList(query);
}

bool AlwaysInsertNeverPrunePolicy::prunePredicate(
    const MoveGroupInterface& /*move_group*/, const MotionPlanRequest& /*key*/,
    const MoveGroupInterface::Plan& /*value*/, const MessageWithMetadata<RobotTrajectory>::ConstPtr& /*matched_entry*/)
{
  return false;  // Never prune.
}

bool AlwaysInsertNeverPrunePolicy::insertPredicate(const MoveGroupInterface& /*move_group*/,
                                                   const MotionPlanRequest& /*key*/,
                                                   const MoveGroupInterface::Plan& /*value*/)
{
  return true;  // Always insert.
}

MoveItErrorCode AlwaysInsertNeverPrunePolicy::appendInsertMetadata(Metadata& metadata,
                                                                   const MoveGroupInterface& move_group,
                                                                   const MotionPlanRequest& key,
                                                                   const MoveGroupInterface::Plan& value)
{
  // Extract and append features.
  // We avoid the heap allocation from getSupportedFeatures() for performance reasons.

  // Start features.
  if (MoveItErrorCode ret = WorkspaceFeatures().appendFeaturesAsInsertMetadata(metadata, key, move_group); !ret)
  {
    return ret;
  }
  if (MoveItErrorCode ret =
          StartStateJointStateFeatures(/*match_tolerance=*/0).appendFeaturesAsInsertMetadata(metadata, key, move_group);
      !ret)
  {
    return ret;
  }

  // Goal features.
  if (MoveItErrorCode ret = MaxSpeedAndAccelerationFeatures().appendFeaturesAsInsertMetadata(metadata, key, move_group);
      !ret)
  {
    return ret;
  }
  if (MoveItErrorCode ret =
          GoalConstraintsFeatures(/*match_tolerance=*/0).appendFeaturesAsInsertMetadata(metadata, key, move_group);
      !ret)
  {
    return ret;
  }
  if (MoveItErrorCode ret =
          PathConstraintsFeatures(/*match_tolerance=*/0).appendFeaturesAsInsertMetadata(metadata, key, move_group);
      !ret)
  {
    return ret;
  }
  if (MoveItErrorCode ret = TrajectoryConstraintsFeatures(/*match_tolerance=*/0)
                                .appendFeaturesAsInsertMetadata(metadata, key, move_group);
      !ret)
  {
    return ret;
  }

  // Append ValueT metadata.
  metadata.append(EXECUTION_TIME,
                  rclcpp::Duration(value.trajectory.joint_trajectory.points.back().time_from_start).seconds());
  metadata.append(PLANNING_TIME, value.planning_time);

  return MoveItErrorCode::SUCCESS;
}

void AlwaysInsertNeverPrunePolicy::reset()
{
  return;
}

}  // namespace trajectory_cache
}  // namespace moveit_ros
