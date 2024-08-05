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

#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include <warehouse_ros/message_collection.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>

#include <moveit/trajectory_cache/cache_insert_policies/best_seen_execution_time_policy.hpp>
#include <moveit/trajectory_cache/features/constant_features.hpp>
#include <moveit/trajectory_cache/features/get_cartesian_path_request_features.hpp>
#include <moveit/trajectory_cache/features/motion_plan_request_features.hpp>
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
using ::moveit_msgs::srv::GetCartesianPath;

using ::moveit_ros::trajectory_cache::FeaturesInterface;

namespace
{

static const std::string EXECUTION_TIME = "execution_time_s";
static const std::string FRACTION = "fraction";
static const std::string PLANNING_TIME = "planning_time_s";

}  // namespace

// =================================================================================================
// BestSeenExecutionTimePolicy.
// =================================================================================================
// moveit_msgs::msg::MotionPlanRequest <=> moveit::planning_interface::MoveGroupInterface::Plan

BestSeenExecutionTimePolicy::BestSeenExecutionTimePolicy()
  : name_("BestSeenExecutionTimePolicy"), best_seen_execution_time_(std::numeric_limits<double>::infinity())
{
  exact_matching_supported_features_ =
      std::move(BestSeenExecutionTimePolicy::getSupportedFeatures(/*start_tolerance=*/0.0,
                                                                  /*goal_tolerance=*/0.0));
}

std::vector<std::unique_ptr<FeaturesInterface<MotionPlanRequest>>>
BestSeenExecutionTimePolicy::getSupportedFeatures(double start_tolerance, double goal_tolerance)
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

std::string BestSeenExecutionTimePolicy::getName() const
{
  return name_;
}

MoveItErrorCode BestSeenExecutionTimePolicy::checkCacheInsertInputs(const MoveGroupInterface& move_group,
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

std::vector<MessageWithMetadata<RobotTrajectory>::ConstPtr> BestSeenExecutionTimePolicy::fetchMatchingEntries(
    const MoveGroupInterface& move_group, const MessageCollection<RobotTrajectory>& coll, const MotionPlanRequest& key,
    const MoveGroupInterface::Plan& /*value*/, double exact_match_precision)
{
  Query::Ptr query = coll.createQuery();
  for (const auto& feature : exact_matching_supported_features_)
  {
    if (MoveItErrorCode ret = feature->appendFeaturesAsExactFetchQuery(*query, key, move_group, exact_match_precision);
        !ret)
    {
      return {};
    }
  }

  std::vector<MessageWithMetadata<RobotTrajectory>::ConstPtr> out =
      coll.queryList(query, /*metadata_only=*/true, /*sort_by=*/EXECUTION_TIME, /*ascending=*/true);
  if (!out.empty())
  {
    best_seen_execution_time_ = out[0]->lookupDouble(EXECUTION_TIME);
  }

  return out;
}

bool BestSeenExecutionTimePolicy::shouldPruneMatchingEntry(
    const MoveGroupInterface& /*move_group*/, const MotionPlanRequest& /*key*/, const MoveGroupInterface::Plan& value,
    const MessageWithMetadata<RobotTrajectory>::ConstPtr& matching_entry)
{
  return matching_entry->lookupDouble(EXECUTION_TIME) >= getExecutionTime(value.trajectory);
}

bool BestSeenExecutionTimePolicy::shouldInsert(const MoveGroupInterface& /*move_group*/,
                                               const MotionPlanRequest& /*key*/, const MoveGroupInterface::Plan& value)
{
  return getExecutionTime(value.trajectory) < best_seen_execution_time_;
}

MoveItErrorCode BestSeenExecutionTimePolicy::appendInsertMetadata(Metadata& metadata,
                                                                  const MoveGroupInterface& move_group,
                                                                  const MotionPlanRequest& key,
                                                                  const MoveGroupInterface::Plan& value)
{
  for (const auto& feature : exact_matching_supported_features_)
  {
    if (MoveItErrorCode ret = feature->appendFeaturesAsInsertMetadata(metadata, key, move_group); !ret)
    {
      return {};
    }
  }

  // Append ValueT metadata.
  metadata.append(EXECUTION_TIME, getExecutionTime(value.trajectory));
  metadata.append(PLANNING_TIME, value.planning_time);

  return MoveItErrorCode::SUCCESS;
}

void BestSeenExecutionTimePolicy::reset()
{
  best_seen_execution_time_ = std::numeric_limits<double>::infinity();
  return;
}

}  // namespace trajectory_cache
}  // namespace moveit_ros
