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
 * @brief Implementation of moveit_msgs::msg::MotionPlanRequest features to key the trajectory cache on.
 * @see FeaturesInterface<FeatureSourceT>
 *
 * @author methylDragon
 */

#include <rclcpp/logging.hpp>
#include <warehouse_ros/message_collection.h>

#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>

#include <moveit/trajectory_cache/features/motion_plan_request_features.hpp>
#include <moveit/trajectory_cache/utils/utils.hpp>

namespace moveit_ros
{
namespace trajectory_cache
{

using ::warehouse_ros::Metadata;
using ::warehouse_ros::Query;

using ::moveit::core::MoveItErrorCode;
using ::moveit::planning_interface::MoveGroupInterface;

using ::moveit_msgs::msg::MotionPlanRequest;

// "Start" features. ===============================================================================

// WorkspaceFeatures.

WorkspaceFeatures::WorkspaceFeatures() : name_("WorkspaceFeatures")
{
}

std::string WorkspaceFeatures::getName() const
{
  return name_;
}

MoveItErrorCode WorkspaceFeatures::appendFeaturesAsFuzzyFetchQuery(Query& query, const MotionPlanRequest& source,
                                                                   const MoveGroupInterface& move_group,
                                                                   double exact_match_precision) const
{
  return appendFeaturesAsExactFetchQuery(query, source, move_group, exact_match_precision);
};

MoveItErrorCode WorkspaceFeatures::appendFeaturesAsExactFetchQuery(Query& query, const MotionPlanRequest& source,
                                                                   const MoveGroupInterface& move_group,
                                                                   double /*exact_match_precision*/) const
{
  query.append(name_ + ".group_name", source.group_name);
  query.append(name_ + ".workspace_parameters.header.frame_id",
               getWorkspaceFrameId(move_group, source.workspace_parameters));
  query.appendGTE(name_ + ".workspace_parameters.min_corner.x", source.workspace_parameters.min_corner.x);
  query.appendGTE(name_ + ".workspace_parameters.min_corner.y", source.workspace_parameters.min_corner.y);
  query.appendGTE(name_ + ".workspace_parameters.min_corner.z", source.workspace_parameters.min_corner.z);
  query.appendLTE(name_ + ".workspace_parameters.max_corner.x", source.workspace_parameters.max_corner.x);
  query.appendLTE(name_ + ".workspace_parameters.max_corner.y", source.workspace_parameters.max_corner.y);
  query.appendLTE(name_ + ".workspace_parameters.max_corner.z", source.workspace_parameters.max_corner.z);
  return MoveItErrorCode::SUCCESS;
};

MoveItErrorCode WorkspaceFeatures::appendFeaturesAsInsertMetadata(Metadata& metadata, const MotionPlanRequest& source,
                                                                  const MoveGroupInterface& move_group) const
{
  metadata.append(name_ + ".group_name", source.group_name);
  metadata.append(name_ + ".workspace_parameters.header.frame_id",
                  getWorkspaceFrameId(move_group, source.workspace_parameters));
  metadata.append(name_ + ".workspace_parameters.min_corner.x", source.workspace_parameters.min_corner.x);
  metadata.append(name_ + ".workspace_parameters.min_corner.y", source.workspace_parameters.min_corner.y);
  metadata.append(name_ + ".workspace_parameters.min_corner.z", source.workspace_parameters.min_corner.z);
  metadata.append(name_ + ".workspace_parameters.max_corner.x", source.workspace_parameters.max_corner.x);
  metadata.append(name_ + ".workspace_parameters.max_corner.y", source.workspace_parameters.max_corner.y);
  metadata.append(name_ + ".workspace_parameters.max_corner.z", source.workspace_parameters.max_corner.z);
  return MoveItErrorCode::SUCCESS;
};

// StartStateJointStateFeatures.

StartStateJointStateFeatures::StartStateJointStateFeatures(double match_tolerance)
  : name_("StartStateJointStateFeatures"), match_tolerance_(match_tolerance)
{
}

std::string StartStateJointStateFeatures::getName() const
{
  return name_;
}

MoveItErrorCode StartStateJointStateFeatures::appendFeaturesAsFuzzyFetchQuery(Query& query,
                                                                              const MotionPlanRequest& source,
                                                                              const MoveGroupInterface& move_group,
                                                                              double exact_match_precision) const
{
  return appendFeaturesAsFetchQueryWithTolerance(query, source, move_group, match_tolerance_ + exact_match_precision);
};

MoveItErrorCode StartStateJointStateFeatures::appendFeaturesAsExactFetchQuery(Query& query,
                                                                              const MotionPlanRequest& source,
                                                                              const MoveGroupInterface& move_group,
                                                                              double exact_match_precision) const
{
  return appendFeaturesAsFetchQueryWithTolerance(query, source, move_group, exact_match_precision);
};

MoveItErrorCode StartStateJointStateFeatures::appendFeaturesAsInsertMetadata(Metadata& metadata,
                                                                             const MotionPlanRequest& source,
                                                                             const MoveGroupInterface& move_group) const
{
  return appendRobotStateJointStateAsInsertMetadata(metadata, source.start_state, move_group, name_ + ".start_state");
};

MoveItErrorCode StartStateJointStateFeatures::appendFeaturesAsFetchQueryWithTolerance(
    Query& query, const MotionPlanRequest& source, const MoveGroupInterface& move_group, double match_tolerance) const
{
  return appendRobotStateJointStateAsFetchQueryWithTolerance(query, source.start_state, move_group, match_tolerance,
                                                             name_ + ".start_state");
};

// "Goal" features. ================================================================================

// MaxSpeedAndAccelerationFeatures.

MaxSpeedAndAccelerationFeatures::MaxSpeedAndAccelerationFeatures() : name_("MaxSpeedAndAccelerationFeatures")
{
}

std::string MaxSpeedAndAccelerationFeatures::getName() const
{
  return name_;
}

MoveItErrorCode MaxSpeedAndAccelerationFeatures::appendFeaturesAsFuzzyFetchQuery(Query& query,
                                                                                 const MotionPlanRequest& source,
                                                                                 const MoveGroupInterface& move_group,
                                                                                 double exact_match_precision) const
{
  return appendFeaturesAsExactFetchQuery(query, source, move_group, exact_match_precision);
};

MoveItErrorCode
MaxSpeedAndAccelerationFeatures::appendFeaturesAsExactFetchQuery(Query& query, const MotionPlanRequest& source,
                                                                 const MoveGroupInterface& /*move_group*/,
                                                                 double /*exact_match_precision*/) const
{
  if (source.max_velocity_scaling_factor <= 0 || source.max_velocity_scaling_factor > 1.0)
  {
    query.appendLTE(name_ + ".max_velocity_scaling_factor", 1.0);
  }
  else
  {
    query.appendLTE(name_ + ".max_velocity_scaling_factor", source.max_velocity_scaling_factor);
  }

  if (source.max_acceleration_scaling_factor <= 0 || source.max_acceleration_scaling_factor > 1.0)
  {
    query.appendLTE(name_ + ".max_acceleration_scaling_factor", 1.0);
  }
  else
  {
    query.appendLTE(name_ + ".max_acceleration_scaling_factor", source.max_acceleration_scaling_factor);
  }

  if (source.max_cartesian_speed > 0)
  {
    query.append(name_ + ".cartesian_speed_limited_link", source.cartesian_speed_limited_link);
    query.appendLTE(name_ + ".max_cartesian_speed", source.max_cartesian_speed);
  }

  return MoveItErrorCode::SUCCESS;
};

MoveItErrorCode
MaxSpeedAndAccelerationFeatures::appendFeaturesAsInsertMetadata(Metadata& metadata, const MotionPlanRequest& source,
                                                                const MoveGroupInterface& /*move_group*/) const
{
  if (source.max_velocity_scaling_factor <= 0 || source.max_velocity_scaling_factor > 1.0)
  {
    metadata.append(name_ + ".max_velocity_scaling_factor", 1.0);
  }
  else
  {
    metadata.append(name_ + ".max_velocity_scaling_factor", source.max_velocity_scaling_factor);
  }

  if (source.max_acceleration_scaling_factor <= 0 || source.max_acceleration_scaling_factor > 1.0)
  {
    metadata.append(name_ + ".max_acceleration_scaling_factor", 1.0);
  }
  else
  {
    metadata.append(name_ + ".max_acceleration_scaling_factor", source.max_acceleration_scaling_factor);
  }

  if (source.max_cartesian_speed > 0)
  {
    metadata.append(name_ + ".cartesian_speed_limited_link", source.cartesian_speed_limited_link);
    metadata.append(name_ + ".max_cartesian_speed", source.max_cartesian_speed);
  }

  return MoveItErrorCode::SUCCESS;
};

// GoalConstraintsFeatures.

GoalConstraintsFeatures::GoalConstraintsFeatures(double match_tolerance)
  : name_("GoalConstraintsFeatures"), match_tolerance_(match_tolerance)
{
}

std::string GoalConstraintsFeatures::getName() const
{
  return name_;
}

MoveItErrorCode GoalConstraintsFeatures::appendFeaturesAsFuzzyFetchQuery(Query& query, const MotionPlanRequest& source,
                                                                         const MoveGroupInterface& move_group,
                                                                         double exact_match_precision) const
{
  return appendFeaturesAsFetchQueryWithTolerance(query, source, move_group, match_tolerance_ + exact_match_precision);
};

MoveItErrorCode GoalConstraintsFeatures::appendFeaturesAsExactFetchQuery(Query& query, const MotionPlanRequest& source,
                                                                         const MoveGroupInterface& move_group,
                                                                         double exact_match_precision) const
{
  return appendFeaturesAsFetchQueryWithTolerance(query, source, move_group, exact_match_precision);
};

MoveItErrorCode GoalConstraintsFeatures::appendFeaturesAsInsertMetadata(Metadata& metadata,
                                                                        const MotionPlanRequest& source,
                                                                        const MoveGroupInterface& move_group) const
{
  std::string workspace_id = getWorkspaceFrameId(move_group, source.workspace_parameters);
  return appendConstraintsAsInsertMetadata(metadata, source.goal_constraints, move_group, workspace_id,
                                           name_ + ".goal_constraints");
};

MoveItErrorCode GoalConstraintsFeatures::appendFeaturesAsFetchQueryWithTolerance(Query& query,
                                                                                 const MotionPlanRequest& source,
                                                                                 const MoveGroupInterface& move_group,
                                                                                 double match_tolerance) const
{
  std::string workspace_id = getWorkspaceFrameId(move_group, source.workspace_parameters);
  return appendConstraintsAsFetchQueryWithTolerance(query, source.goal_constraints, move_group, match_tolerance,
                                                    workspace_id, name_ + ".goal_constraints");
};

// PathConstraintsFeatures.

PathConstraintsFeatures::PathConstraintsFeatures(double match_tolerance)
  : name_("PathConstraintsFeatures"), match_tolerance_(match_tolerance)
{
}

std::string PathConstraintsFeatures::getName() const
{
  return name_;
}

MoveItErrorCode PathConstraintsFeatures::appendFeaturesAsFuzzyFetchQuery(Query& query, const MotionPlanRequest& source,
                                                                         const MoveGroupInterface& move_group,
                                                                         double exact_match_precision) const
{
  return appendFeaturesAsFetchQueryWithTolerance(query, source, move_group, match_tolerance_ + exact_match_precision);
};

MoveItErrorCode PathConstraintsFeatures::appendFeaturesAsExactFetchQuery(Query& query, const MotionPlanRequest& source,
                                                                         const MoveGroupInterface& move_group,
                                                                         double exact_match_precision) const
{
  return appendFeaturesAsFetchQueryWithTolerance(query, source, move_group, exact_match_precision);
};

MoveItErrorCode PathConstraintsFeatures::appendFeaturesAsInsertMetadata(Metadata& metadata,
                                                                        const MotionPlanRequest& source,
                                                                        const MoveGroupInterface& move_group) const
{
  std::string workspace_id = getWorkspaceFrameId(move_group, source.workspace_parameters);
  return appendConstraintsAsInsertMetadata(metadata, { source.path_constraints }, move_group, workspace_id,
                                           name_ + ".path_constraints");
};

MoveItErrorCode PathConstraintsFeatures::appendFeaturesAsFetchQueryWithTolerance(Query& query,
                                                                                 const MotionPlanRequest& source,
                                                                                 const MoveGroupInterface& move_group,
                                                                                 double match_tolerance) const
{
  std::string workspace_id = getWorkspaceFrameId(move_group, source.workspace_parameters);
  return appendConstraintsAsFetchQueryWithTolerance(query, { source.path_constraints }, move_group, match_tolerance,
                                                    workspace_id, name_ + ".path_constraints");
};

// TrajectoryConstraintsFeatures.

TrajectoryConstraintsFeatures::TrajectoryConstraintsFeatures(double match_tolerance)
  : name_("TrajectoryConstraintsFeatures"), match_tolerance_(match_tolerance)
{
}

std::string TrajectoryConstraintsFeatures::getName() const
{
  return name_;
}

MoveItErrorCode TrajectoryConstraintsFeatures::appendFeaturesAsFuzzyFetchQuery(Query& query,
                                                                               const MotionPlanRequest& source,
                                                                               const MoveGroupInterface& move_group,
                                                                               double exact_match_precision) const
{
  return appendFeaturesAsFetchQueryWithTolerance(query, source, move_group, match_tolerance_ + exact_match_precision);
};

MoveItErrorCode TrajectoryConstraintsFeatures::appendFeaturesAsExactFetchQuery(Query& query,
                                                                               const MotionPlanRequest& source,
                                                                               const MoveGroupInterface& move_group,
                                                                               double exact_match_precision) const
{
  return appendFeaturesAsFetchQueryWithTolerance(query, source, move_group, exact_match_precision);
};

MoveItErrorCode
TrajectoryConstraintsFeatures::appendFeaturesAsInsertMetadata(Metadata& metadata, const MotionPlanRequest& source,
                                                              const MoveGroupInterface& move_group) const
{
  std::string workspace_id = getWorkspaceFrameId(move_group, source.workspace_parameters);
  return appendConstraintsAsInsertMetadata(metadata, source.trajectory_constraints.constraints, move_group,
                                           workspace_id, name_ + ".trajectory_constraints.constraints");
};

MoveItErrorCode TrajectoryConstraintsFeatures::appendFeaturesAsFetchQueryWithTolerance(
    Query& query, const MotionPlanRequest& source, const MoveGroupInterface& move_group, double match_tolerance) const
{
  std::string workspace_id = getWorkspaceFrameId(move_group, source.workspace_parameters);
  return appendConstraintsAsFetchQueryWithTolerance(query, source.trajectory_constraints.constraints, move_group,
                                                    match_tolerance, workspace_id,
                                                    name_ + ".trajectory_constraints.constraints");
};

}  // namespace trajectory_cache
}  // namespace moveit_ros
