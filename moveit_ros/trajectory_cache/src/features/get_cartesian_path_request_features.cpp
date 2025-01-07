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
 * @brief Implementation of GetCartesianPath::Request features to key the trajectory cache on.
 * @see FeaturesInterface<FeatureSourceT>
 *
 * @author methylDragon
 */

#include <rclcpp/logging.hpp>
#include <warehouse_ros/message_collection.h>

#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>

#include <moveit/trajectory_cache/features/get_cartesian_path_request_features.hpp>
#include <moveit/trajectory_cache/utils/utils.hpp>

namespace moveit_ros
{
namespace trajectory_cache
{

using ::warehouse_ros::Metadata;
using ::warehouse_ros::Query;

using ::moveit::core::MoveItErrorCode;
using ::moveit::planning_interface::MoveGroupInterface;

using ::moveit_msgs::srv::GetCartesianPath;

// "Start" features. ===============================================================================

// CartesianWorkspaceFeatures.

CartesianWorkspaceFeatures::CartesianWorkspaceFeatures() : name_("CartesianWorkspaceFeatures")
{
}

std::string CartesianWorkspaceFeatures::getName() const
{
  return name_;
}

MoveItErrorCode CartesianWorkspaceFeatures::appendFeaturesAsFuzzyFetchQuery(Query& query,
                                                                            const GetCartesianPath::Request& source,
                                                                            const MoveGroupInterface& move_group,
                                                                            double exact_match_precision) const
{
  return appendFeaturesAsExactFetchQuery(query, source, move_group, exact_match_precision);
};

MoveItErrorCode CartesianWorkspaceFeatures::appendFeaturesAsExactFetchQuery(Query& query,
                                                                            const GetCartesianPath::Request& source,
                                                                            const MoveGroupInterface& move_group,
                                                                            double /*exact_match_precision*/) const
{
  query.append(name_ + ".group_name", source.group_name);
  query.append(name_ + ".header.frame_id", getCartesianPathRequestFrameId(move_group, source));
  return moveit::core::MoveItErrorCode::SUCCESS;
};

MoveItErrorCode CartesianWorkspaceFeatures::appendFeaturesAsInsertMetadata(Metadata& metadata,
                                                                           const GetCartesianPath::Request& source,
                                                                           const MoveGroupInterface& move_group) const
{
  metadata.append(name_ + ".group_name", source.group_name);
  metadata.append(name_ + ".header.frame_id", getCartesianPathRequestFrameId(move_group, source));
  return moveit::core::MoveItErrorCode::SUCCESS;
};

// CartesianStartStateJointStateFeatures.

CartesianStartStateJointStateFeatures::CartesianStartStateJointStateFeatures(double match_tolerance)
  : name_("CartesianStartStateJointStateFeatures"), match_tolerance_(match_tolerance)
{
}

std::string CartesianStartStateJointStateFeatures::getName() const
{
  return name_;
}

MoveItErrorCode CartesianStartStateJointStateFeatures::appendFeaturesAsFuzzyFetchQuery(
    Query& query, const GetCartesianPath::Request& source, const MoveGroupInterface& move_group,
    double exact_match_precision) const
{
  return appendFeaturesAsFetchQueryWithTolerance(query, source, move_group, match_tolerance_ + exact_match_precision);
};

MoveItErrorCode CartesianStartStateJointStateFeatures::appendFeaturesAsExactFetchQuery(
    Query& query, const GetCartesianPath::Request& source, const MoveGroupInterface& move_group,
    double exact_match_precision) const
{
  return appendFeaturesAsFetchQueryWithTolerance(query, source, move_group, exact_match_precision);
};

MoveItErrorCode CartesianStartStateJointStateFeatures::appendFeaturesAsInsertMetadata(
    Metadata& metadata, const GetCartesianPath::Request& source, const MoveGroupInterface& move_group) const
{
  return appendRobotStateJointStateAsInsertMetadata(metadata, source.start_state, move_group, name_ + ".start_state");
};

MoveItErrorCode CartesianStartStateJointStateFeatures::appendFeaturesAsFetchQueryWithTolerance(
    Query& query, const GetCartesianPath::Request& source, const MoveGroupInterface& move_group,
    double match_tolerance) const
{
  return appendRobotStateJointStateAsFetchQueryWithTolerance(query, source.start_state, move_group, match_tolerance,
                                                             name_ + ".start_state");
};

// "Goal" features. ================================================================================

// CartesianMaxSpeedAndAccelerationFeatures.

CartesianMaxSpeedAndAccelerationFeatures::CartesianMaxSpeedAndAccelerationFeatures()
  : name_("CartesianMaxSpeedAndAccelerationFeatures")
{
}

std::string CartesianMaxSpeedAndAccelerationFeatures::getName() const
{
  return name_;
}

MoveItErrorCode CartesianMaxSpeedAndAccelerationFeatures::appendFeaturesAsFuzzyFetchQuery(
    Query& query, const GetCartesianPath::Request& source, const MoveGroupInterface& move_group,
    double exact_match_precision) const
{
  return appendFeaturesAsExactFetchQuery(query, source, move_group, exact_match_precision);
};

MoveItErrorCode CartesianMaxSpeedAndAccelerationFeatures::appendFeaturesAsExactFetchQuery(
    Query& query, const GetCartesianPath::Request& source, const MoveGroupInterface& /*move_group*/,
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

  return moveit::core::MoveItErrorCode::SUCCESS;
};

MoveItErrorCode CartesianMaxSpeedAndAccelerationFeatures::appendFeaturesAsInsertMetadata(
    Metadata& metadata, const GetCartesianPath::Request& source, const MoveGroupInterface& /*move_group*/) const
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

  return moveit::core::MoveItErrorCode::SUCCESS;
};

// CartesianMaxStepAndJumpThresholdFeatures.

CartesianMaxStepAndJumpThresholdFeatures::CartesianMaxStepAndJumpThresholdFeatures()
  : name_("CartesianMaxStepAndJumpThresholdFeatures")
{
}

std::string CartesianMaxStepAndJumpThresholdFeatures::getName() const
{
  return name_;
}

MoveItErrorCode CartesianMaxStepAndJumpThresholdFeatures::appendFeaturesAsFuzzyFetchQuery(
    Query& query, const GetCartesianPath::Request& source, const MoveGroupInterface& move_group,
    double exact_match_precision) const
{
  return appendFeaturesAsExactFetchQuery(query, source, move_group, exact_match_precision);
};

MoveItErrorCode CartesianMaxStepAndJumpThresholdFeatures::appendFeaturesAsExactFetchQuery(
    Query& query, const GetCartesianPath::Request& source, const MoveGroupInterface& /*move_group*/,
    double /*exact_match_precision*/) const
{
  query.appendLTE(name_ + ".max_step", source.max_step);

  if (source.jump_threshold > 0)
  {
    query.appendLTE(name_ + ".jump_threshold", source.jump_threshold);
  }
  if (source.prismatic_jump_threshold > 0)
  {
    query.appendLTE(name_ + ".prismatic_jump_threshold", source.prismatic_jump_threshold);
  }
  if (source.revolute_jump_threshold > 0)
  {
    query.appendLTE(name_ + ".revolute_jump_threshold", source.revolute_jump_threshold);
  }

  return moveit::core::MoveItErrorCode::SUCCESS;
};

MoveItErrorCode CartesianMaxStepAndJumpThresholdFeatures::appendFeaturesAsInsertMetadata(
    Metadata& metadata, const GetCartesianPath::Request& source, const MoveGroupInterface& /*move_group*/) const
{
  metadata.append(name_ + ".max_step", source.max_step);

  if (source.jump_threshold > 0)
  {
    metadata.append(name_ + ".jump_threshold", source.jump_threshold);
  }
  if (source.prismatic_jump_threshold > 0)
  {
    metadata.append(name_ + ".prismatic_jump_threshold", source.prismatic_jump_threshold);
  }
  if (source.revolute_jump_threshold > 0)
  {
    metadata.append(name_ + ".revolute_jump_threshold", source.revolute_jump_threshold);
  }

  return moveit::core::MoveItErrorCode::SUCCESS;
};

// CartesianWaypointsFeatures.

CartesianWaypointsFeatures::CartesianWaypointsFeatures(double match_tolerance)
  : name_("CartesianWaypointsFeatures"), match_tolerance_(match_tolerance)
{
}

std::string CartesianWaypointsFeatures::getName() const
{
  return name_;
}

MoveItErrorCode CartesianWaypointsFeatures::appendFeaturesAsFuzzyFetchQuery(Query& query,
                                                                            const GetCartesianPath::Request& source,
                                                                            const MoveGroupInterface& move_group,
                                                                            double exact_match_precision) const
{
  return appendFeaturesAsFetchQueryWithTolerance(query, source, move_group, match_tolerance_ + exact_match_precision);
};

MoveItErrorCode CartesianWaypointsFeatures::appendFeaturesAsExactFetchQuery(Query& query,
                                                                            const GetCartesianPath::Request& source,
                                                                            const MoveGroupInterface& move_group,
                                                                            double exact_match_precision) const
{
  return appendFeaturesAsFetchQueryWithTolerance(query, source, move_group, exact_match_precision);
};

MoveItErrorCode CartesianWaypointsFeatures::appendFeaturesAsInsertMetadata(Metadata& metadata,
                                                                           const GetCartesianPath::Request& source,
                                                                           const MoveGroupInterface& move_group) const
{
  std::string path_request_frame_id = getCartesianPathRequestFrameId(move_group, source);
  std::string base_frame = move_group.getRobotModel()->getModelFrame();

  metadata.append(name_ + ".link_name", source.link_name);
  metadata.append(name_ + ".robot_model.frame_id", base_frame);

  // Waypoints.

  size_t waypoint_idx = 0;
  for (const auto& waypoint : source.waypoints)
  {
    std::string meta_name = name_ + ".waypoints_" + std::to_string(waypoint_idx++);

    geometry_msgs::msg::Point canonical_position = waypoint.position;
    geometry_msgs::msg::Quaternion canonical_orientation = waypoint.orientation;

    // Canonicalize to robot base frame if necessary.
    if (path_request_frame_id != base_frame)
    {
      if (MoveItErrorCode status = restateInNewFrame(move_group.getTF(), path_request_frame_id, base_frame,
                                                     &canonical_position, &canonical_orientation, tf2::TimePointZero);
          status != MoveItErrorCode::SUCCESS)
      {
        // NOTE: methyldragon -
        //   Ideally we would restore the original state here and undo our changes, however copy of the query is not
        //   supported.
        std::stringstream ss;
        ss << "Skipping " << name_ << " metadata append: " << status.message;
        return MoveItErrorCode(status.val, status.message);
      }
    }

    // Position
    metadata.append(meta_name + ".position.x", canonical_position.x);
    metadata.append(meta_name + ".position.y", canonical_position.y);
    metadata.append(meta_name + ".position.z", canonical_position.z);

    // Orientation
    metadata.append(meta_name + ".orientation.x", canonical_orientation.x);
    metadata.append(meta_name + ".orientation.y", canonical_orientation.y);
    metadata.append(meta_name + ".orientation.z", canonical_orientation.z);
    metadata.append(meta_name + ".orientation.w", canonical_orientation.w);
  }

  return moveit::core::MoveItErrorCode::SUCCESS;
};

MoveItErrorCode CartesianWaypointsFeatures::appendFeaturesAsFetchQueryWithTolerance(
    Query& query, const GetCartesianPath::Request& source, const MoveGroupInterface& move_group,
    double match_tolerance) const
{
  std::string path_request_frame_id = getCartesianPathRequestFrameId(move_group, source);
  std::string base_frame = move_group.getRobotModel()->getModelFrame();

  query.append(name_ + ".link_name", source.link_name);
  query.append(name_ + ".robot_model.frame_id", base_frame);

  // Waypoints.

  size_t waypoint_idx = 0;
  for (const auto& waypoint : source.waypoints)
  {
    std::string meta_name = name_ + ".waypoints_" + std::to_string(waypoint_idx++);

    geometry_msgs::msg::Point canonical_position = waypoint.position;
    geometry_msgs::msg::Quaternion canonical_orientation = waypoint.orientation;

    // Canonicalize to robot base frame if necessary.
    if (path_request_frame_id != base_frame)
    {
      if (MoveItErrorCode status = restateInNewFrame(move_group.getTF(), path_request_frame_id, base_frame,
                                                     &canonical_position, &canonical_orientation, tf2::TimePointZero);
          status != MoveItErrorCode::SUCCESS)
      {
        // NOTE: methyldragon -
        //   Ideally we would restore the original state here and undo our changes, however copy of the query is not
        //   supported.
        std::stringstream ss;
        ss << "Skipping " << name_ << " query append: " << status.message;
        return MoveItErrorCode(status.val, status.message);
      }
    }

    // Position
    queryAppendCenterWithTolerance(query, meta_name + ".position.x", canonical_position.x, match_tolerance);
    queryAppendCenterWithTolerance(query, meta_name + ".position.y", canonical_position.y, match_tolerance);
    queryAppendCenterWithTolerance(query, meta_name + ".position.z", canonical_position.z, match_tolerance);

    // Orientation
    queryAppendCenterWithTolerance(query, meta_name + ".orientation.x", canonical_orientation.x, match_tolerance);
    queryAppendCenterWithTolerance(query, meta_name + ".orientation.y", canonical_orientation.y, match_tolerance);
    queryAppendCenterWithTolerance(query, meta_name + ".orientation.z", canonical_orientation.z, match_tolerance);
    queryAppendCenterWithTolerance(query, meta_name + ".orientation.w", canonical_orientation.w, match_tolerance);
  }

  return moveit::core::MoveItErrorCode::SUCCESS;
};

// CartesianPathConstraintsFeatures.

CartesianPathConstraintsFeatures::CartesianPathConstraintsFeatures(double match_tolerance)
  : name_("CartesianPathConstraintsFeatures"), match_tolerance_(match_tolerance)
{
}

std::string CartesianPathConstraintsFeatures::getName() const
{
  return name_;
}

MoveItErrorCode
CartesianPathConstraintsFeatures::appendFeaturesAsFuzzyFetchQuery(Query& query, const GetCartesianPath::Request& source,
                                                                  const MoveGroupInterface& move_group,
                                                                  double exact_match_precision) const
{
  return appendFeaturesAsFetchQueryWithTolerance(query, source, move_group, match_tolerance_ + exact_match_precision);
};

MoveItErrorCode
CartesianPathConstraintsFeatures::appendFeaturesAsExactFetchQuery(Query& query, const GetCartesianPath::Request& source,
                                                                  const MoveGroupInterface& move_group,
                                                                  double exact_match_precision) const
{
  return appendFeaturesAsFetchQueryWithTolerance(query, source, move_group, exact_match_precision);
};

MoveItErrorCode CartesianPathConstraintsFeatures::appendFeaturesAsInsertMetadata(
    Metadata& metadata, const GetCartesianPath::Request& source, const MoveGroupInterface& move_group) const
{
  return appendConstraintsAsInsertMetadata(metadata, { source.path_constraints }, move_group,
                                           getCartesianPathRequestFrameId(move_group, source),
                                           name_ + ".path_constraints");
};

MoveItErrorCode CartesianPathConstraintsFeatures::appendFeaturesAsFetchQueryWithTolerance(
    Query& query, const GetCartesianPath::Request& source, const MoveGroupInterface& move_group,
    double match_tolerance) const
{
  return appendConstraintsAsFetchQueryWithTolerance(query, { source.path_constraints }, move_group, match_tolerance,
                                                    getCartesianPathRequestFrameId(move_group, source),
                                                    name_ + ".path_constraints");
};

}  // namespace trajectory_cache
}  // namespace moveit_ros
