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
 * @brief Implementation of moveit_msgs::srv::GetCartesianPath::Request features to key the trajectory cache on.
 * @see FeaturesInterface<FeatureSourceT>
 *
 * @author methylDragon
 */

#include <rclcpp/logging.hpp>
#include <warehouse_ros/message_collection.h>

#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/srv/get_cartesian_path.hpp>

#include <moveit/trajectory_cache/features/get_cartesian_path_request_features.hpp>
#include <moveit/trajectory_cache/utils/utils.hpp>

namespace moveit_ros
{
namespace trajectory_cache
{

// "Start" features. ===============================================================================

// CartesianWorkspaceFeatures.

CartesianWorkspaceFeatures::CartesianWorkspaceFeatures() : name_("CartesianWorkspaceFeatures")
{
}

std::string CartesianWorkspaceFeatures::getName() const
{
  return name_;
}

moveit::core::MoveItErrorCode CartesianWorkspaceFeatures::appendFeaturesAsFuzzyFetchQuery(
    warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
    const moveit::planning_interface::MoveGroupInterface& move_group, double exact_match_precision) const
{
  return appendFeaturesAsExactFetchQuery(query, source, move_group, exact_match_precision);
};

moveit::core::MoveItErrorCode CartesianWorkspaceFeatures::appendFeaturesAsExactFetchQuery(
    warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
    const moveit::planning_interface::MoveGroupInterface& move_group, double /*exact_match_precision*/) const
{
  query.append(name_ + ".group_name", source.group_name);
  query.append(name_ + ".header.frame_id", getCartesianPathRequestFrameId(move_group, source));
  return moveit::core::MoveItErrorCode::SUCCESS;
};

moveit::core::MoveItErrorCode CartesianWorkspaceFeatures::appendFeaturesAsInsertMetadata(
    warehouse_ros::Metadata& metadata, const moveit_msgs::srv::GetCartesianPath::Request& source,
    const moveit::planning_interface::MoveGroupInterface& move_group) const
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

moveit::core::MoveItErrorCode CartesianStartStateJointStateFeatures::appendFeaturesAsFuzzyFetchQuery(
    warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
    const moveit::planning_interface::MoveGroupInterface& move_group, double exact_match_precision) const
{
  return appendFeaturesAsFetchQueryWithTolerance(query, source, move_group, match_tolerance_ + exact_match_precision);
};

moveit::core::MoveItErrorCode CartesianStartStateJointStateFeatures::appendFeaturesAsExactFetchQuery(
    warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
    const moveit::planning_interface::MoveGroupInterface& move_group, double exact_match_precision) const
{
  return appendFeaturesAsFetchQueryWithTolerance(query, source, move_group, exact_match_precision);
};

moveit::core::MoveItErrorCode CartesianStartStateJointStateFeatures::appendFeaturesAsInsertMetadata(
    warehouse_ros::Metadata& metadata, const moveit_msgs::srv::GetCartesianPath::Request& source,
    const moveit::planning_interface::MoveGroupInterface& move_group) const
{
  return appendRobotStateJointStateAsInsertMetadata(metadata, source.start_state, move_group, name_ + ".start_state");
};

moveit::core::MoveItErrorCode CartesianStartStateJointStateFeatures::appendFeaturesAsFetchQueryWithTolerance(
    warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
    const moveit::planning_interface::MoveGroupInterface& move_group, double match_tolerance) const
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

moveit::core::MoveItErrorCode CartesianMaxSpeedAndAccelerationFeatures::appendFeaturesAsFuzzyFetchQuery(
    warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
    const moveit::planning_interface::MoveGroupInterface& move_group, double exact_match_precision) const
{
  return appendFeaturesAsExactFetchQuery(query, source, move_group, exact_match_precision);
};

moveit::core::MoveItErrorCode CartesianMaxSpeedAndAccelerationFeatures::appendFeaturesAsExactFetchQuery(
    warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
    const moveit::planning_interface::MoveGroupInterface& /*move_group*/, double /*exact_match_precision*/) const
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

moveit::core::MoveItErrorCode CartesianMaxSpeedAndAccelerationFeatures::appendFeaturesAsInsertMetadata(
    warehouse_ros::Metadata& metadata, const moveit_msgs::srv::GetCartesianPath::Request& source,
    const moveit::planning_interface::MoveGroupInterface& /*move_group*/) const
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

moveit::core::MoveItErrorCode CartesianMaxStepAndJumpThresholdFeatures::appendFeaturesAsFuzzyFetchQuery(
    warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
    const moveit::planning_interface::MoveGroupInterface& move_group, double exact_match_precision) const
{
  return appendFeaturesAsExactFetchQuery(query, source, move_group, exact_match_precision);
};

moveit::core::MoveItErrorCode CartesianMaxStepAndJumpThresholdFeatures::appendFeaturesAsExactFetchQuery(
    warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
    const moveit::planning_interface::MoveGroupInterface& /*move_group*/, double /*exact_match_precision*/) const
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

moveit::core::MoveItErrorCode CartesianMaxStepAndJumpThresholdFeatures::appendFeaturesAsInsertMetadata(
    warehouse_ros::Metadata& metadata, const moveit_msgs::srv::GetCartesianPath::Request& source,
    const moveit::planning_interface::MoveGroupInterface& /*move_group*/) const
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

moveit::core::MoveItErrorCode CartesianWaypointsFeatures::appendFeaturesAsFuzzyFetchQuery(
    warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
    const moveit::planning_interface::MoveGroupInterface& move_group, double exact_match_precision) const
{
  return appendFeaturesAsFetchQueryWithTolerance(query, source, move_group, match_tolerance_ + exact_match_precision);
};

moveit::core::MoveItErrorCode CartesianWaypointsFeatures::appendFeaturesAsExactFetchQuery(
    warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
    const moveit::planning_interface::MoveGroupInterface& move_group, double exact_match_precision) const
{
  return appendFeaturesAsFetchQueryWithTolerance(query, source, move_group, exact_match_precision);
};

moveit::core::MoveItErrorCode CartesianWaypointsFeatures::appendFeaturesAsInsertMetadata(
    warehouse_ros::Metadata& metadata, const moveit_msgs::srv::GetCartesianPath::Request& source,
    const moveit::planning_interface::MoveGroupInterface& move_group) const
{
  std::string path_request_frame_id = getCartesianPathRequestFrameId(move_group, source);
  std::string base_frame = move_group.getRobotModel()->getModelFrame();

  metadata.append(name_ + ".link_name", source.link_name);
  metadata.append(name_ + ".robot_model.frame_id", base_frame);

  // Waypoints.

  // Restating them in terms of the robot model frame (usually base_link)
  double x_offset = 0;
  double y_offset = 0;
  double z_offset = 0;

  geometry_msgs::msg::Quaternion quat_offset;
  quat_offset.x = 0;
  quat_offset.y = 0;
  quat_offset.z = 0;
  quat_offset.w = 1;

  if (base_frame != path_request_frame_id)
  {
    try
    {
      auto transform = move_group.getTF()->lookupTransform(path_request_frame_id, base_frame, tf2::TimePointZero);
      x_offset = transform.transform.translation.x;
      y_offset = transform.transform.translation.y;
      z_offset = transform.transform.translation.z;
      quat_offset = transform.transform.rotation;
    }
    catch (tf2::TransformException& ex)
    {
      // NOTE: methyldragon -
      //   Ideally we would restore the original state here and undo our changes, however copy of the query is not
      //   supported.
      std::stringstream ss;
      ss << "Skipping " << name_ << " metadata append: " << "Could not get transform for translation " << base_frame
         << " to " << path_request_frame_id << ": " << ex.what();
      return moveit::core::MoveItErrorCode(moveit_msgs::msg::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE, ss.str());
    }
  }

  tf2::Quaternion tf2_quat_frame_offset(quat_offset.x, quat_offset.y, quat_offset.z, quat_offset.w);
  tf2_quat_frame_offset.normalize();

  size_t waypoint_idx = 0;
  for (auto& waypoint : source.waypoints)
  {
    std::string meta_name = name_ + ".waypoints_" + std::to_string(waypoint_idx++);

    // Apply offsets
    // Position
    metadata.append(meta_name + ".position.x", x_offset + waypoint.position.x);
    metadata.append(meta_name + ".position.y", y_offset + waypoint.position.y);
    metadata.append(meta_name + ".position.z", z_offset + waypoint.position.z);

    // Orientation
    tf2::Quaternion tf2_quat_goal_offset(waypoint.orientation.x, waypoint.orientation.y, waypoint.orientation.z,
                                         waypoint.orientation.w);
    tf2_quat_goal_offset.normalize();

    auto final_quat = tf2_quat_goal_offset * tf2_quat_frame_offset;
    final_quat.normalize();

    metadata.append(meta_name + ".orientation.x", final_quat.getX());
    metadata.append(meta_name + ".orientation.y", final_quat.getY());
    metadata.append(meta_name + ".orientation.z", final_quat.getZ());
    metadata.append(meta_name + ".orientation.w", final_quat.getW());
  }

  return moveit::core::MoveItErrorCode::SUCCESS;
};

moveit::core::MoveItErrorCode CartesianWaypointsFeatures::appendFeaturesAsFetchQueryWithTolerance(
    warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
    const moveit::planning_interface::MoveGroupInterface& move_group, double match_tolerance) const
{
  std::string path_request_frame_id = getCartesianPathRequestFrameId(move_group, source);
  std::string base_frame = move_group.getRobotModel()->getModelFrame();

  query.append(name_ + ".link_name", source.link_name);
  query.append(name_ + ".robot_model.frame_id", base_frame);

  // Waypoints.

  // Restating them in terms of the robot model frame (usually base_link)
  double x_offset = 0;
  double y_offset = 0;
  double z_offset = 0;

  geometry_msgs::msg::Quaternion quat_offset;
  quat_offset.x = 0;
  quat_offset.y = 0;
  quat_offset.z = 0;
  quat_offset.w = 1;

  if (base_frame != path_request_frame_id)
  {
    try
    {
      auto transform = move_group.getTF()->lookupTransform(path_request_frame_id, base_frame, tf2::TimePointZero);
      x_offset = transform.transform.translation.x;
      y_offset = transform.transform.translation.y;
      z_offset = transform.transform.translation.z;
      quat_offset = transform.transform.rotation;
    }
    catch (tf2::TransformException& ex)
    {
      // NOTE: methyldragon -
      //   Ideally we would restore the original state here and undo our changes, however copy of the query is not
      //   supported.
      std::stringstream ss;
      ss << "Skipping " << name_ << " query append: " << "Could not get transform for translation " << base_frame
         << " to " << path_request_frame_id << ": " << ex.what();
      return moveit::core::MoveItErrorCode(moveit_msgs::msg::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE, ss.str());
    }
  }

  tf2::Quaternion tf2_quat_frame_offset(quat_offset.x, quat_offset.y, quat_offset.z, quat_offset.w);
  tf2_quat_frame_offset.normalize();

  size_t waypoint_idx = 0;
  for (auto& waypoint : source.waypoints)
  {
    std::string meta_name = name_ + ".waypoints_" + std::to_string(waypoint_idx++);

    // Apply offsets
    // Position
    queryAppendCenterWithTolerance(query, meta_name + ".position.x", x_offset + waypoint.position.x, match_tolerance);
    queryAppendCenterWithTolerance(query, meta_name + ".position.y", y_offset + waypoint.position.y, match_tolerance);
    queryAppendCenterWithTolerance(query, meta_name + ".position.z", z_offset + waypoint.position.z, match_tolerance);

    // Orientation
    tf2::Quaternion tf2_quat_goal_offset(waypoint.orientation.x, waypoint.orientation.y, waypoint.orientation.z,
                                         waypoint.orientation.w);
    tf2_quat_goal_offset.normalize();

    auto final_quat = tf2_quat_goal_offset * tf2_quat_frame_offset;
    final_quat.normalize();

    queryAppendCenterWithTolerance(query, meta_name + ".orientation.x", final_quat.getX(), match_tolerance);
    queryAppendCenterWithTolerance(query, meta_name + ".orientation.y", final_quat.getY(), match_tolerance);
    queryAppendCenterWithTolerance(query, meta_name + ".orientation.z", final_quat.getZ(), match_tolerance);
    queryAppendCenterWithTolerance(query, meta_name + ".orientation.w", final_quat.getW(), match_tolerance);
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

moveit::core::MoveItErrorCode CartesianPathConstraintsFeatures::appendFeaturesAsFuzzyFetchQuery(
    warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
    const moveit::planning_interface::MoveGroupInterface& move_group, double exact_match_precision) const
{
  return appendFeaturesAsFetchQueryWithTolerance(query, source, move_group, match_tolerance_ + exact_match_precision);
};

moveit::core::MoveItErrorCode CartesianPathConstraintsFeatures::appendFeaturesAsExactFetchQuery(
    warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
    const moveit::planning_interface::MoveGroupInterface& move_group, double exact_match_precision) const
{
  return appendFeaturesAsFetchQueryWithTolerance(query, source, move_group, exact_match_precision);
};

moveit::core::MoveItErrorCode CartesianPathConstraintsFeatures::appendFeaturesAsInsertMetadata(
    warehouse_ros::Metadata& metadata, const moveit_msgs::srv::GetCartesianPath::Request& source,
    const moveit::planning_interface::MoveGroupInterface& move_group) const
{
  return appendConstraintsAsInsertMetadata(metadata, { source.path_constraints }, move_group,
                                           getCartesianPathRequestFrameId(move_group, source),
                                           name_ + ".path_constraints");
};

moveit::core::MoveItErrorCode CartesianPathConstraintsFeatures::appendFeaturesAsFetchQueryWithTolerance(
    warehouse_ros::Query& query, const moveit_msgs::srv::GetCartesianPath::Request& source,
    const moveit::planning_interface::MoveGroupInterface& move_group, double match_tolerance) const
{
  return appendConstraintsAsFetchQueryWithTolerance(query, { source.path_constraints }, move_group, match_tolerance,
                                                    getCartesianPathRequestFrameId(move_group, source),
                                                    name_ + ".path_constraints");
};

}  // namespace trajectory_cache
}  // namespace moveit_ros
