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
 * @brief Implementation of the utilities used by the trajectory_cache package.
 * @author methylDragon
 */

#include <sstream>
#include <string>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>

#include <tf2_ros/buffer.h>

#include <warehouse_ros/message_collection.h>

#include <moveit/trajectory_cache/utils/utils.hpp>

namespace
{

rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.ros.trajectory_cache.features");
}

}  // namespace

// Frames. =========================================================================================

namespace moveit_ros
{
namespace trajectory_cache
{

using ::warehouse_ros::Metadata;
using ::warehouse_ros::Query;

using ::moveit::core::MoveItErrorCode;
using ::moveit::planning_interface::MoveGroupInterface;

using ::moveit_msgs::srv::GetCartesianPath;

std::string getWorkspaceFrameId(const MoveGroupInterface& move_group,
                                const moveit_msgs::msg::WorkspaceParameters& workspace_parameters)
{
  if (workspace_parameters.header.frame_id.empty())
  {
    return move_group.getRobotModel()->getModelFrame();
  }
  else
  {
    return workspace_parameters.header.frame_id;
  }
}

std::string getCartesianPathRequestFrameId(const MoveGroupInterface& move_group,
                                           const GetCartesianPath::Request& path_request)
{
  if (path_request.header.frame_id.empty())
  {
    return move_group.getPoseReferenceFrame();
  }
  else
  {
    return path_request.header.frame_id;
  }
}

MoveItErrorCode restateInNewFrame(const std::shared_ptr<tf2_ros::Buffer>& tf, const std::string& target_frame,
                                  const std::string& source_frame, geometry_msgs::msg::Point* translation,
                                  geometry_msgs::msg::Quaternion* rotation, const tf2::TimePoint& lookup_time)
{
  if (target_frame == source_frame)
  {
    return MoveItErrorCode::SUCCESS;
  }
  if (translation == nullptr && rotation == nullptr)
  {
    return MoveItErrorCode::SUCCESS;
  }

  // Fetch transforms.
  geometry_msgs::msg::TransformStamped transform;
  try
  {
    transform = tf->lookupTransform(target_frame, source_frame, lookup_time);
  }
  catch (tf2::TransformException& ex)
  {
    std::stringstream ss;
    ss << "Could not get transform for " << source_frame << " to " << target_frame << ": " << ex.what();
    return moveit::core::MoveItErrorCode(moveit_msgs::msg::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE, ss.str());
  }

  // Translation.
  if (translation != nullptr)
  {
    translation->x += transform.transform.translation.x;
    translation->y += transform.transform.translation.y;
    translation->z += transform.transform.translation.z;
  }

  // Rotation.
  if (rotation != nullptr)
  {
    tf2::Quaternion input_quat(rotation->x, rotation->y, rotation->z, rotation->w);
    tf2::Quaternion transform_quat(transform.transform.rotation.x, transform.transform.rotation.y,
                                   transform.transform.rotation.z, transform.transform.rotation.w);

    input_quat.normalize();
    transform_quat.normalize();

    auto final_quat = input_quat * transform_quat;
    final_quat.normalize();

    rotation->x = final_quat.getX();
    rotation->y = final_quat.getY();
    rotation->z = final_quat.getZ();
    rotation->w = final_quat.getW();
  }

  return moveit::core::MoveItErrorCode::SUCCESS;
}

// Execution Time. =================================================================================

double getExecutionTime(const moveit_msgs::msg::RobotTrajectory& trajectory)
{
  return rclcpp::Duration(trajectory.joint_trajectory.points.back().time_from_start).seconds();
}

// Request Construction. ===========================================================================

GetCartesianPath::Request constructGetCartesianPathRequest(MoveGroupInterface& move_group,
                                                           const std::vector<geometry_msgs::msg::Pose>& waypoints,
                                                           double max_step, double jump_threshold,
                                                           bool avoid_collisions)
{
  GetCartesianPath::Request out;

  move_group.constructRobotState(out.start_state);

  out.group_name = move_group.getName();
  out.max_velocity_scaling_factor = move_group.getMaxVelocityScalingFactor();
  out.max_acceleration_scaling_factor = move_group.getMaxVelocityScalingFactor();

  out.header.frame_id = move_group.getPoseReferenceFrame();
  out.waypoints = waypoints;
  out.max_step = max_step;
  out.jump_threshold = jump_threshold;
  out.path_constraints = moveit_msgs::msg::Constraints();
  out.avoid_collisions = avoid_collisions;
  out.link_name = move_group.getEndEffectorLink();
  out.header.stamp = move_group.getNode()->now();

  return out;
}

// Queries. ========================================================================================

void queryAppendCenterWithTolerance(Query& query, const std::string& name, double center, double tolerance)
{
  query.appendRangeInclusive(name, center - tolerance / 2, center + tolerance / 2);
}

// Constraints. ====================================================================================

void sortJointConstraints(std::vector<moveit_msgs::msg::JointConstraint>& joint_constraints)
{
  std::sort(joint_constraints.begin(), joint_constraints.end(),
            [](const moveit_msgs::msg::JointConstraint& l, const moveit_msgs::msg::JointConstraint& r) {
              return l.joint_name < r.joint_name;
            });
}

void sortPositionConstraints(std::vector<moveit_msgs::msg::PositionConstraint>& position_constraints)
{
  std::sort(position_constraints.begin(), position_constraints.end(),
            [](const moveit_msgs::msg::PositionConstraint& l, const moveit_msgs::msg::PositionConstraint& r) {
              return l.link_name < r.link_name;
            });
}

void sortOrientationConstraints(std::vector<moveit_msgs::msg::OrientationConstraint>& orientation_constraints)
{
  std::sort(orientation_constraints.begin(), orientation_constraints.end(),
            [](const moveit_msgs::msg::OrientationConstraint& l, const moveit_msgs::msg::OrientationConstraint& r) {
              return l.link_name < r.link_name;
            });
}

moveit::core::MoveItErrorCode
appendConstraintsAsFetchQueryWithTolerance(Query& query, std::vector<moveit_msgs::msg::Constraints> constraints,
                                           const MoveGroupInterface& move_group, double match_tolerance,
                                           const std::string& reference_frame_id, const std::string& prefix)
{
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer = move_group.getTF();  // NOLINT: Deliberate lifetime extension.

  // Make ignored members explicit.

  bool emit_position_constraint_warning = false;
  for (const auto& constraint : constraints)
  {
    for (const auto& position_constraint : constraint.position_constraints)
    {
      if (!position_constraint.constraint_region.primitives.empty())
      {
        emit_position_constraint_warning = true;
        break;
      }
    }
    if (emit_position_constraint_warning)
    {
      break;
    }
  }
  if (emit_position_constraint_warning)
  {
    RCLCPP_WARN_STREAM(getLogger(), "Ignoring " << prefix << ".position_constraints.constraint_region: Not supported.");
  }

  bool emit_visibility_constraint_warning = false;
  for (const auto& constraint : constraints)
  {
    if (!constraint.visibility_constraints.empty())
    {
      emit_visibility_constraint_warning = true;
      break;
    }
  }
  if (emit_visibility_constraint_warning)
  {
    RCLCPP_WARN_STREAM(getLogger(), "Ignoring " << prefix << ".visibility_constraints: Not supported.");
  }

  // Begin extraction.

  size_t constraint_idx = 0;
  for (auto& constraint : constraints)  // Non-const as constraints are sorted in-place.
  {
    // We sort to avoid cardinality.
    sortJointConstraints(constraint.joint_constraints);
    sortPositionConstraints(constraint.position_constraints);
    sortOrientationConstraints(constraint.orientation_constraints);

    std::string constraint_prefix = prefix + "_" + std::to_string(constraint_idx++);

    // Joint constraints
    size_t joint_idx = 0;
    for (const auto& joint_constraint : constraint.joint_constraints)
    {
      std::string query_name = constraint_prefix + ".joint_constraints_" + std::to_string(joint_idx++);
      query.append(query_name + ".joint_name", joint_constraint.joint_name);
      queryAppendCenterWithTolerance(query, query_name + ".position", joint_constraint.position, match_tolerance);
      query.appendGTE(query_name + ".tolerance_above", joint_constraint.tolerance_above);
      query.appendLTE(query_name + ".tolerance_below", joint_constraint.tolerance_below);
    }

    // Position constraints
    if (!constraint.position_constraints.empty())
    {
      // All offsets will be "frozen" and computed wrt. the workspace frame instead.
      query.append(constraint_prefix + ".position_constraints.header.frame_id", reference_frame_id);

      size_t position_idx = 0;
      for (const auto& position_constraint : constraint.position_constraints)
      {
        std::string query_name = constraint_prefix + ".position_constraints_" + std::to_string(position_idx++);

        geometry_msgs::msg::Point canonical_position;
        canonical_position.x = position_constraint.target_point_offset.x;
        canonical_position.y = position_constraint.target_point_offset.y;
        canonical_position.z = position_constraint.target_point_offset.z;

        // Canonicalize to robot base frame if necessary.
        if (position_constraint.header.frame_id != reference_frame_id)
        {
          if (MoveItErrorCode status =
                  restateInNewFrame(move_group.getTF(), position_constraint.header.frame_id, reference_frame_id,
                                    &canonical_position, /*rotation=*/nullptr, tf2::TimePointZero);
              status != MoveItErrorCode::SUCCESS)
          {
            // NOTE: methyldragon -
            //   Ideally we would restore the original state here and undo our changes, however copy of the query is not
            //   supported.
            std::stringstream ss;
            ss << "Skipping " << prefix << ":" << query_name << " query append: " << status.message;
            return MoveItErrorCode(status.val, status.message);
          }
        }

        query.append(query_name + ".link_name", position_constraint.link_name);
        queryAppendCenterWithTolerance(query, query_name + ".target_point_offset.x", canonical_position.x,
                                       match_tolerance);
        queryAppendCenterWithTolerance(query, query_name + ".target_point_offset.y", canonical_position.y,
                                       match_tolerance);
        queryAppendCenterWithTolerance(query, query_name + ".target_point_offset.z", canonical_position.z,
                                       match_tolerance);
      }
    }

    // Orientation constraints
    if (!constraint.orientation_constraints.empty())
    {
      // All offsets will be "frozen" and computed wrt. the workspace frame instead.
      query.append(constraint_prefix + ".orientation_constraints.header.frame_id", reference_frame_id);

      size_t ori_idx = 0;
      for (const auto& orientation_constraint : constraint.orientation_constraints)
      {
        std::string query_name = constraint_prefix + ".orientation_constraints_" + std::to_string(ori_idx++);
        geometry_msgs::msg::Quaternion canonical_orientation = orientation_constraint.orientation;

        // Canonicalize to robot base frame if necessary.
        if (orientation_constraint.header.frame_id != reference_frame_id)
        {
          if (MoveItErrorCode status =
                  restateInNewFrame(move_group.getTF(), orientation_constraint.header.frame_id, reference_frame_id,
                                    /*translation=*/nullptr, &canonical_orientation, tf2::TimePointZero);
              status != MoveItErrorCode::SUCCESS)
          {
            // NOTE: methyldragon -
            //   Ideally we would restore the original state here and undo our changes, however copy of the query is not
            //   supported.
            std::stringstream ss;
            ss << "Skipping " << prefix << ":" << query_name << " query append: " << status.message;
            return MoveItErrorCode(status.val, status.message);
          }
        }

        query.append(query_name + ".link_name", orientation_constraint.link_name);
        queryAppendCenterWithTolerance(query, query_name + ".orientation.x", canonical_orientation.x, match_tolerance);
        queryAppendCenterWithTolerance(query, query_name + ".orientation.y", canonical_orientation.y, match_tolerance);
        queryAppendCenterWithTolerance(query, query_name + ".orientation.z", canonical_orientation.z, match_tolerance);
        queryAppendCenterWithTolerance(query, query_name + ".orientation.w", canonical_orientation.w, match_tolerance);
      }
    }
  }

  return moveit::core::MoveItErrorCode::SUCCESS;
}

moveit::core::MoveItErrorCode appendConstraintsAsInsertMetadata(Metadata& metadata,
                                                                std::vector<moveit_msgs::msg::Constraints> constraints,
                                                                const MoveGroupInterface& move_group,
                                                                const std::string& workspace_frame_id,
                                                                const std::string& prefix)
{
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer = move_group.getTF();  // NOLINT: Deliberate lifetime extension.

  // Make ignored members explicit

  bool emit_position_constraint_warning = false;
  for (const auto& constraint : constraints)
  {
    for (const auto& position_constraint : constraint.position_constraints)
    {
      if (!position_constraint.constraint_region.primitives.empty())
      {
        emit_position_constraint_warning = true;
        break;
      }
    }
    if (emit_position_constraint_warning)
    {
      break;
    }
  }
  if (emit_position_constraint_warning)
  {
    RCLCPP_WARN_STREAM(getLogger(), "Ignoring " << prefix << ".position_constraints.constraint_region: Not supported.");
  }

  bool emit_visibility_constraint_warning = false;
  for (const auto& constraint : constraints)
  {
    if (!constraint.visibility_constraints.empty())
    {
      emit_visibility_constraint_warning = true;
      break;
    }
  }
  if (emit_visibility_constraint_warning)
  {
    RCLCPP_WARN_STREAM(getLogger(), "Ignoring " << prefix << ".visibility_constraints: Not supported.");
  }

  // Begin extraction.

  size_t constraint_idx = 0;
  for (auto& constraint : constraints)  // Non-const as constraints are sorted in-place.
  {
    // We sort to avoid cardinality.
    sortJointConstraints(constraint.joint_constraints);
    sortPositionConstraints(constraint.position_constraints);
    sortOrientationConstraints(constraint.orientation_constraints);

    std::string constraint_prefix = prefix + "_" + std::to_string(constraint_idx++);

    // Joint constraints
    size_t joint_idx = 0;
    for (const auto& joint_constraint : constraint.joint_constraints)
    {
      std::string meta_name = constraint_prefix + ".joint_constraints_" + std::to_string(joint_idx++);
      metadata.append(meta_name + ".joint_name", joint_constraint.joint_name);
      metadata.append(meta_name + ".position", joint_constraint.position);
      metadata.append(meta_name + ".tolerance_above", joint_constraint.tolerance_above);
      metadata.append(meta_name + ".tolerance_below", joint_constraint.tolerance_below);
    }

    // Position constraints
    if (!constraint.position_constraints.empty())
    {
      // All offsets will be "frozen" and computed wrt. the workspace frame instead.
      metadata.append(constraint_prefix + ".position_constraints.header.frame_id", workspace_frame_id);

      size_t position_idx = 0;
      for (const auto& position_constraint : constraint.position_constraints)
      {
        std::string meta_name = constraint_prefix + ".position_constraints_" + std::to_string(position_idx++);

        geometry_msgs::msg::Point canonical_position;
        canonical_position.x = position_constraint.target_point_offset.x;
        canonical_position.y = position_constraint.target_point_offset.y;
        canonical_position.z = position_constraint.target_point_offset.z;

        // Canonicalize to robot base frame if necessary.
        if (position_constraint.header.frame_id != workspace_frame_id)
        {
          if (MoveItErrorCode status =
                  restateInNewFrame(move_group.getTF(), position_constraint.header.frame_id, workspace_frame_id,
                                    &canonical_position, /*rotation=*/nullptr, tf2::TimePointZero);
              status != MoveItErrorCode::SUCCESS)
          {
            // NOTE: methyldragon -
            //   Ideally we would restore the original state here and undo our changes, however copy of the query is not
            //   supported.
            std::stringstream ss;
            ss << "Skipping " << prefix << ":" << meta_name << " metadata append: " << status.message;
            return MoveItErrorCode(status.val, status.message);
          }
        }

        metadata.append(meta_name + ".link_name", position_constraint.link_name);
        metadata.append(meta_name + ".target_point_offset.x", canonical_position.x);
        metadata.append(meta_name + ".target_point_offset.y", canonical_position.y);
        metadata.append(meta_name + ".target_point_offset.z", canonical_position.z);
      }
    }

    // Orientation constraints
    if (!constraint.orientation_constraints.empty())
    {
      // All offsets will be "frozen" and computed wrt. the workspace frame instead.
      metadata.append(constraint_prefix + ".orientation_constraints.header.frame_id", workspace_frame_id);

      size_t ori_idx = 0;
      for (const auto& orientation_constraint : constraint.orientation_constraints)
      {
        std::string meta_name = constraint_prefix + ".orientation_constraints_" + std::to_string(ori_idx++);
        geometry_msgs::msg::Quaternion canonical_orientation = orientation_constraint.orientation;

        // Canonicalize to robot base frame if necessary.
        if (orientation_constraint.header.frame_id != workspace_frame_id)
        {
          if (MoveItErrorCode status =
                  restateInNewFrame(move_group.getTF(), orientation_constraint.header.frame_id, workspace_frame_id,
                                    /*translation=*/nullptr, &canonical_orientation, tf2::TimePointZero);
              status != MoveItErrorCode::SUCCESS)
          {
            // NOTE: methyldragon -
            //   Ideally we would restore the original state here and undo our changes, however copy of the query is not
            //   supported.
            std::stringstream ss;
            ss << "Skipping " << prefix << ":" << meta_name << " metadata append: " << status.message;
            return MoveItErrorCode(status.val, status.message);
          }
        }

        metadata.append(meta_name + ".link_name", orientation_constraint.link_name);
        metadata.append(meta_name + ".orientation.x", canonical_orientation.x);
        metadata.append(meta_name + ".orientation.y", canonical_orientation.y);
        metadata.append(meta_name + ".orientation.z", canonical_orientation.z);
        metadata.append(meta_name + ".orientation.w", canonical_orientation.w);
      }
    }
  }

  return moveit::core::MoveItErrorCode::SUCCESS;
}

// RobotState. =====================================================================================

moveit::core::MoveItErrorCode
appendRobotStateJointStateAsFetchQueryWithTolerance(Query& query, const moveit_msgs::msg::RobotState& robot_state,
                                                    const MoveGroupInterface& move_group, double match_tolerance,
                                                    const std::string& prefix)
{
  // Make ignored members explicit

  if (!robot_state.multi_dof_joint_state.joint_names.empty())
  {
    RCLCPP_WARN_STREAM(getLogger(), "Ignoring " << prefix << ".multi_dof_joint_states: Not supported.");
  }
  if (!robot_state.attached_collision_objects.empty())
  {
    RCLCPP_WARN_STREAM(getLogger(), "Ignoring " << prefix << ".attached_collision_objects: Not supported.");
  }

  // Begin extraction.

  if (robot_state.is_diff)
  {
    // If plan request states that the start_state is_diff, then we need to get
    // the current state from the move_group.

    // NOTE: methyldragon -
    //   I think if is_diff is on, the joint states will not be populated in all
    //   of the motion plan requests? If this isn't the case we might need to
    //   apply the joint states as offsets as well.
    //
    // TODO: Since MoveIt also potentially does another getCurrentState() call
    //   when planning, there is a chance that the current state in the cache
    //   differs from the state used in MoveIt's plan.
    //
    //   This issue should go away once the class is used within the move group's
    //   Plan call.
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    if (!current_state)
    {
      // NOTE: methyldragon -
      //   Ideally we would restore the original state here and undo our changes, however copy of the metadata is not
      //   supported.
      std::stringstream ss;
      ss << "Skipping " << prefix << " query append: "
         << "Could not get robot state.";
      return MoveItErrorCode(MoveItErrorCode::UNABLE_TO_AQUIRE_SENSOR_DATA, ss.str());
    }

    moveit_msgs::msg::RobotState current_state_msg;
    robotStateToRobotStateMsg(*current_state, current_state_msg);

    for (size_t i = 0; i < current_state_msg.joint_state.name.size(); ++i)
    {
      query.append(prefix + ".joint_state.name_" + std::to_string(i), current_state_msg.joint_state.name.at(i));
      queryAppendCenterWithTolerance(query, prefix + ".joint_state.position_" + std::to_string(i),
                                     current_state_msg.joint_state.position.at(i), match_tolerance);
    }
  }
  else
  {
    for (size_t i = 0; i < robot_state.joint_state.name.size(); ++i)
    {
      query.append(prefix + ".joint_state.name_" + std::to_string(i), robot_state.joint_state.name.at(i));
      queryAppendCenterWithTolerance(query, prefix + ".joint_state.position_" + std::to_string(i),
                                     robot_state.joint_state.position.at(i), match_tolerance);
    }
  }

  return moveit::core::MoveItErrorCode::SUCCESS;
}

moveit::core::MoveItErrorCode
appendRobotStateJointStateAsInsertMetadata(Metadata& metadata, const moveit_msgs::msg::RobotState& robot_state,
                                           const MoveGroupInterface& move_group, const std::string& prefix)
{
  // Make ignored members explicit

  if (!robot_state.multi_dof_joint_state.joint_names.empty())
  {
    RCLCPP_WARN_STREAM(getLogger(), "Ignoring " << prefix << ".multi_dof_joint_states: Not supported.");
  }
  if (!robot_state.attached_collision_objects.empty())
  {
    RCLCPP_WARN_STREAM(getLogger(), "Ignoring " << prefix << ".attached_collision_objects: Not supported.");
  }

  // Begin extraction.

  if (robot_state.is_diff)
  {
    // If plan request states that the start_state is_diff, then we need to get
    // the current state from the move_group.

    // NOTE: methyldragon -
    //   I think if is_diff is on, the joint states will not be populated in all
    //   of the motion plan requests? If this isn't the case we might need to
    //   apply the joint states as offsets as well.
    //
    // TODO: Since MoveIt also potentially does another getCurrentState() call
    //   when planning, there is a chance that the current state in the cache
    //   differs from the state used in MoveIt's plan.
    //
    //   This issue should go away once the class is used within the move group's
    //   Plan call.
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    if (!current_state)
    {
      // NOTE: methyldragon -
      //   Ideally we would restore the original state here and undo our changes, however copy of the metadata is not
      //   supported.
      std::stringstream ss;
      ss << "Skipping " << prefix << " metadata append: "
         << "Could not get robot state.";
      return MoveItErrorCode(MoveItErrorCode::UNABLE_TO_AQUIRE_SENSOR_DATA, ss.str());
    }

    moveit_msgs::msg::RobotState current_state_msg;
    robotStateToRobotStateMsg(*current_state, current_state_msg);

    for (size_t i = 0; i < current_state_msg.joint_state.name.size(); ++i)
    {
      metadata.append(prefix + ".joint_state.name_" + std::to_string(i), current_state_msg.joint_state.name.at(i));
      metadata.append(prefix + ".joint_state.position_" + std::to_string(i),
                      current_state_msg.joint_state.position.at(i));
    }
  }
  else
  {
    for (size_t i = 0; i < robot_state.joint_state.name.size(); ++i)
    {
      metadata.append(prefix + ".joint_state.name_" + std::to_string(i), robot_state.joint_state.name.at(i));
      metadata.append(prefix + ".joint_state.position_" + std::to_string(i), robot_state.joint_state.position.at(i));
    }
  }

  return moveit::core::MoveItErrorCode::SUCCESS;
}

}  // namespace trajectory_cache
}  // namespace moveit_ros
