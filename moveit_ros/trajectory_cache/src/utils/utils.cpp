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

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>

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
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer = move_group.getTF();

  // Make ignored members explicit.

  bool emit_position_constraint_warning = false;
  for (auto& constraint : constraints)
  {
    for (auto& position_constraint : constraint.position_constraints)
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
  for (auto& constraint : constraints)
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
  for (auto& constraint : constraints)
  {
    // We sort to avoid cardinality.
    sortJointConstraints(constraint.joint_constraints);
    sortPositionConstraints(constraint.position_constraints);
    sortOrientationConstraints(constraint.orientation_constraints);

    std::string constraint_prefix = prefix + "_" + std::to_string(constraint_idx++);

    // Joint constraints
    size_t joint_idx = 0;
    for (auto& joint_constraint : constraint.joint_constraints)
    {
      std::string meta_name = constraint_prefix + ".joint_constraints_" + std::to_string(joint_idx++);
      query.append(meta_name + ".joint_name", joint_constraint.joint_name);
      queryAppendCenterWithTolerance(query, meta_name + ".position", joint_constraint.position, match_tolerance);
      query.appendGTE(meta_name + ".tolerance_above", joint_constraint.tolerance_above);
      query.appendLTE(meta_name + ".tolerance_below", joint_constraint.tolerance_below);
    }

    // Position constraints
    if (!constraint.position_constraints.empty())
    {
      // All offsets will be "frozen" and computed wrt. the workspace frame instead.
      query.append(constraint_prefix + ".position_constraints.header.frame_id", reference_frame_id);

      size_t position_idx = 0;
      for (auto& position_constraint : constraint.position_constraints)
      {
        std::string meta_name = constraint_prefix + ".position_constraints_" + std::to_string(position_idx++);

        // Compute offsets wrt. to workspace frame.
        double x_offset = 0;
        double y_offset = 0;
        double z_offset = 0;

        if (reference_frame_id != position_constraint.header.frame_id)
        {
          try
          {
            auto transform =
                tf_buffer->lookupTransform(position_constraint.header.frame_id, reference_frame_id, tf2::TimePointZero);

            x_offset = transform.transform.translation.x;
            y_offset = transform.transform.translation.y;
            z_offset = transform.transform.translation.z;
          }
          catch (tf2::TransformException& ex)
          {
            // NOTE: methyldragon -
            //   Ideally we would restore the original state here and undo our changes, however copy of the query is not
            //   supported.
            std::stringstream ss;
            ss << "Skipping " << prefix << " metadata append: "
               << "Could not get transform for translation " << reference_frame_id << " to "
               << position_constraint.header.frame_id << ": " << ex.what();
            return MoveItErrorCode(MoveItErrorCode::FRAME_TRANSFORM_FAILURE, ss.str());
          }
        }

        query.append(meta_name + ".link_name", position_constraint.link_name);

        queryAppendCenterWithTolerance(query, meta_name + ".target_point_offset.x",
                                       x_offset + position_constraint.target_point_offset.x, match_tolerance);
        queryAppendCenterWithTolerance(query, meta_name + ".target_point_offset.y",
                                       y_offset + position_constraint.target_point_offset.y, match_tolerance);
        queryAppendCenterWithTolerance(query, meta_name + ".target_point_offset.z",
                                       z_offset + position_constraint.target_point_offset.z, match_tolerance);
      }
    }

    // Orientation constraints
    if (!constraint.orientation_constraints.empty())
    {
      // All offsets will be "frozen" and computed wrt. the workspace frame instead.
      query.append(constraint_prefix + ".orientation_constraints.header.frame_id", reference_frame_id);

      size_t ori_idx = 0;
      for (auto& orientation_constraint : constraint.orientation_constraints)
      {
        std::string meta_name = constraint_prefix + ".orientation_constraints_" + std::to_string(ori_idx++);

        // Compute offsets wrt. to workspace frame.
        geometry_msgs::msg::Quaternion quat_offset;
        quat_offset.x = 0;
        quat_offset.y = 0;
        quat_offset.z = 0;
        quat_offset.w = 1;

        if (reference_frame_id != orientation_constraint.header.frame_id)
        {
          try
          {
            auto transform = tf_buffer->lookupTransform(orientation_constraint.header.frame_id, reference_frame_id,
                                                        tf2::TimePointZero);

            quat_offset = transform.transform.rotation;
          }
          catch (tf2::TransformException& ex)
          {
            // NOTE: methyldragon -
            //   Ideally we would restore the original state here and undo our changes, however copy of the query is not
            //   supported.
            std::stringstream ss;
            ss << "Skipping " << prefix << " metadata append: "
               << "Could not get transform for orientation " << reference_frame_id << " to "
               << orientation_constraint.header.frame_id << ": " << ex.what();
            return MoveItErrorCode(MoveItErrorCode::FRAME_TRANSFORM_FAILURE, ss.str());
          }
        }

        query.append(meta_name + ".link_name", orientation_constraint.link_name);

        // Orientation of constraint frame wrt. workspace frame
        tf2::Quaternion tf2_quat_frame_offset(quat_offset.x, quat_offset.y, quat_offset.z, quat_offset.w);

        // Added offset on top of the constraint frame's orientation stated in the constraint.
        tf2::Quaternion tf2_quat_goal_offset(orientation_constraint.orientation.x, orientation_constraint.orientation.y,
                                             orientation_constraint.orientation.z,
                                             orientation_constraint.orientation.w);

        tf2_quat_frame_offset.normalize();
        tf2_quat_goal_offset.normalize();

        auto final_quat = tf2_quat_goal_offset * tf2_quat_frame_offset;
        final_quat.normalize();

        queryAppendCenterWithTolerance(query, meta_name + ".target_point_offset.x", final_quat.getX(), match_tolerance);
        queryAppendCenterWithTolerance(query, meta_name + ".target_point_offset.y", final_quat.getY(), match_tolerance);
        queryAppendCenterWithTolerance(query, meta_name + ".target_point_offset.z", final_quat.getZ(), match_tolerance);
        queryAppendCenterWithTolerance(query, meta_name + ".target_point_offset.w", final_quat.getW(), match_tolerance);
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
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer = move_group.getTF();

  // Make ignored members explicit

  bool emit_position_constraint_warning = false;
  for (auto& constraint : constraints)
  {
    for (auto& position_constraint : constraint.position_constraints)
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
  for (auto& constraint : constraints)
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
  for (auto& constraint : constraints)
  {
    // We sort to avoid cardinality.
    sortJointConstraints(constraint.joint_constraints);
    sortPositionConstraints(constraint.position_constraints);
    sortOrientationConstraints(constraint.orientation_constraints);

    std::string constraint_prefix = prefix + "_" + std::to_string(constraint_idx++);

    // Joint constraints
    size_t joint_idx = 0;
    for (auto& joint_constraint : constraint.joint_constraints)
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
      for (auto& position_constraint : constraint.position_constraints)
      {
        std::string meta_name = constraint_prefix + ".position_constraints_" + std::to_string(position_idx++);

        // Compute offsets wrt. to workspace frame.
        double x_offset = 0;
        double y_offset = 0;
        double z_offset = 0;

        if (workspace_frame_id != position_constraint.header.frame_id)
        {
          try
          {
            auto transform =
                tf_buffer->lookupTransform(position_constraint.header.frame_id, workspace_frame_id, tf2::TimePointZero);

            x_offset = transform.transform.translation.x;
            y_offset = transform.transform.translation.y;
            z_offset = transform.transform.translation.z;
          }
          catch (tf2::TransformException& ex)
          {
            // NOTE: methyldragon -
            //   Ideally we would restore the original state here and undo our changes, however copy of the query is not
            //   supported.
            std::stringstream ss;
            ss << "Skipping " << prefix << " metadata append: "
               << "Could not get transform for translation " << workspace_frame_id << " to "
               << position_constraint.header.frame_id << ": " << ex.what();
            return MoveItErrorCode(MoveItErrorCode::FRAME_TRANSFORM_FAILURE, ss.str());
          }
        }

        metadata.append(meta_name + ".link_name", position_constraint.link_name);
        metadata.append(meta_name + ".target_point_offset.x", x_offset + position_constraint.target_point_offset.x);
        metadata.append(meta_name + ".target_point_offset.y", y_offset + position_constraint.target_point_offset.y);
        metadata.append(meta_name + ".target_point_offset.z", z_offset + position_constraint.target_point_offset.z);
      }
    }

    // Orientation constraints
    if (!constraint.orientation_constraints.empty())
    {
      // All offsets will be "frozen" and computed wrt. the workspace frame instead.
      metadata.append(constraint_prefix + ".orientation_constraints.header.frame_id", workspace_frame_id);

      size_t ori_idx = 0;
      for (auto& orientation_constraint : constraint.orientation_constraints)
      {
        std::string meta_name = constraint_prefix + ".orientation_constraints_" + std::to_string(ori_idx++);

        // Compute offsets wrt. to workspace frame.
        geometry_msgs::msg::Quaternion quat_offset;
        quat_offset.x = 0;
        quat_offset.y = 0;
        quat_offset.z = 0;
        quat_offset.w = 1;

        if (workspace_frame_id != orientation_constraint.header.frame_id)
        {
          try
          {
            auto transform = tf_buffer->lookupTransform(orientation_constraint.header.frame_id, workspace_frame_id,
                                                        tf2::TimePointZero);

            quat_offset = transform.transform.rotation;
          }
          catch (tf2::TransformException& ex)
          {
            // NOTE: methyldragon -
            //   Ideally we would restore the original state here and undo our changes, however copy of the query is not
            //   supported.
            std::stringstream ss;
            ss << "Skipping " << prefix << " metadata append: "
               << "Could not get transform for orientation " << workspace_frame_id << " to "
               << orientation_constraint.header.frame_id << ": " << ex.what();
            return MoveItErrorCode(MoveItErrorCode::FRAME_TRANSFORM_FAILURE, ss.str());
          }
        }

        metadata.append(meta_name + ".link_name", orientation_constraint.link_name);

        // Orientation of constraint frame wrt. workspace frame
        tf2::Quaternion tf2_quat_frame_offset(quat_offset.x, quat_offset.y, quat_offset.z, quat_offset.w);

        // Added offset on top of the constraint frame's orientation stated in the constraint.
        tf2::Quaternion tf2_quat_goal_offset(orientation_constraint.orientation.x, orientation_constraint.orientation.y,
                                             orientation_constraint.orientation.z,
                                             orientation_constraint.orientation.w);

        tf2_quat_frame_offset.normalize();
        tf2_quat_goal_offset.normalize();

        auto final_quat = tf2_quat_goal_offset * tf2_quat_frame_offset;
        final_quat.normalize();

        metadata.append(meta_name + ".target_point_offset.x", final_quat.getX());
        metadata.append(meta_name + ".target_point_offset.y", final_quat.getY());
        metadata.append(meta_name + ".target_point_offset.z", final_quat.getZ());
        metadata.append(meta_name + ".target_point_offset.w", final_quat.getW());
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

    for (size_t i = 0; i < current_state_msg.joint_state.name.size(); i++)
    {
      query.append(prefix + ".joint_state.name_" + std::to_string(i), current_state_msg.joint_state.name.at(i));
      queryAppendCenterWithTolerance(query, prefix + ".joint_state.position_" + std::to_string(i),
                                     current_state_msg.joint_state.position.at(i), match_tolerance);
    }
  }
  else
  {
    for (size_t i = 0; i < robot_state.joint_state.name.size(); i++)
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

    for (size_t i = 0; i < current_state_msg.joint_state.name.size(); i++)
    {
      metadata.append(prefix + ".joint_state.name_" + std::to_string(i), current_state_msg.joint_state.name.at(i));
      metadata.append(prefix + ".joint_state.position_" + std::to_string(i),
                      current_state_msg.joint_state.position.at(i));
    }
  }
  else
  {
    for (size_t i = 0; i < robot_state.joint_state.name.size(); i++)
    {
      metadata.append(prefix + ".joint_state.name_" + std::to_string(i), robot_state.joint_state.name.at(i));
      metadata.append(prefix + ".joint_state.position_" + std::to_string(i), robot_state.joint_state.position.at(i));
    }
  }

  return moveit::core::MoveItErrorCode::SUCCESS;
}

}  // namespace trajectory_cache
}  // namespace moveit_ros
