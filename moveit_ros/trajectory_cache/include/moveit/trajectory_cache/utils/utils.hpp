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
 * @brief Utilities used by the trajectory_cache package.
 * @author methylDragon
 */

#pragma once

#include <string>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>

#include <tf2_ros/buffer.h>

#include <warehouse_ros/message_collection.h>

namespace moveit_ros
{
namespace trajectory_cache
{

// Frames. =========================================================================================

/** @brief Gets workspace frame ID.
 * If workspace_parameters has no frame ID, fetch it from move_group.
 *
 * It makes sense to use getPoseReferenceFrame() in the absence of a frame ID in the request because
 * the same method is used to populate the header frame ID in the MoveGroupInterface's
 * computeCartesianPath() method, which this function is associated with.
 */
std::string getWorkspaceFrameId(const moveit::planning_interface::MoveGroupInterface& move_group,
                                const moveit_msgs::msg::WorkspaceParameters& workspace_parameters);

/** @brief Gets cartesian path request frame ID.
 * If path_request has no frame ID, fetch it from move_group.
 *
 * It makes sense to use getPoseReferenceFrame() in the absence of a frame ID in the request because
 * the same method is used to populate the header frame ID in the MoveGroupInterface's
 * computeCartesianPath() method, which this function is associated with.
 */
std::string getCartesianPathRequestFrameId(const moveit::planning_interface::MoveGroupInterface& move_group,
                                           const moveit_msgs::srv::GetCartesianPath::Request& path_request);

/** @brief Restates a translation and rotation in a new frame.
 *
 * @param[in] tf. The transform buffer to use.
 * @param[in] target_frame. The frame to restate in.
 * @param[in] source_frame. The frame to restate from.
 * @param[in,out] translation. The translation to restate. Ignored if nullptr.
 * @param[in,out] rotation. The rotation to restate. Ignored if nullptr.
 * @returns MoveItErrorCode::SUCCESS if successfully restated. Otherwise, will return return
 * MoveItErrorCode::FRAME_TRANSFORM_FAILURE if the transform could not be retrieved.
 */
moveit::core::MoveItErrorCode restateInNewFrame(const std::shared_ptr<tf2_ros::Buffer>& tf,
                                                const std::string& target_frame, const std::string& source_frame,
                                                geometry_msgs::msg::Point* translation,
                                                geometry_msgs::msg::Quaternion* rotation,
                                                const tf2::TimePoint& lookup_time = tf2::TimePointZero);

// Execution Time. =================================================================================

/** @brief Returns the execution time of the trajectory in double seconds. */
double getExecutionTime(const moveit_msgs::msg::RobotTrajectory& trajectory);

// Request Construction. ===========================================================================

/**
 * @brief Constructs a GetCartesianPath request.
 *
 * This is a convenience function.
 * This mimics the move group computeCartesianPath signature (without path constraints).
 *
 * WARNING: The following fields are not supported, if you want to specify them, add them in yourself.
 *   - prismatic_jump_threshold
 *   - revolute_jump_threshold
 *   - cartesian_speed_limited_link
 *   - max_cartesian_speed
 *
 * @param[in] move_group. The manipulator move group, used to get its state, frames, and link.
 * @param[in] waypoints. The cartesian waypoints to request the path for.
 * @param[in] max_step. The value to populate into the `GetCartesianPath` request's max_step field.
 * @param[in] jump_threshold. The value to populate into the `GetCartesianPath` request's jump_threshold field.
 * @param[in] avoid_collisions. The value to populate into the `GetCartesianPath` request's avoid_collisions field.
 * @returns
 */
moveit_msgs::srv::GetCartesianPath::Request
constructGetCartesianPathRequest(moveit::planning_interface::MoveGroupInterface& move_group,
                                 const std::vector<geometry_msgs::msg::Pose>& waypoints, double max_step,
                                 double jump_threshold, bool avoid_collisions = true);

// Queries. ========================================================================================

/** @brief Appends a range inclusive query with some tolerance about some center value. */
void queryAppendCenterWithTolerance(warehouse_ros::Query& query, const std::string& name, double center,
                                    double tolerance);

// Constraints. ====================================================================================

/** @brief Sorts a vector of joint constraints in-place by joint name. */
void sortJointConstraints(std::vector<moveit_msgs::msg::JointConstraint>& joint_constraints);

/** @brief Sorts a vector of position constraints in-place by link name. */
void sortPositionConstraints(std::vector<moveit_msgs::msg::PositionConstraint>& position_constraints);

/** @brief Sorts a vector of orientation constraints in-place by link name. */
void sortOrientationConstraints(std::vector<moveit_msgs::msg::OrientationConstraint>& orientation_constraints);

/** @brief Extracts relevant features from a vector of moveit_msgs::msg::Constraints messages to a fetch query, with
 * tolerance.
 *
 * This will extract relevant features from the joint, position, and orientation constraints per element.
 * This exists because many keyable messages contain constraints which should be handled similarly.
 *
 * WARNING: Visibility constraints are not supported.
 *
 * Additionally, the component constraints within each vector element are sorted to reduce cache cardinality.
 * For the same reason, constraints with frames are restated in terms of the workspace frame.
 *
 * We copy the input constraints to support this.
 *
 * @param[in,out] query. The query to add features to.
 * @param[in] constraints. The constraints to extract features from.
 * @param[in] move_group. The manipulator move group, used to get its state.
 * @param[in] reference_frame_id. The frame to restate constraints in.
 * @param[in] prefix. A prefix to add to feature keys.
 * @returns moveit::core::MoveItErrorCode::SUCCESS if successfully appended. Otherwise, will return a different error
 * code, in which case the query should not be reused.
 */
moveit::core::MoveItErrorCode appendConstraintsAsFetchQueryWithTolerance(
    warehouse_ros::Query& query, std::vector<moveit_msgs::msg::Constraints> constraints,
    const moveit::planning_interface::MoveGroupInterface& move_group, double match_tolerance,
    const std::string& reference_frame_id, const std::string& prefix);

/** @brief Extracts relevant features from a vector of moveit_msgs::msg::Constraints messages to a cache entry's
 * metadata.
 *
 * This will extract relevant features from the joint, position, and orientation constraints per element.
 * This exists because many keyable messages contain constraints which should be handled similarly.
 *
 * WARNING: Visibility constraints and constraint regions are not supported.
 *
 * Additionally, the component constraints within each vector element are sorted to reduce cache cardinality.
 * For the same reason, constraints with frames are restated in terms of the workspace frame.
 *
 * We copy the input constraints to support this.
 *
 * @param[in,out] metadata. The metadata to add features to.
 * @param[in] constraints. The constraints to extract features from.
 * @param[in] move_group. The manipulator move group, used to get its state.
 * @param[in] reference_frame_id. The frame to restate constraints in.
 * @param[in] prefix. A prefix to add to feature keys.
 * @returns moveit::core::MoveItErrorCode::SUCCESS if successfully appended. Otherwise, will return a different error
 * code, in which case the metadata should not be reused.
 */
moveit::core::MoveItErrorCode
appendConstraintsAsInsertMetadata(warehouse_ros::Metadata& metadata,
                                  std::vector<moveit_msgs::msg::Constraints> constraints,
                                  const moveit::planning_interface::MoveGroupInterface& move_group,
                                  const std::string& reference_frame_id, const std::string& prefix);

// RobotState. =====================================================================================

/** @brief Extracts relevant features from a vector of moveit_msgs::msg::Constraints messages to a fetch query, with
 * tolerance.
 *
 * This will extract relevant features from the joint, position, and orientation constraints per element.
 * This exists because many keyable messages contain constraints which should be handled similarly.
 *
 * WARNING: Visibility constraints are not supported.
 *
 * Additionally, the component constraints within each vector element are sorted to reduce cache cardinality.
 * For the same reason, constraints with frames are restated in terms of the workspace frame.
 *
 * We copy the input constraints to support this.
 *
 * @param[in,out] query. The query to add features to.
 * @param[in] robot_state. The robot state to extract features from.
 * @param[in] move_group. The manipulator move group, used to get its state.
 * @param[in] prefix. A prefix to add to feature keys.
 * @returns moveit::core::MoveItErrorCode::SUCCESS if successfully appended. Otherwise, will return a different error
 * code, in which case the query should not be reused.
 */
moveit::core::MoveItErrorCode
appendRobotStateJointStateAsFetchQueryWithTolerance(warehouse_ros::Query& query,
                                                    const moveit_msgs::msg::RobotState& robot_state,
                                                    const moveit::planning_interface::MoveGroupInterface& move_group,
                                                    double match_tolerance, const std::string& prefix);

/** @brief Extracts relevant features from a vector of moveit_msgs::msg::Constraints messages to a cache entry's
 * metadata.
 *
 * This will extract relevant features from the joint, position, and orientation constraints per element.
 * This exists because many keyable messages contain constraints which should be handled similarly.
 *
 * WARNING: Visibility constraints and constraint regions are not supported.
 *
 * Additionally, the component constraints within each vector element are sorted to reduce cache cardinality.
 * For the same reason, constraints with frames are restated in terms of the workspace frame.
 *
 * We copy the input constraints to support this.
 *
 * @param[in,out] metadata. The metadata to add features to.
 * @param[in] robot_state. The robot state to extract features from.
 * @param[in] move_group. The manipulator move group, used to get its state.
 * @param[in] prefix. A prefix to add to feature keys.
 * @returns moveit::core::MoveItErrorCode::SUCCESS if successfully appended. Otherwise, will return a different error
 * code, in which case the metadata should not be reused.
 */
moveit::core::MoveItErrorCode appendRobotStateJointStateAsInsertMetadata(
    warehouse_ros::Metadata& metadata, const moveit_msgs::msg::RobotState& robot_state,
    const moveit::planning_interface::MoveGroupInterface& move_group, const std::string& prefix);

}  // namespace trajectory_cache
}  // namespace moveit_ros
