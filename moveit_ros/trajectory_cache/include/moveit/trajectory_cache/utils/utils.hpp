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

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/srv/get_cartesian_path.hpp>

#include <warehouse_ros/message_collection.h>

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

// Features. =======================================================================================

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
