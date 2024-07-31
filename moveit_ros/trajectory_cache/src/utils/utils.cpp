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

#include <string>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/trajectory_cache/utils/utils.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>

#include <warehouse_ros/message_collection.h>

// Frames. =========================================================================================

std::string getWorkspaceFrameId(const moveit::planning_interface::MoveGroupInterface& move_group,
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

std::string getCartesianPathRequestFrameId(const moveit::planning_interface::MoveGroupInterface& move_group,
                                           const moveit_msgs::srv::GetCartesianPath::Request& path_request)
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

// Features. =======================================================================================

void queryAppendCenterWithTolerance(warehouse_ros::Query& query, const std::string& name, double center,
                                    double tolerance)
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
