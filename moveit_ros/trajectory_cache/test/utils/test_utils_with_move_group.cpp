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
 * @author methylDragon
 */

#include <gtest/gtest.h>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/trajectory_cache/utils/utils.hpp>

#include "../fixtures/move_group_fixture.hpp"

namespace
{

TEST_F(MoveGroupFixture, GetWorkspaceFrameIdWorks)
{
  moveit_msgs::msg::WorkspaceParameters workspace_parameters;
  EXPECT_EQ(moveit_ros::trajectory_cache::getWorkspaceFrameId(*move_group_, workspace_parameters),
            move_group_->getRobotModel()->getModelFrame());

  workspace_parameters.header.frame_id = "test_frame";
  EXPECT_EQ(moveit_ros::trajectory_cache::getWorkspaceFrameId(*move_group_, workspace_parameters), "test_frame");
}

TEST_F(MoveGroupFixture, GetCartesianPathRequestFrameId)
{
  moveit_msgs::srv::GetCartesianPath::Request path_request;
  EXPECT_EQ(moveit_ros::trajectory_cache::getCartesianPathRequestFrameId(*move_group_, path_request),
            move_group_->getPoseReferenceFrame());

  path_request.header.frame_id = "test_frame";
  EXPECT_EQ(moveit_ros::trajectory_cache::getCartesianPathRequestFrameId(*move_group_, path_request), "test_frame");
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
