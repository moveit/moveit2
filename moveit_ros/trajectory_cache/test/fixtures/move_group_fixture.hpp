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

#pragma once

#include <memory>

#include <gtest/gtest.h>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/warehouse/moveit_message_storage.hpp>
#include <rclcpp/rclcpp.hpp>

/** @class MoveGroupFixture
 * @brief Test fixture to spin up a node to start a move group with. */
class MoveGroupFixture : public ::testing::Test
{
public:
  MoveGroupFixture();
  ~MoveGroupFixture() override;

protected:
  void SetUp() override;
  void TearDown() override;

  rclcpp::Node::SharedPtr node_;
  warehouse_ros::DatabaseConnection::Ptr db_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::string move_group_name_;

private:
  void spinNode();

  std::thread spin_thread_;
  std::atomic<bool> is_spinning_;
};
