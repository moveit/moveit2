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

#include <thread>

#include <gtest/gtest.h>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/warehouse/moveit_message_storage.hpp>
#include <rclcpp/rclcpp.hpp>

#include "move_group_fixture.hpp"

MoveGroupFixture::MoveGroupFixture()
  : node_(rclcpp::Node::make_shared("move_group_fixture")), move_group_name_("panda_arm")
{
  node_->declare_parameter<std::string>("warehouse_plugin", "warehouse_ros_sqlite::DatabaseConnection");
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, move_group_name_);
}

MoveGroupFixture::~MoveGroupFixture()
{
  is_spinning_ = false;
  if (spin_thread_.joinable())
  {
    spin_thread_.join();
  }
}

void MoveGroupFixture::SetUp()
{
  is_spinning_ = true;
  spin_thread_ = std::thread(&MoveGroupFixture::spinNode, this);

  db_ = moveit_warehouse::loadDatabase(node_);
  db_->setParams(":memory:", 1);
  ASSERT_TRUE(db_->connect());
}

void MoveGroupFixture::TearDown()
{
  db_.reset();
  is_spinning_ = false;
  spin_thread_.join();
}

void MoveGroupFixture::spinNode()
{
  while (is_spinning_ && rclcpp::ok())
  {
    rclcpp::spin_some(node_);
  }
}
