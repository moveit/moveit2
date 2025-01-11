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
#include <rclcpp/rclcpp.hpp>
#include <moveit/warehouse/moveit_message_storage.hpp>

#include "warehouse_fixture.hpp"

WarehouseFixture::WarehouseFixture() : node_(rclcpp::Node::make_shared("warehouse_fixture"))
{
  node_->declare_parameter<std::string>("warehouse_plugin", "warehouse_ros_sqlite::DatabaseConnection");
}

WarehouseFixture::~WarehouseFixture()
{
  is_spinning_ = false;
  if (spin_thread_.joinable())
  {
    spin_thread_.join();
  }
}

void WarehouseFixture::SetUp()
{
  is_spinning_ = true;
  spin_thread_ = std::thread(&WarehouseFixture::spinNode, this);

  db_ = moveit_warehouse::loadDatabase(node_);
  db_->setParams(":memory:", 1);
  ASSERT_TRUE(db_->connect());
}

void WarehouseFixture::TearDown()
{
  db_.reset();
  is_spinning_ = false;
  spin_thread_.join();
}

void WarehouseFixture::spinNode()
{
  while (is_spinning_ && rclcpp::ok())
  {
    rclcpp::spin_some(node_);
  }
}
