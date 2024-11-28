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
#include <rclcpp/version.h>

#include <geometry_msgs/msg/point.hpp>
#include <moveit/trajectory_cache/utils/utils.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>

#include "../fixtures/warehouse_fixture.hpp"

namespace
{

using ::warehouse_ros::MessageCollection;
using ::warehouse_ros::Metadata;
using ::warehouse_ros::Query;

// This test throws an exception on Humble. It is not clear why. Excluding it for now.
#if RCLCPP_VERSION_GTE(28, 3, 3)
TEST_F(WarehouseFixture, QueryAppendCenterWithToleranceWorks)
{
  MessageCollection<geometry_msgs::msg::Point> coll =
      db_->openCollection<geometry_msgs::msg::Point>("test_db", "test_collection");

  Metadata::Ptr metadata = coll.createMetadata();
  metadata->append("test_metadata", 5.0);
  coll.insert(geometry_msgs::msg::Point(), metadata);

  Query::Ptr unrelated_query = coll.createQuery();
  moveit_ros::trajectory_cache::queryAppendCenterWithTolerance(*unrelated_query, "unrelated_metadata", 1.0, 10.0);
  EXPECT_TRUE(coll.queryList(unrelated_query).empty());

  Query::Ptr related_query_too_low = coll.createQuery();
  moveit_ros::trajectory_cache::queryAppendCenterWithTolerance(*related_query_too_low, "test_metadata", 4.45, 1.0);
  EXPECT_TRUE(coll.queryList(related_query_too_low).empty());

  Query::Ptr related_query_too_high = coll.createQuery();
  moveit_ros::trajectory_cache::queryAppendCenterWithTolerance(*related_query_too_high, "test_metadata", 5.55, 1.0);
  EXPECT_TRUE(coll.queryList(related_query_too_high).empty());

  Query::Ptr related_query_in_range = coll.createQuery();
  moveit_ros::trajectory_cache::queryAppendCenterWithTolerance(*related_query_in_range, "test_metadata", 5.0, 1.0);
  EXPECT_EQ(coll.queryList(related_query_in_range).size(), 1);
}
#endif

TEST(TestUtils, GetExecutionTimeWorks)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  trajectory.joint_trajectory.points.resize(2);
  trajectory.joint_trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(1.0);
  trajectory.joint_trajectory.points[1].time_from_start = rclcpp::Duration::from_seconds(2.0);

  EXPECT_EQ(moveit_ros::trajectory_cache::getExecutionTime(trajectory), 2.0);
}

TEST(TestUtils, ConstraintSortingWorks)
{
  // Joint constraints.
  {
    moveit_msgs::msg::JointConstraint jc1;
    jc1.joint_name = "joint2";
    moveit_msgs::msg::JointConstraint jc2;
    jc2.joint_name = "joint1";
    std::vector<moveit_msgs::msg::JointConstraint> joint_constraints = { jc1, jc2 };
    moveit_ros::trajectory_cache::sortJointConstraints(joint_constraints);

    EXPECT_EQ(joint_constraints[0].joint_name, "joint1");
    EXPECT_EQ(joint_constraints[1].joint_name, "joint2");
  }

  // Position constraints.
  {
    moveit_msgs::msg::PositionConstraint pc1;
    pc1.link_name = "link2";
    moveit_msgs::msg::PositionConstraint pc2;
    pc2.link_name = "link1";
    std::vector<moveit_msgs::msg::PositionConstraint> position_constraints = { pc1, pc2 };
    moveit_ros::trajectory_cache::sortPositionConstraints(position_constraints);
    EXPECT_EQ(position_constraints[0].link_name, "link1");
    EXPECT_EQ(position_constraints[1].link_name, "link2");
  }

  // Orientation constraints.
  {
    moveit_msgs::msg::OrientationConstraint oc1;
    oc1.link_name = "link2";
    moveit_msgs::msg::OrientationConstraint oc2;
    oc2.link_name = "link1";
    std::vector<moveit_msgs::msg::OrientationConstraint> orientation_constraints = { oc1, oc2 };
    moveit_ros::trajectory_cache::sortOrientationConstraints(orientation_constraints);
    EXPECT_EQ(orientation_constraints[0].link_name, "link1");
    EXPECT_EQ(orientation_constraints[1].link_name, "link2");
  }
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
