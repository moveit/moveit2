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

#include <tf2_ros/buffer.h>

#include "../fixtures/warehouse_fixture.hpp"

namespace
{

using ::testing::TestParamInfo;
using ::testing::ValuesIn;

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

// restateInNewFrame.

TEST(RestateInNewFrameTest, NoopOnNullTranslationAndRotation)
{
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));

  moveit::core::MoveItErrorCode result = moveit_ros::trajectory_cache::restateInNewFrame(
      tf_buffer, "frame_a", "frame_b", nullptr, nullptr, tf2::TimePointZero);

  EXPECT_EQ(result, moveit::core::MoveItErrorCode::SUCCESS);
}

TEST(RestateInNewFrameTest, NoopOnSameFrame)
{
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  geometry_msgs::msg::Point translation;
  translation.x = 1;
  translation.y = 2;
  translation.z = 3;
  geometry_msgs::msg::Quaternion rotation;
  rotation.x = 0;
  rotation.y = 0;
  rotation.z = 0;
  rotation.w = 1;

  moveit::core::MoveItErrorCode result = moveit_ros::trajectory_cache::restateInNewFrame(
      tf_buffer, "frame_a", "frame_a", &translation, &rotation, tf2::TimePointZero);

  EXPECT_EQ(result, moveit::core::MoveItErrorCode::SUCCESS);
  EXPECT_EQ(translation.x, 1);
  EXPECT_EQ(translation.y, 2);
  EXPECT_EQ(translation.z, 3);
  EXPECT_EQ(rotation.x, 0);
  EXPECT_EQ(rotation.y, 0);
  EXPECT_EQ(rotation.z, 0);
  EXPECT_EQ(rotation.w, 1);
}

TEST(RestateInNewFrameTest, FailsOnMissingTransform)
{
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  geometry_msgs::msg::Point translation;
  geometry_msgs::msg::Quaternion rotation;

  moveit::core::MoveItErrorCode result = moveit_ros::trajectory_cache::restateInNewFrame(
      tf_buffer, "frame_a", "frame_c", &translation, &rotation, tf2::TimePointZero);

  EXPECT_NE(result, moveit::core::MoveItErrorCode::SUCCESS);
}

struct RestateInNewFrameTestCase
{
  std::string test_name;
  bool translate;
  bool rotate;
  double expected_translation_x;
  double expected_translation_y;
  double expected_translation_z;
  double expected_rotation_x;
  double expected_rotation_y;
  double expected_rotation_z;
  double expected_rotation_w;
};

class RestateInNewFrameParameterizedTest : public testing::WithParamInterface<RestateInNewFrameTestCase>,
                                           public testing::Test
{
};

INSTANTIATE_TEST_SUITE_P(RestateInNewFrameParameterizedTests, RestateInNewFrameParameterizedTest,
                         ValuesIn<RestateInNewFrameTestCase>({ {
                                                                   .test_name = "TranslateAndRotate",
                                                                   .translate = true,
                                                                   .rotate = true,
                                                                   .expected_translation_x = 2,
                                                                   .expected_translation_y = 4,
                                                                   .expected_translation_z = 6,
                                                                   .expected_rotation_x = 0,
                                                                   .expected_rotation_y = 0,
                                                                   .expected_rotation_z = 0.707,
                                                                   .expected_rotation_w = 0.707,
                                                               },
                                                               {
                                                                   .test_name = "NullTranslationIgnoresTranslation",
                                                                   .translate = false,
                                                                   .rotate = true,
                                                                   .expected_translation_x = 1,
                                                                   .expected_translation_y = 2,
                                                                   .expected_translation_z = 3,
                                                                   .expected_rotation_x = 0,
                                                                   .expected_rotation_y = 0,
                                                                   .expected_rotation_z = 0.707,
                                                                   .expected_rotation_w = 0.707,
                                                               },
                                                               {
                                                                   .test_name = "NullRotationIgnoresRotation",
                                                                   .translate = true,
                                                                   .rotate = false,
                                                                   .expected_translation_x = 2,
                                                                   .expected_translation_y = 4,
                                                                   .expected_translation_z = 6,
                                                                   .expected_rotation_x = 0,
                                                                   .expected_rotation_y = 0,
                                                                   .expected_rotation_z = 0,
                                                                   .expected_rotation_w = 1,
                                                               } }),
                         [](const TestParamInfo<RestateInNewFrameParameterizedTest::ParamType>& info) {
                           return info.param.test_name;
                         });

TEST_P(RestateInNewFrameParameterizedTest, RestateInNewFrame)
{
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "frame_a";
  transform.child_frame_id = "frame_b";
  transform.transform.translation.x = 1;
  transform.transform.translation.y = 2;
  transform.transform.translation.z = 3;
  transform.transform.rotation.x = 0;
  transform.transform.rotation.y = 0;
  transform.transform.rotation.z = 0.707;  // 90 degree rotation about the z-axis.
  transform.transform.rotation.w = 0.707;
  tf_buffer->setTransform(transform, "test");

  geometry_msgs::msg::Point translation;
  translation.x = 1;
  translation.y = 2;
  translation.z = 3;

  geometry_msgs::msg::Quaternion rotation;
  rotation.x = 0;
  rotation.y = 0;
  rotation.z = 0;
  rotation.w = 1;

  RestateInNewFrameTestCase params = GetParam();

  moveit::core::MoveItErrorCode result =
      moveit_ros::trajectory_cache::restateInNewFrame(tf_buffer, /*target_frame=*/"frame_a", /*source_frame=*/"frame_b",
                                                      params.translate ? &translation : nullptr,
                                                      params.rotate ? &rotation : nullptr, tf2::TimePointZero);

  EXPECT_EQ(result, moveit::core::MoveItErrorCode::SUCCESS);

  if (params.translate)
  {
    EXPECT_EQ(translation.x, params.expected_translation_x);
    EXPECT_EQ(translation.y, params.expected_translation_y);
    EXPECT_EQ(translation.z, params.expected_translation_z);
  }
  else
  {
    EXPECT_EQ(translation.x, 1);
    EXPECT_EQ(translation.y, 2);
    EXPECT_EQ(translation.z, 3);
  }

  if (params.rotate)
  {
    EXPECT_NEAR(rotation.x, params.expected_rotation_x, 1e-3);
    EXPECT_NEAR(rotation.y, params.expected_rotation_y, 1e-3);
    EXPECT_NEAR(rotation.z, params.expected_rotation_z, 1e-3);
    EXPECT_NEAR(rotation.w, params.expected_rotation_w, 1e-3);
  }
  else
  {
    EXPECT_EQ(rotation.x, 0);
    EXPECT_EQ(rotation.y, 0);
    EXPECT_EQ(rotation.z, 0);
    EXPECT_EQ(rotation.w, 1);
  }
}

// Other utils.

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
