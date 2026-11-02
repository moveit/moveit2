/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Universitaet Hamburg
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Michael 'v4hn' Goerner */

#include <gtest/gtest.h>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/utils/message_checks.hpp>
#include <moveit/utils/robot_model_test_utils.hpp>
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <sstream>
#include <string>

#include <tf2_eigen/tf2_eigen.hpp>

namespace
{
void makeSphere(moveit_msgs::msg::CollisionObject& co)
{
  // set valid primitive
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  primitive.dimensions.push_back(/* SPHERE_RADIUS */ 1.0);
  co.primitives.push_back(primitive);
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  co.primitive_poses.push_back(pose);
}
}  // namespace

TEST(PlanningScene, fillInObjectPoseFromPrimitive)
{
  moveit::core::RobotModelPtr robot_model(moveit::core::RobotModelBuilder("empty_robot", "base_link").build());
  planning_scene::PlanningScene scene(robot_model);

  moveit_msgs::msg::CollisionObject co;
  co.header.frame_id = robot_model->getModelFrame();
  co.id = "object_no_pose";
  co.operation = moveit_msgs::msg::CollisionObject::ADD;
  makeSphere(co);
  scene.processCollisionObjectMsg(co);

  Eigen::Isometry3d primitive_pose;
  tf2::fromMsg(co.primitive_poses.at(0), primitive_pose);
  ASSERT_TRUE(scene.knowsFrameTransform(co.id)) << "failed to add object";
  EXPECT_TRUE(scene.getFrameTransform(co.id).isApprox(primitive_pose))
      << "scene did not use only primitive pose as object pose";
}

TEST(PlanningScene, fillInPrimitivePose)
{
  moveit::core::RobotModelPtr robot_model(moveit::core::RobotModelBuilder("empty_robot", "base_link").build());
  planning_scene::PlanningScene scene(robot_model);

  moveit_msgs::msg::CollisionObject co;
  co.header.frame_id = robot_model->getModelFrame();
  co.id = "object_no_primitive_pose";
  co.operation = moveit_msgs::msg::CollisionObject::ADD;
  makeSphere(co);
  co.pose = co.primitive_poses.at(0);
  co.primitive_poses.resize(0);
  scene.processCollisionObjectMsg(co);

  Eigen::Isometry3d object_pose;
  tf2::fromMsg(co.pose, object_pose);
  ASSERT_TRUE(scene.knowsFrameTransform(co.id)) << "failed to add object";
  EXPECT_TRUE(scene.getFrameTransform(co.id).isApprox(object_pose))
      << "scene did not implicitly fill in identity pose for only primitive";
}

TEST(PlanningScene, rememberMetadataWhenAttached)
{
  moveit::core::RobotModelPtr robot_model(moveit::core::RobotModelBuilder("empty_robot", "base_link").build());
  planning_scene::PlanningScene scene(robot_model);

  // prepare planning scene message to add a colored object
  moveit_msgs::msg::PlanningScene scene_msg;
  scene_msg.robot_model_name = robot_model->getName();
  scene_msg.is_diff = true;
  scene_msg.robot_state.is_diff = true;

  moveit_msgs::msg::CollisionObject co;
  co.header.frame_id = robot_model->getModelFrame();
  co.id = "blue_sphere";
  co.operation = moveit_msgs::msg::CollisionObject::ADD;
  co.pose.orientation.w = 1.0;
  makeSphere(co);

  // meta-data 1: object type
  co.type.key = "blue_sphere_type";
  co.type.db = "{'type':'CustomDB'}";
  scene_msg.world.collision_objects.push_back(co);

  // meta-data 2: object color
  moveit_msgs::msg::ObjectColor color;
  color.id = co.id;
  color.color.b = 1.0;
  color.color.a = 1.0;
  scene_msg.object_colors.push_back(color);

  EXPECT_FALSE(scene.hasObjectColor(co.id)) << "scene knows color before adding it(?)";
  EXPECT_FALSE(scene.hasObjectType(co.id)) << "scene knows type before adding it(?)";

  // add object to scene
  scene.usePlanningSceneMsg(scene_msg);

  EXPECT_TRUE(scene.hasObjectColor(co.id)) << "scene failed to add object color";
  EXPECT_EQ(scene.getObjectColor(co.id), color.color) << "scene added wrong object color";
  EXPECT_TRUE(scene.hasObjectType(co.id)) << "scene failed to add object type";
  EXPECT_EQ(scene.getObjectType(co.id), co.type) << "scene added wrong object type";

  // attach object
  moveit_msgs::msg::AttachedCollisionObject aco;
  aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  aco.object.id = co.id;
  aco.link_name = robot_model->getModelFrame();
  scene.processAttachedCollisionObjectMsg(aco);

  EXPECT_EQ(scene.getObjectColor(co.id), color.color) << "scene forgot object color after it got attached";
  EXPECT_EQ(scene.getObjectType(co.id), co.type) << "scene forgot object type after it got attached";

  // trying to remove object from the scene while it is attached is expected to fail
  co.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  EXPECT_FALSE(scene.processCollisionObjectMsg(co))
      << "scene removed attached object from collision world (although it's not there)";

  // detach again right away
  aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  scene.processAttachedCollisionObjectMsg(aco);

  EXPECT_EQ(scene.getObjectColor(co.id), color.color) << "scene forgot specified color after attach/detach";
  EXPECT_EQ(scene.getObjectType(co.id), co.type) << "scene forgot specified type after attach/detach";
}

TEST(PlanningScene, rejectMismatchedSubframesInCollisionObjectAdd)
{
  moveit::core::RobotModelPtr robot_model(moveit::core::RobotModelBuilder("empty_robot", "base_link").build());
  planning_scene::PlanningScene scene(robot_model);

  moveit_msgs::msg::CollisionObject co;
  co.header.frame_id = robot_model->getModelFrame();
  co.id = "object_subframe_mismatch";
  co.operation = moveit_msgs::msg::CollisionObject::ADD;
  makeSphere(co);

  // subframe_poses has 1 element, subframe_names is empty
  geometry_msgs::msg::Pose subframe_pose;
  subframe_pose.orientation.w = 1.0;
  co.subframe_poses.push_back(subframe_pose);
  EXPECT_FALSE(scene.processCollisionObjectMsg(co));
  EXPECT_FALSE(scene.getWorld()->hasObject(co.id));

  // Reverse mismatch: subframe_names has 1 element, subframe_poses is empty
  co.subframe_poses.clear();
  co.subframe_names.push_back("tip");
  EXPECT_FALSE(scene.processCollisionObjectMsg(co));
  EXPECT_FALSE(scene.getWorld()->hasObject(co.id));

  // Matching subframes succeeds
  co.subframe_poses.push_back(subframe_pose);
  EXPECT_TRUE(scene.processCollisionObjectMsg(co));
  EXPECT_TRUE(scene.getWorld()->hasObject(co.id));
  EXPECT_TRUE(scene.knowsFrameTransform(co.id + "/tip"));
}

TEST(PlanningScene, rejectMismatchedSubframesInAttachedCollisionObject)
{
  moveit::core::RobotModelPtr robot_model(moveit::core::RobotModelBuilder("empty_robot", "base_link").build());
  planning_scene::PlanningScene scene(robot_model);

  moveit_msgs::msg::AttachedCollisionObject aco;
  aco.link_name = robot_model->getModelFrame();
  aco.object.header.frame_id = robot_model->getModelFrame();
  aco.object.id = "attached_subframe_mismatch";
  aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  makeSphere(aco.object);

  // subframe_poses has 1 element, subframe_names is empty
  geometry_msgs::msg::Pose subframe_pose;
  subframe_pose.orientation.w = 1.0;
  aco.object.subframe_poses.push_back(subframe_pose);
  EXPECT_FALSE(scene.processAttachedCollisionObjectMsg(aco));
  EXPECT_FALSE(scene.getCurrentState().hasAttachedBody(aco.object.id));

  // Reverse mismatch: subframe_names has 1 element, subframe_poses is empty
  aco.object.subframe_poses.clear();
  aco.object.subframe_names.push_back("tool_tip");
  EXPECT_FALSE(scene.processAttachedCollisionObjectMsg(aco));
  EXPECT_FALSE(scene.getCurrentState().hasAttachedBody(aco.object.id));

  // Matching subframes succeeds
  aco.object.subframe_poses.push_back(subframe_pose);
  EXPECT_TRUE(scene.processAttachedCollisionObjectMsg(aco));
  EXPECT_TRUE(scene.getCurrentState().hasAttachedBody(aco.object.id));
  EXPECT_TRUE(scene.knowsFrameTransform(aco.object.id + "/tool_tip"));

  // Test attachment of an existing world object with mismatched subframe arrays
  moveit_msgs::msg::CollisionObject co;
  co.header.frame_id = robot_model->getModelFrame();
  co.id = "world_object_for_attach";
  co.operation = moveit_msgs::msg::CollisionObject::ADD;
  makeSphere(co);
  ASSERT_TRUE(scene.processCollisionObjectMsg(co));
  ASSERT_TRUE(scene.getWorld()->hasObject(co.id));

  moveit_msgs::msg::AttachedCollisionObject aco_existing;
  aco_existing.link_name = robot_model->getModelFrame();
  aco_existing.object.id = co.id;
  aco_existing.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  // No geometry specified (attaching existing world object), but subframes mismatch
  aco_existing.object.subframe_poses.push_back(subframe_pose);
  EXPECT_FALSE(scene.processAttachedCollisionObjectMsg(aco_existing));
  EXPECT_FALSE(scene.getCurrentState().hasAttachedBody(co.id));
  EXPECT_TRUE(scene.getWorld()->hasObject(co.id)) << "World object should not be removed on rejected attachment";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
