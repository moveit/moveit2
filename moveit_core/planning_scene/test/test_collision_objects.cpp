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

TEST(PlanningScene, failedShapeConversionRejectsCollisionObject)
{
  moveit::core::RobotModelPtr robot_model(moveit::core::RobotModelBuilder("empty_robot", "base_link").build());
  planning_scene::PlanningScene scene(robot_model);

  moveit_msgs::msg::CollisionObject co;
  co.header.frame_id = robot_model->getModelFrame();
  co.id = "invalid_primitive_object";
  co.operation = moveit_msgs::msg::CollisionObject::ADD;

  // Add 1 valid sphere primitive
  makeSphere(co);

  // Add 1 malformed primitive (unknown type and empty dimensions)
  shape_msgs::msg::SolidPrimitive malformed_primitive;
  malformed_primitive.type = 0;
  co.primitives.push_back(malformed_primitive);

  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  co.primitive_poses.push_back(pose);

  // processCollisionObjectMsg must fail
  EXPECT_FALSE(scene.processCollisionObjectMsg(co));

  // The collision world must not contain the partially accepted object
  EXPECT_FALSE(scene.getWorld()->hasObject(co.id));
}

TEST(PlanningScene, failedShapeConversionPreservesExistingCollisionObject)
{
  moveit::core::RobotModelPtr robot_model(moveit::core::RobotModelBuilder("empty_robot", "base_link").build());
  planning_scene::PlanningScene scene(robot_model);

  moveit_msgs::msg::CollisionObject co;
  co.header.frame_id = robot_model->getModelFrame();
  co.id = "existing_object";
  co.operation = moveit_msgs::msg::CollisionObject::ADD;
  makeSphere(co);

  EXPECT_TRUE(scene.processCollisionObjectMsg(co));
  ASSERT_TRUE(scene.getWorld()->hasObject(co.id));
  EXPECT_EQ(scene.getWorld()->getObject(co.id)->shapes_.size(), 1u);

  // Try replacing with an ADD message that has a malformed shape
  moveit_msgs::msg::CollisionObject bad_co;
  bad_co.header.frame_id = robot_model->getModelFrame();
  bad_co.id = "existing_object";
  bad_co.operation = moveit_msgs::msg::CollisionObject::ADD;
  shape_msgs::msg::SolidPrimitive malformed_primitive;
  malformed_primitive.type = 0;
  bad_co.primitives.push_back(malformed_primitive);
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  bad_co.primitive_poses.push_back(pose);

  EXPECT_FALSE(scene.processCollisionObjectMsg(bad_co));

  // Verify the existing object was NOT removed or corrupted
  ASSERT_TRUE(scene.getWorld()->hasObject("existing_object"));
  EXPECT_EQ(scene.getWorld()->getObject("existing_object")->shapes_.size(), 1u);
}

TEST(PlanningScene, shapesAndPosesFromCollisionObjectMessageFailsOnInvalidGeometry)
{
  moveit::core::RobotModelPtr robot_model(moveit::core::RobotModelBuilder("empty_robot", "base_link").build());
  planning_scene::PlanningScene scene(robot_model);

  // Test invalid primitive
  {
    moveit_msgs::msg::CollisionObject co;
    co.header.frame_id = robot_model->getModelFrame();
    co.id = "bad_prim";
    shape_msgs::msg::SolidPrimitive malformed_primitive;
    malformed_primitive.type = 255;  // Invalid primitive type
    co.primitives.push_back(malformed_primitive);

    Eigen::Isometry3d object_pose;
    std::vector<shapes::ShapeConstPtr> shapes;
    EigenSTL::vector_Isometry3d shape_poses;

    EXPECT_FALSE(scene.shapesAndPosesFromCollisionObjectMessage(co, object_pose, shapes, shape_poses));
    EXPECT_TRUE(shapes.empty());
    EXPECT_TRUE(shape_poses.empty());
  }

  // Test invalid mesh
  {
    moveit_msgs::msg::CollisionObject co;
    co.header.frame_id = robot_model->getModelFrame();
    co.id = "bad_mesh";
    shape_msgs::msg::Mesh empty_mesh;
    co.meshes.push_back(empty_mesh);

    Eigen::Isometry3d object_pose;
    std::vector<shapes::ShapeConstPtr> shapes;
    EigenSTL::vector_Isometry3d shape_poses;

    EXPECT_FALSE(scene.shapesAndPosesFromCollisionObjectMessage(co, object_pose, shapes, shape_poses));
    EXPECT_TRUE(shapes.empty());
    EXPECT_TRUE(shape_poses.empty());
  }
}

TEST(PlanningScene, failedShapeConversionRejectsAttachedCollisionObject)
{
  moveit::core::RobotModelPtr robot_model(moveit::core::RobotModelBuilder("empty_robot", "base_link").build());
  planning_scene::PlanningScene scene(robot_model);

  moveit_msgs::msg::AttachedCollisionObject aco;
  aco.link_name = robot_model->getModelFrame();
  aco.object.header.frame_id = robot_model->getModelFrame();
  aco.object.id = "bad_attached_object";
  aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;

  makeSphere(aco.object);

  shape_msgs::msg::SolidPrimitive malformed_primitive;
  malformed_primitive.type = 0;
  aco.object.primitives.push_back(malformed_primitive);

  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  aco.object.primitive_poses.push_back(pose);

  EXPECT_FALSE(scene.processAttachedCollisionObjectMsg(aco));
  EXPECT_FALSE(scene.getCurrentState().hasAttachedBody(aco.object.id));
}

TEST(PlanningScene, replaceObjectWithHeaderFrameReferencingExistingObjectSubframe)
{
  moveit::core::RobotModelPtr robot_model(moveit::core::RobotModelBuilder("empty_robot", "base_link").build());
  planning_scene::PlanningScene scene(robot_model);

  moveit_msgs::msg::CollisionObject co;
  co.header.frame_id = robot_model->getModelFrame();
  co.id = "self_referencing_obj";
  co.operation = moveit_msgs::msg::CollisionObject::ADD;
  makeSphere(co);

  geometry_msgs::msg::Pose subframe_pose;
  subframe_pose.position.x = 1.0;
  subframe_pose.orientation.w = 1.0;
  co.subframe_poses.push_back(subframe_pose);
  co.subframe_names.push_back("subframe_1");

  ASSERT_TRUE(scene.processCollisionObjectMsg(co));
  ASSERT_TRUE(scene.knowsFrameTransform("self_referencing_obj/subframe_1"));

  // Replace using an ADD message whose header.frame_id is the existing object's subframe
  moveit_msgs::msg::CollisionObject replace_co;
  replace_co.header.frame_id = "self_referencing_obj/subframe_1";
  replace_co.id = "self_referencing_obj";
  replace_co.operation = moveit_msgs::msg::CollisionObject::ADD;
  makeSphere(replace_co);

  EXPECT_TRUE(scene.processCollisionObjectMsg(replace_co));
  EXPECT_TRUE(scene.getWorld()->hasObject(replace_co.id));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
