/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Nanjing University
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
 *   * Neither the name of Nanjing University nor the names of its
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

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_interface/planning_request_adapter.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/utils/robot_model_test_utils.hpp>
#include <pluginlib/class_loader.hpp>

class TestCheckStartStateCollision : public testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_check_start_state_collision_adapter");

    moveit::core::RobotModelBuilder builder("slider_robot", "base");
    builder.addChain("base->slider", "prismatic");
    geometry_msgs::msg::Pose link_collision_pose;
    link_collision_pose.orientation.w = 1.0;
    builder.addCollisionBox("slider", { 0.1, 0.1, 0.1 }, link_collision_pose);
    builder.addGroup({}, { "base-slider-joint" }, "slider_group");
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(builder.build());

    auto& current_state = planning_scene_->getCurrentStateNonConst();
    current_state.setVariablePosition("base-slider-joint", 0.8);
    current_state.update();

    moveit_msgs::msg::CollisionObject obstacle;
    obstacle.header.frame_id = "base";
    obstacle.id = "start_box";
    obstacle.operation = moveit_msgs::msg::CollisionObject::ADD;
    obstacle.pose.orientation.w = 1.0;
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = { 0.2, 0.2, 0.2 };
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    obstacle.primitives = { box };
    obstacle.primitive_poses = { box_pose };
    ASSERT_TRUE(planning_scene_->processCollisionObjectMsg(obstacle));

    plugin_loader_ = std::make_unique<pluginlib::ClassLoader<planning_interface::PlanningRequestAdapter>>(
        "moveit_core", "planning_interface::PlanningRequestAdapter");
    adapter_ = plugin_loader_->createUniqueInstance("default_planning_request_adapters/CheckStartStateCollision");
    adapter_->initialize(node_, "");
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<planning_scene::PlanningScene> planning_scene_;
  std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlanningRequestAdapter>> plugin_loader_;
  pluginlib::UniquePtr<planning_interface::PlanningRequestAdapter> adapter_;
};

TEST_F(TestCheckStartStateCollision, ReportsContactsFromRequestStartState)
{
  ASSERT_FALSE(planning_scene_->isStateColliding(planning_scene_->getCurrentState(), "slider_group"));

  planning_interface::MotionPlanRequest request;
  request.group_name = "slider_group";
  request.start_state.is_diff = true;
  request.start_state.joint_state.name = { "base-slider-joint" };
  request.start_state.joint_state.position = { 0.0 };

  moveit::core::RobotState request_state = planning_scene_->getCurrentState();
  moveit::core::robotStateMsgToRobotState(planning_scene_->getTransforms(), request.start_state, request_state);
  ASSERT_TRUE(planning_scene_->isStateColliding(request_state, request.group_name));

  const auto result = adapter_->adapt(planning_scene_, request);
  EXPECT_EQ(result.val, moveit_msgs::msg::MoveItErrorCodes::START_STATE_IN_COLLISION);
  EXPECT_EQ(result.source, "CheckStartStateCollision");
  EXPECT_EQ(result.message.find("0 contact(s)"), std::string::npos);
  EXPECT_NE(result.message.find("start_box"), std::string::npos);
  EXPECT_NE(result.message.find("slider"), std::string::npos);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
