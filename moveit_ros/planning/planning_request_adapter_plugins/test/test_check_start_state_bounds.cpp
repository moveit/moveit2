/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Sebastian Castro
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
 *   * Neither the name of PickNik nor the names of its
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

/* Author: Sebastian Castro */

#include <memory>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_pipeline/planning_pipeline.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/utils/robot_model_test_utils.hpp>

class TestCheckStartStateBounds : public testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_check_start_state_bounds_adapter", "");

    // Load a robot model and place it in a planning scene.
    urdf::ModelInterfaceSharedPtr urdf_model = moveit::core::loadModelInterface("pr2");
    srdf::ModelSharedPtr srdf_model = std::make_shared<srdf::Model>();
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(urdf_model, srdf_model);

    // Load the planning request adapter.
    plugin_loader_ = std::make_unique<pluginlib::ClassLoader<planning_interface::PlanningRequestAdapter>>(
        "moveit_core", "planning_interface::PlanningRequestAdapter");
    adapter_ = plugin_loader_->createUniqueInstance("default_planning_request_adapters/CheckStartStateBounds");
    adapter_->initialize(node_, "");
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<planning_scene::PlanningScene> planning_scene_;
  std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlanningRequestAdapter>> plugin_loader_;
  pluginlib::UniquePtr<planning_interface::PlanningRequestAdapter> adapter_;
};

TEST_F(TestCheckStartStateBounds, TestWithinBounds)
{
  planning_interface::MotionPlanRequest request;
  request.group_name = "right_arm";
  request.start_state.joint_state.name = {
    "r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_forearm_roll_joint",
    "r_elbow_flex_joint",   "r_wrist_flex_joint",    "r_wrist_roll_joint",
  };
  request.start_state.joint_state.position = {
    0.0, 0.0, 0.0, 0.0, -0.5, -0.5, 0.0,
  };

  const auto result = adapter_->adapt(planning_scene_, request);
  EXPECT_EQ(result.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
  EXPECT_EQ(result.message, "");
}

TEST_F(TestCheckStartStateBounds, TestRevoluteJointOutOfBounds)
{
  planning_interface::MotionPlanRequest request;
  request.group_name = "right_arm";
  request.start_state.joint_state.name = {
    "r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_forearm_roll_joint",
    "r_elbow_flex_joint",   "r_wrist_flex_joint",    "r_wrist_roll_joint",
  };
  request.start_state.joint_state.position = {
    1.0,  // revolute joint out of bounds
    0.0, 0.0, 0.0, -0.5, -0.5, 0.0,
  };

  const auto result = adapter_->adapt(planning_scene_, request);
  EXPECT_EQ(result.val, moveit_msgs::msg::MoveItErrorCodes::START_STATE_INVALID);
  EXPECT_EQ(result.message, "Start state out of bounds.");
}

TEST_F(TestCheckStartStateBounds, TestContinuousJointOutOfBounds)
{
  planning_interface::MotionPlanRequest request;
  request.group_name = "right_arm";
  request.start_state.joint_state.name = {
    "r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_forearm_roll_joint",
    "r_elbow_flex_joint",   "r_wrist_flex_joint",    "r_wrist_roll_joint",
  };
  request.start_state.joint_state.position = {
    0.0,  0.0,  0.0, 100.0,  // continuous joint out of bounds
    -0.5, -0.5, 0.0,
  };

  const auto result = adapter_->adapt(planning_scene_, request);
  EXPECT_EQ(result.val, moveit_msgs::msg::MoveItErrorCodes::START_STATE_INVALID);
  EXPECT_EQ(result.message, "Start state out of bounds.");
}

TEST_F(TestCheckStartStateBounds, TestContinuousJointFixedBounds)
{
  planning_interface::MotionPlanRequest request;
  request.group_name = "right_arm";
  request.start_state.joint_state.name = {
    "r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_forearm_roll_joint",
    "r_elbow_flex_joint",   "r_wrist_flex_joint",    "r_wrist_roll_joint",
  };
  request.start_state.joint_state.position = {
    0.0,  0.0,  0.0, 100.0,  // continuous joint out of bounds
    -0.5, -0.5, 0.0,
  };

  // Modify the start state. The adapter should succeed.
  node_->set_parameter(rclcpp::Parameter("fix_start_state", true));

  const auto result = adapter_->adapt(planning_scene_, request);
  EXPECT_EQ(result.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
  EXPECT_EQ(result.message, "Normalized start state.");

  // Validate that the large continuous joint position value in the request start state was normalized.
  const auto& joint_names = request.start_state.joint_state.name;
  const size_t joint_idx =
      std::find(joint_names.begin(), joint_names.end(), "r_forearm_roll_joint") - joint_names.begin();
  EXPECT_NEAR(request.start_state.joint_state.position[joint_idx], -0.530965, 1.0e-4);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
