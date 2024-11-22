/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, University of Hamburg
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
 *   * Neither the name of the copyright holder nor the names of its
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

/* Author: Michael 'v4hn' Goerner
   Desc: Tests for PlanningSceneMonitor
*/

// ROS
#include <rclcpp/rclcpp.hpp>

// Testing
#include <gtest/gtest.h>

// Main class
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>

class PlanningSceneMonitorTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    test_node_ = std::make_shared<rclcpp::Node>("moveit_planning_scene_monitor_test");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    planning_scene_monitor_ = std::make_unique<planning_scene_monitor::PlanningSceneMonitor>(
        test_node_, "robot_description", "planning_scene_monitor");
    planning_scene_monitor_->monitorDiffs(true);
    scene_ = planning_scene_monitor_->getPlanningScene();
    executor_->add_node(test_node_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });

    // Needed to avoid race conditions on high-load CPUs.
    std::this_thread::sleep_for(std::chrono::seconds{ 1 });
  }

  void TearDown() override
  {
    scene_.reset();
    executor_->cancel();
    if (executor_thread_.joinable())
    {
      executor_thread_.join();
    }
  }

protected:
  std::shared_ptr<rclcpp::Node> test_node_;

  // Executor and a thread to run the executor.
  rclcpp::Executor::SharedPtr executor_;
  std::thread executor_thread_;

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  planning_scene::PlanningScenePtr scene_;
};

// various code expects the monitored scene to remain the same
TEST_F(PlanningSceneMonitorTest, TestPersistentScene)
{
  auto scene{ planning_scene_monitor_->getPlanningScene() };
  moveit_msgs::msg::PlanningScene msg;
  msg.is_diff = msg.robot_state.is_diff = true;
  planning_scene_monitor_->newPlanningSceneMessage(msg);
  EXPECT_EQ(scene, planning_scene_monitor_->getPlanningScene());
  msg.is_diff = msg.robot_state.is_diff = false;
  planning_scene_monitor_->newPlanningSceneMessage(msg);
  EXPECT_EQ(scene, planning_scene_monitor_->getPlanningScene());
}

using UpdateType = planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType;

#define TRIGGERS_UPDATE(msg, expected_update_type)                                                                     \
  {                                                                                                                    \
    planning_scene_monitor_->clearUpdateCallbacks();                                                                   \
    auto received_update_type{ UpdateType::UPDATE_NONE };                                                              \
    planning_scene_monitor_->addUpdateCallback([&](auto type) { received_update_type = type; });                       \
    planning_scene_monitor_->newPlanningSceneMessage(msg);                                                             \
    EXPECT_EQ(received_update_type, expected_update_type);                                                             \
  }

TEST_F(PlanningSceneMonitorTest, UpdateTypes)
{
  moveit_msgs::msg::PlanningScene msg;
  msg.is_diff = msg.robot_state.is_diff = true;
  TRIGGERS_UPDATE(msg, UpdateType::UPDATE_NONE);

  msg.fixed_frame_transforms.emplace_back();
  msg.fixed_frame_transforms.back().header.frame_id = "base_link";
  msg.fixed_frame_transforms.back().child_frame_id = "object";
  msg.fixed_frame_transforms.back().transform.rotation.w = 1.0;
  TRIGGERS_UPDATE(msg, UpdateType::UPDATE_TRANSFORMS);
  msg.fixed_frame_transforms.clear();
  moveit::core::robotStateToRobotStateMsg(scene_->getCurrentState(), msg.robot_state, false);
  msg.robot_state.is_diff = true;
  TRIGGERS_UPDATE(msg, UpdateType::UPDATE_STATE);

  msg.robot_state.is_diff = false;
  TRIGGERS_UPDATE(msg, UpdateType::UPDATE_STATE | UpdateType::UPDATE_GEOMETRY);

  msg.robot_state = moveit_msgs::msg::RobotState{};
  msg.robot_state.is_diff = true;
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "base_link";
  collision_object.id = "object";
  collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
  collision_object.pose.orientation.w = 1.0;
  collision_object.primitives.emplace_back();
  collision_object.primitives.back().type = shape_msgs::msg::SolidPrimitive::SPHERE;
  collision_object.primitives.back().dimensions = { 1.0 };
  msg.world.collision_objects.emplace_back(collision_object);
  TRIGGERS_UPDATE(msg, UpdateType::UPDATE_GEOMETRY);

  msg.world.collision_objects.clear();
  msg.is_diff = false;

  TRIGGERS_UPDATE(msg, UpdateType::UPDATE_SCENE);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
