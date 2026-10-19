/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Jorge Santos Simón
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

#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <moveit/move_group/capability_names.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Stands in for move_group, serving a fixed set of collision objects
class MockPlanningSceneServices final : public rclcpp::Node
{
public:
  MockPlanningSceneServices() : Node("mock_planning_scene_services")
  {
    get_planning_scene_service_ = create_service<moveit_msgs::srv::GetPlanningScene>(
        move_group::GET_PLANNING_SCENE_SERVICE_NAME,
        [this](const std::shared_ptr<moveit_msgs::srv::GetPlanningScene::Request>& /*request*/,
               const std::shared_ptr<moveit_msgs::srv::GetPlanningScene::Response>& response) {
          response->scene.world.collision_objects = collision_objects;
        });
    apply_planning_scene_service_ = create_service<moveit_msgs::srv::ApplyPlanningScene>(
        move_group::APPLY_PLANNING_SCENE_SERVICE_NAME,
        [](const std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene::Request>& /*request*/,
           const std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene::Response>& response) {
          response->success = true;
        });
  }

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

private:
  rclcpp::Service<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_planning_scene_service_;
  rclcpp::Service<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_planning_scene_service_;
};

geometry_msgs::msg::Pose makePose(double x, double y, double z, double yaw = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, yaw);
  pose.orientation = tf2::toMsg(orientation);
  return pose;
}

class PlanningSceneInterfaceTest : public testing::Test
{
protected:
  void SetUp() override
  {
    mock_services_ = std::make_shared<MockPlanningSceneServices>();
    executor_.add_node(mock_services_);
    ros_thread_ = std::thread([this]() { executor_.spin(); });
    planning_scene_interface_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
  }

  void TearDown() override
  {
    planning_scene_interface_.reset();
    executor_.cancel();
    ros_thread_.join();
  }

  std::shared_ptr<MockPlanningSceneServices> mock_services_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread ros_thread_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
};

TEST_F(PlanningSceneInterfaceTest, KnownObjectNamesInROIWithPrimitive)
{
  // GIVEN a box at (1, 0, 0), with its primitive at the object's origin
  moveit_msgs::msg::CollisionObject box;
  box.header.frame_id = "world";
  box.id = "box";
  box.pose = makePose(1.0, 0.0, 0.0);
  box.primitives.resize(1);
  box.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  box.primitives[0].dimensions = { 0.1, 0.1, 0.1 };
  box.primitive_poses = { makePose(0.0, 0.0, 0.0) };
  mock_services_->collision_objects = { box };

  // THEN it is found in a ROI around (1, 0, 0), but not around the origin
  EXPECT_EQ(planning_scene_interface_->getKnownObjectNamesInROI(0.9, -0.1, -0.1, 1.1, 0.1, 0.1),
            std::vector<std::string>({ "box" }));
  EXPECT_TRUE(planning_scene_interface_->getKnownObjectNamesInROI(-0.1, -0.1, -0.1, 0.1, 0.1, 0.1).empty());
}

TEST_F(PlanningSceneInterfaceTest, KnownObjectNamesInROIWithMesh)
{
  // GIVEN an object at (1, 0, 0), rotated 90 degrees around z, with its mesh at (0.5, 0, 0) relative to it
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "world";
  object.id = "object";
  object.pose = makePose(1.0, 0.0, 0.0, M_PI_2);
  object.meshes.resize(1);
  object.mesh_poses = { makePose(0.5, 0.0, 0.0) };
  mock_services_->collision_objects = { object };

  // THEN it is found in a ROI around (1, 0.5, 0), but not around (1.5, 0, 0) nor (0.5, 0, 0)
  EXPECT_EQ(planning_scene_interface_->getKnownObjectNamesInROI(0.9, 0.4, -0.1, 1.1, 0.6, 0.1),
            std::vector<std::string>({ "object" }));
  EXPECT_TRUE(planning_scene_interface_->getKnownObjectNamesInROI(1.4, -0.1, -0.1, 1.6, 0.1, 0.1).empty());
  EXPECT_TRUE(planning_scene_interface_->getKnownObjectNamesInROI(0.4, -0.1, -0.1, 0.6, 0.1, 0.1).empty());
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
