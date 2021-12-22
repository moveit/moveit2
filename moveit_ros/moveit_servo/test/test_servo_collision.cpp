/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
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
 *   * Neither the name of PickNik Inc. nor the names of its
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

/*      Title     : test_servo_collision.cpp
 *      Project   : moveit_servo
 *      Created   : 08/03/2020
 *      Author    : Adam Pettinger
 */

#include "servo_launch_test_common.hpp"
#include <moveit_msgs/msg/planning_scene.hpp>

namespace moveit_servo
{
TEST_F(ServoFixture, SelfCollision)
{
  auto log_time_start = node_->now();
  ASSERT_TRUE(setupStartClient());
  auto log_time_end = node_->now();
  RCLCPP_INFO_STREAM(LOGGER, "Setup time: " << (log_time_end - log_time_start).seconds());

  // Start Servo
  ASSERT_TRUE(start());
  EXPECT_EQ(latest_status_, moveit_servo::StatusCode::NO_WARNING);

  // Look for DECELERATE_FOR_COLLISION status
  watchForStatus(moveit_servo::StatusCode::DECELERATE_FOR_COLLISION);

  // Publish some joint jog commands that will bring us to collision
  rclcpp::Rate publish_loop_rate(test_parameters_->publish_hz);
  log_time_start = node_->now();
  size_t iterations = 0;
  while (!sawTrackedStatus() && iterations++ < test_parameters_->timeout_iterations)
  {
    auto msg = std::make_unique<control_msgs::msg::JointJog>();
    msg->header.stamp = node_->now();
    msg->header.frame_id = "panda_link3";
    msg->joint_names.push_back("panda_joint4");
    msg->velocities.push_back(-1.0);
    pub_joint_cmd_->publish(std::move(msg));
    publish_loop_rate.sleep();
  }

  EXPECT_LT(iterations, test_parameters_->timeout_iterations);
  log_time_end = node_->now();
  RCLCPP_INFO_STREAM(LOGGER, "Wait for collision: " << (log_time_end - log_time_start).seconds());
}

TEST_F(ServoFixture, ExternalCollision)
{
  // NOTE: This test is meant to be run after the SelfCollision test
  // It assumes the position is where the robot ends in SelfCollision test

  auto log_time_start = node_->now();
  ASSERT_TRUE(setupStartClient());
  auto log_time_end = node_->now();
  RCLCPP_INFO_STREAM(LOGGER, "Setup time: " << (log_time_end - log_time_start).seconds());

  // Start Servo
  ASSERT_TRUE(start());

  // Create collision object, in the way of servoing
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "panda_link0";
  collision_object.id = "box";

  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = { 0.1, 0.4, 0.1 };

  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = 0.4;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.4;

  collision_object.primitives.push_back(box);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  moveit_msgs::msg::PlanningSceneWorld psw;
  psw.collision_objects.push_back(collision_object);

  moveit_msgs::msg::PlanningScene ps;
  ps.is_diff = true;
  ps.world = psw;

  // Publish the collision object to the planning scene
  auto scene_pub = node_->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);
  scene_pub->publish(ps);

  // Look for DECELERATE_FOR_COLLISION status
  watchForStatus(moveit_servo::StatusCode::DECELERATE_FOR_COLLISION);

  // Now publish twist commands that collide with the box
  rclcpp::Rate publish_loop_rate(test_parameters_->publish_hz);
  log_time_start = node_->now();
  size_t iterations = 0;
  while (!sawTrackedStatus() && iterations++ < test_parameters_->timeout_iterations)
  {
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = node_->now();
    msg->twist.linear.x = 0.2;
    pub_twist_cmd_->publish(std::move(msg));
    publish_loop_rate.sleep();
  }

  EXPECT_LT(iterations, test_parameters_->timeout_iterations);
  log_time_end = node_->now();
  RCLCPP_INFO_STREAM(LOGGER, "Wait for collision: " << (log_time_end - log_time_start).seconds());
}

}  // namespace moveit_servo

int main(int argc, char** argv)
{
  // It is important we init ros before google test because we are going to
  // create a node during the google test init.
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);

  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
