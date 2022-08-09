/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos_event.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_ros.planning_scene_monitor.demo_scene");

static const std::string ROBOT_DESCRIPTION = "robot_description";

void sendKnife(const rclcpp::Node::SharedPtr& node)
{
  auto pub_aco = node->create_publisher<moveit_msgs::msg::AttachedCollisionObject>("attached_collision_object", 10);
  moveit_msgs::msg::AttachedCollisionObject aco;
  aco.link_name = "r_wrist_roll_link";
  aco.touch_links.push_back("r_wrist_roll_link");
  aco.touch_links.push_back("r_gripper_palm_link");
  aco.touch_links.push_back("r_gripper_led_frame");
  aco.touch_links.push_back("r_gripper_motor_accelerometer_link");
  aco.touch_links.push_back("r_gripper_tool_frame");
  aco.touch_links.push_back("r_gripper_motor_slider_link");
  aco.touch_links.push_back("r_gripper_motor_screw_link");
  aco.touch_links.push_back("r_gripper_l_finger_link");
  aco.touch_links.push_back("r_gripper_l_finger_tip_link");
  aco.touch_links.push_back("r_gripper_r_finger_link");
  aco.touch_links.push_back("r_gripper_r_finger_tip_link");
  aco.touch_links.push_back("r_gripper_l_finger_tip_frame");

  moveit_msgs::msg::CollisionObject& co = aco.object;
  co.id = "knife";
  co.header.stamp = rclcpp::Clock().now();
  co.header.frame_id = aco.link_name;
  co.pose.orientation.w = 1.0;
  co.operation = moveit_msgs::msg::CollisionObject::ADD;
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  co.primitives[0].dimensions.push_back(0.1);
  co.primitives[0].dimensions.push_back(0.1);
  co.primitives[0].dimensions.push_back(0.4);
  co.primitive_poses.resize(1);
  co.primitive_poses[0].position.x = 0.1;
  co.primitive_poses[0].position.y = 0;
  co.primitive_poses[0].position.z = -0.2;

  using namespace std::chrono_literals;
  pub_aco->publish(aco);
  rclcpp::sleep_for(1s);
  pub_aco->publish(aco);
  RCLCPP_INFO(LOGGER, "Object published.");
  rclcpp::sleep_for(1500ms);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("demo_scene");

  sendKnife(node);

  rclcpp::spin(node);

  return 0;
}
