/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

/*      Title     : servo_cpp_interface_demo.cpp
 *      Project   : moveit_servo
 *      Created   : 07/13/2020
 *      Author    : Adam Pettinger
 */

// ROS
#include <rclcpp/rclcpp.hpp>

// Servo
#include <moveit_servo/servo_parameters.cpp>
#include <moveit_servo/servo.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.servo_demo_node.cpp");

  // ROS objects
  rclcpp::NodeOptions node_options;
  auto node = std::make_shared<rclcpp::Node>("servo_demo_node", node_options);
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  // Get Servo Parameters
  auto servo_parameters = std::make_shared<moveit_servo::ServoParameters>();
  if(!moveit_servo::readParameters(*node, LOGGER, *servo_parameters))
  {
    RCLCPP_ERROR(LOGGER, "Could not get parameters");
    return -1;
  }

  // Create the planning_scene_monitor
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description", tf_buffer, "planning_scene_monitor");

  // Get the planning_scene_monitor to publish scene diff's for RViz visualization
  if (planning_scene_monitor->getPlanningScene())
  {
    planning_scene_monitor->startStateMonitor("/joint_states");
    planning_scene_monitor->setPlanningScenePublishingFrequency(25);
    planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE, "/moveit_servo/publish_planning_scene");
    planning_scene_monitor->startSceneMonitor();
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Planning scene not configured");
    return -1;
  }

  // Pause for RViz to come up..
  rclcpp::sleep_for(std::chrono::seconds(4));

  // Create collision object, in the way of servoing
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "panda_link0";
  collision_object.id = "box";

  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = { 0.1, 0.4, 0.1 };

  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = 0.6;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.6;

  collision_object.primitives.push_back(box);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  moveit_msgs::msg::PlanningSceneWorld psw;
  psw.collision_objects.push_back(collision_object);

  moveit_msgs::msg::PlanningScene ps;
  ps.world = psw;

  // Publish the collision object to the planning scene
  auto scene_pub = node->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);
  scene_pub->publish(ps);

  // Create Servo and start it
  moveit_servo::Servo servo(node, servo_parameters, planning_scene_monitor);
  while (!servo.waitForInitialized())
  {
    auto& clock = *node->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(LOGGER, clock, 5000,
                                "Waiting for ServoCalcs to recieve joint states");
  }
  servo.start();

  // Create publisher for publishing a few joint commands
  size_t count{0};
  auto joint_pub = node->create_publisher<control_msgs::msg::JointJog>("servo_server/delta_joint_cmds", 10);
  std::weak_ptr<std::remove_pointer<decltype(joint_pub.get())>::type> captured_joint_pub = joint_pub;
  std::weak_ptr<std::remove_pointer<decltype(node.get())>::type> captured_node = node;
  auto joint_callback = [captured_joint_pub, captured_node, &count]() -> void {
      auto pub_ptr = captured_joint_pub.lock();
      auto node_ptr = captured_node.lock();
      if (!pub_ptr || !node_ptr) {
        return;
      }
      if(count > 100)
        return;
      auto msg = std::make_unique<control_msgs::msg::JointJog>();
      msg->header.stamp = node_ptr->now();
      msg->header.frame_id = "panda_link3";
      msg->joint_names.push_back("panda_joint1");
      msg->velocities.push_back(0.3);
      pub_ptr->publish(std::move(msg));
      ++count;
    };
  auto joint_timer = node->create_wall_timer(50ms, joint_callback);

  // Create a publisher for publishing the twist commands
  auto twist_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("servo_server/delta_twist_cmds", 10);
  std::weak_ptr<std::remove_pointer<decltype(twist_pub.get())>::type> captured_twist_pub = twist_pub;
  auto twist_callback = [captured_twist_pub, captured_node, &count]() -> void {
      auto pub_ptr = captured_twist_pub.lock();
      auto node_ptr = captured_node.lock();
      if (!pub_ptr || !node_ptr) {
        return;
      }
      if (count < 100)
        return;
      auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
      msg->header.stamp = node_ptr->now();
      msg->header.frame_id = "panda_link0";
      msg->twist.linear.x = 0.3;
      msg->twist.angular.z = 0.5;
      pub_ptr->publish(std::move(msg));
    };
  auto twist_timer = node->create_wall_timer(50ms, twist_callback);

  executor->add_node(node);
  
  executor->spin();
  rclcpp::shutdown();
  return 0;
}