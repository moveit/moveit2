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

/* Author: Adam Pettinger
   Desc:   TODO(adamp)
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
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Planning scene not configured");
    return -1;
  }

  // Pause for RViz to come up..
  rclcpp::Rate loop_rate(0.25);
  loop_rate.sleep();

  // Create Servo and start it
  moveit_servo::Servo servo(node, servo_parameters, planning_scene_monitor);
  servo.start();

  // Create a publisher for publishing the jog commands
  auto pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("servo_server/delta_twist_cmds", 10);
  std::weak_ptr<std::remove_pointer<decltype(pub.get())>::type> captured_pub = pub;
  std::weak_ptr<std::remove_pointer<decltype(node.get())>::type> captured_node = node;
  auto callback = [captured_pub, captured_node]() -> void {
      auto pub_ptr = captured_pub.lock();
      auto node_ptr = captured_node.lock();
      if (!pub_ptr || !node_ptr) {
        return;
      }
      auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
      msg->header.stamp = node_ptr->now();
      msg->header.frame_id = "panda_link0";
      msg->twist.linear.x = 0.1;
      msg->twist.angular.z = 0.5;
      pub_ptr->publish(std::move(msg));
    };
  auto timer = node->create_wall_timer(50ms, callback);

  executor->add_node(node);
  
  executor->spin();
  rclcpp::shutdown();
  return 0;
}