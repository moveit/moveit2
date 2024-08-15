/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, PickNik Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*      Title     : demo_twist.cpp
 *      Project   : moveit_servo
 *      Created   : 06/01/2023
 *      Author    : V Mohammed Ibrahim
 *
 *      Description : Example of controlling a robot through twist commands via the C++ API.
 */

#include <chrono>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <moveit/utils/logger.hpp>

using namespace moveit_servo;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // The servo object expects to get a ROS node.
  const rclcpp::Node::SharedPtr demo_node = std::make_shared<rclcpp::Node>("moveit_servo_demo");
  moveit::setNodeLoggerName(demo_node->get_name());

  // Get the servo parameters.
  const std::string param_namespace = "moveit_servo";
  const std::shared_ptr<const servo::ParamListener> servo_param_listener =
      std::make_shared<const servo::ParamListener>(demo_node, param_namespace);
  const servo::Params servo_params = servo_param_listener->get_params();

  // The publisher to send trajectory message to the robot controller.
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_outgoing_cmd_pub =
      demo_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(servo_params.command_out_topic,
                                                                         rclcpp::SystemDefaultsQoS());

  // Create the servo object
  const planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor =
      createPlanningSceneMonitor(demo_node, servo_params);
  Servo servo = Servo(demo_node, servo_param_listener, planning_scene_monitor);

  // Wait for some time, so that the planning scene is loaded in rviz.
  // This is just for convenience, should not be used for sync in real application.
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // Get the robot state and joint model group info.
  auto robot_state = planning_scene_monitor->getStateMonitor()->getCurrentState();
  const moveit::core::JointModelGroup* joint_model_group =
      robot_state->getJointModelGroup(servo_params.move_group_name);

  // Set the command type for servo.
  servo.setCommandType(CommandType::TWIST);

  // Move end effector in the +z direction at 5 cm/s
  // while turning around z axis in the +ve direction at 0.4 rad/s
  TwistCommand target_twist{ "panda_link0", { 0.0, 0.0, 0.05, 0.0, 0.0, 0.4 } };

  // Frequency at which commands will be sent to the robot controller.
  rclcpp::WallRate rate(1.0 / servo_params.publish_period);

  std::chrono::seconds timeout_duration(4);
  std::chrono::seconds time_elapsed(0);
  const auto start_time = std::chrono::steady_clock::now();

  // create command queue to build trajectory message and add current robot state
  std::deque<KinematicState> joint_cmd_rolling_window;
  KinematicState current_state = servo.getCurrentRobotState(true /* wait for updated state */);
  updateSlidingWindow(current_state, joint_cmd_rolling_window, servo_params.max_expected_latency, demo_node->now());

  RCLCPP_INFO_STREAM(demo_node->get_logger(), servo.getStatusMessage());
  while (rclcpp::ok())
  {
    KinematicState joint_state = servo.getNextJointState(robot_state, target_twist);
    const StatusCode status = servo.getStatus();

    const auto current_time = std::chrono::steady_clock::now();
    time_elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
    if (time_elapsed > timeout_duration)
    {
      RCLCPP_INFO_STREAM(demo_node->get_logger(), "Timed out");
      break;
    }
    else if (status != StatusCode::INVALID)
    {
      updateSlidingWindow(joint_state, joint_cmd_rolling_window, servo_params.max_expected_latency, demo_node->now());
      if (const auto msg = composeTrajectoryMessage(servo_params, joint_cmd_rolling_window))
      {
        trajectory_outgoing_cmd_pub->publish(msg.value());
      }
      if (!joint_cmd_rolling_window.empty())
      {
        robot_state->setJointGroupPositions(joint_model_group, joint_cmd_rolling_window.back().positions);
        robot_state->setJointGroupVelocities(joint_model_group, joint_cmd_rolling_window.back().velocities);
      }
    }
    rate.sleep();
  }

  RCLCPP_INFO(demo_node->get_logger(), "Exiting demo.");
  rclcpp::shutdown();
}
