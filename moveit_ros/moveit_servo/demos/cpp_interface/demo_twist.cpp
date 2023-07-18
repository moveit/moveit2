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

using namespace moveit_servo;

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.twist_demo");
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // The servo object expects to get a ROS node.
  auto demo_node = std::make_shared<rclcpp::Node>("moveit_servo_demo");

  // Get the servo parameters.
  std::string param_namespace = "moveit_servo";
  auto servo_param_listener = std::make_shared<const servo::ParamListener>(demo_node, param_namespace);
  auto servo_params = servo_param_listener->get_params();

  // The publisher to send trajectory message to the robot controller.
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_outgoing_cmd_pub =
      demo_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(servo_params.command_out_topic,
                                                                         rclcpp::SystemDefaultsQoS());

  // Create the servo object
  auto planning_scene_monitor = createPlanningSceneMonitor(demo_node, servo_params);
  auto servo = Servo(demo_node, servo_param_listener, planning_scene_monitor);

  // Wait for some time, so that the planning scene is loaded in rviz.
  // This is just for convenience, should not be used for sync in real application.
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // Set the command type for servo.
  servo.expectedCommandType(CommandType::TWIST);
  // Move end effector in the +z direction at 10 cm/s (in ee_frame)
  // while turning around z axis in the +ve direction at 0.5 rad/s (in ee_frame)
  TwistCommand target_twist{ servo_params.ee_frame, { 0.0, 0.0, 0.1, 0.0, 0.0, 0.5 } };
  // Convert the command to planning frame.
  tf2_ros::Buffer transform_buffer(demo_node->get_clock());
  tf2_ros::TransformListener transform_listener(transform_buffer);
  auto command_to_planning_frame = transform_buffer.lookupTransform(servo_params.planning_frame, target_twist.frame_id,
                                                                    rclcpp::Time(0), rclcpp::Duration::from_seconds(2));
  Eigen::VectorXd transformed_twist = target_twist.velocities;
  tf2::doTransform(transformed_twist, transformed_twist, command_to_planning_frame);
  target_twist = TwistCommand{ servo_params.planning_frame, transformed_twist };

  // Frequency at which the commands will be send to robot controller.
  rclcpp::WallRate rate(1.0 / servo_params.publish_period);

  std::chrono::seconds timeout_duration(5);
  std::chrono::seconds time_elapsed(0);
  auto start_time = std::chrono::steady_clock::now();

  RCLCPP_INFO_STREAM(LOGGER, servo.getStatusMessage());
  while (rclcpp::ok())
  {
    KinematicState joint_state = servo.getNextJointState(target_twist);
    StatusCode status = servo.getStatus();

    auto current_time = std::chrono::steady_clock::now();
    time_elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
    if (time_elapsed > timeout_duration)
    {
      RCLCPP_INFO_STREAM(LOGGER, "Timed out");
      break;
    }
    else if (status != StatusCode::INVALID)
    {
      trajectory_outgoing_cmd_pub->publish(composeTrajectoryMessage(servo_params, joint_state));
    }
    rate.sleep();
  }

  // Reverse the previous operation by applying the same twist, but in planning frame.
  TwistCommand target_twist_reverse{ servo_params.planning_frame, { 0.0, 0.0, 0.1, 0.0, 0.0, 0.5 } };
  start_time = std::chrono::steady_clock::now();
  while (rclcpp::ok())
  {
    KinematicState joint_state = servo.getNextJointState(target_twist_reverse);
    StatusCode status = servo.getStatus();

    auto current_time = std::chrono::steady_clock::now();
    time_elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
    if (time_elapsed > timeout_duration)
    {
      RCLCPP_INFO_STREAM(LOGGER, "Timed out");
      break;
    }
    else if (status != StatusCode::INVALID)
    {
      trajectory_outgoing_cmd_pub->publish(composeTrajectoryMessage(servo_params, joint_state));
    }
    rate.sleep();
  }

  RCLCPP_INFO(LOGGER, "Exiting demo.");
  rclcpp::shutdown();
}
