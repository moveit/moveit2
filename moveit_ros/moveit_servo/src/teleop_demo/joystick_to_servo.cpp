/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik LLC
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

/*      Title     : joystick_to_servo.cpp
 *      Project   : moveit_servo
 *      Created   : 18/02/2021
 *      Author    : Denis Stogl
 */

#include <moveit_servo/joystick_to_servo.hpp>

#include <mutex>
#include <utility>
#include <thread>

#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

const size_t ROS_QUEUE_SIZE = 10;

namespace moveit_servo
{

JoystickToServo::JoystickToServo(const rclcpp::NodeOptions& options)
: rclcpp::Node("joystick_to_servo", options)
{
  // Declare parameters
  this->declare_parameter<std::vector<std::string>>(
    "cartesian_command_frames", cartesian_command_frames_);
  this->declare_parameter<int>("enable_joint_commands_button", enable_joint_commands_button_);
  this->declare_parameter<int>("enable_twist_commands_button", enable_twist_commands_button_);
  this->declare_parameter<int>(
    "switch_cartesian_command_frame_button", switch_cartesian_command_frame_button_);
  this->declare_parameter<int>(
    "switch_joint_axis_mode_button", switch_joint_axis_mode_button_);
  this->declare_parameter<std::vector<std::string>>("joints", joint_names_);
  this->declare_parameter<int>("num_joint_axis_control_modes", 0);

  //TODO(denis): add info output regarding parameters
  // Read parameters
  cartesian_command_frames_ = this->get_parameter("cartesian_command_frames").as_string_array();
  enable_joint_commands_button_ = this->get_parameter("enable_joint_commands_button").as_int();
  enable_twist_commands_button_ = this->get_parameter("enable_twist_commands_button").as_int();
  switch_cartesian_command_frame_button_ = this->get_parameter("switch_cartesian_command_frame_button").as_int();
  switch_joint_axis_mode_button_ = this->get_parameter("switch_joint_axis_mode_button").as_int();

  RCLCPP_INFO(this->get_logger(),
              "Defined %d cartesian command frames.", cartesian_command_frames_.size());
  RCLCPP_INFO(this->get_logger(),
              "Enable button for joint commands: %d", enable_joint_commands_button_);
  RCLCPP_INFO(this->get_logger(),
              "Enable button for twist commands: %d", enable_twist_commands_button_);
  RCLCPP_INFO(this->get_logger(),
              "Button to switch command frame: %d", switch_cartesian_command_frame_button_);
  RCLCPP_INFO(this->get_logger(),
              "Button to switch joint to joystick mapping mode: %d",
              switch_joint_axis_mode_button_);

  joint_names_ = this->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_WARN(this->get_logger(),
                "No joint names specified. Only cartesian/twist control is available.");
  } else {
    RCLCPP_INFO(this->get_logger(), "Got list with %d joints.", joint_names_.size());
    uint num_joint_axis_control_modes = this->get_parameter("num_joint_axis_control_modes").as_int();
    RCLCPP_INFO(this->get_logger(),
                "There are %d joint axis mapping-modes defined.", num_joint_axis_control_modes);
    joint_to_joystick_axes_.resize(num_joint_axis_control_modes);
    for (uint j = 0; j < num_joint_axis_control_modes; ++j) {
      joint_to_joystick_axes_[j].resize(joint_names_.size(), -1);
      for (uint i = 0; i < joint_names_.size(); ++i) {
        // Declare and read joint axis parameters
        std::string param_name = "joint" + std::to_string(i+1) + "_axis";
        if (num_joint_axis_control_modes > 1) {
          param_name += "." + std::to_string(j+1);
        }
        this->declare_parameter<int>(param_name, joint_to_joystick_axes_[j][i]);
        // TODO: Maybe it will not work so fast...
        joint_to_joystick_axes_[j][i] = this->get_parameter(param_name).as_int();
        RCLCPP_INFO(this->get_logger(),
                    "Joint '%s' using axis: %d in control mode %d.",
                    joint_names_[i].c_str(), joint_to_joystick_axes_[j][i], j+1);
      }
    }
  }

  // Setup pub/sub
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", ROS_QUEUE_SIZE, std::bind(&JoystickToServo::joystickCallback, this, std::placeholders::_1));
  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "~/input_twist", ROS_QUEUE_SIZE, std::bind(
      &JoystickToServo::inputTwistCallback, this, std::placeholders::_1));

  joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
    "~/servo_joint_commands", ROS_QUEUE_SIZE);
  twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "~/servo_twist_commands", ROS_QUEUE_SIZE);

  // Create a service client to start the ServoServer
  servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_server/start_servo");
  servo_start_client_->wait_for_service(std::chrono::seconds(1));
  servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
}


void JoystickToServo::convertJoyToCmd(
  const std::vector<float> & axes, const std::vector<int> & buttons)
{
  if (buttons[switch_cartesian_command_frame_button_]) {
    command_frame_index_ = (command_frame_index_ + 1) % cartesian_command_frames_.size();
  }
  if (buttons[switch_joint_axis_mode_button_]) {
    joint_axis_mode_index_ = (joint_axis_mode_index_ + 1) % joint_to_joystick_axes_.size();
    RCLCPP_INFO(this->get_logger(), "Using joint axis control mode: %d", joint_axis_mode_index_+1);
  }

  if (buttons[enable_joint_commands_button_] && buttons[enable_twist_commands_button_]) {
    RCLCPP_DEBUG(this->get_logger(), "Both enalble buttons are pressed. Not moving!");
    return;
  }

  if (buttons[enable_joint_commands_button_]) {
    auto joint_msg = control_msgs::msg::JointJog();
    joint_msg.header.stamp = this->now();

    for (uint i = 0; i < joint_names_.size(); ++i) {
      joint_msg.joint_names.push_back(joint_names_[i]);
      if (joint_to_joystick_axes_[joint_axis_mode_index_][i] != -1) {
        joint_msg.velocities.push_back(axes[joint_to_joystick_axes_[joint_axis_mode_index_][i]]);
      } else {
        joint_msg.velocities.push_back(0.0);
      }
    }

    joint_pub_->publish(joint_msg);
  }

  if (buttons[enable_twist_commands_button_]) {
    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.header.frame_id = cartesian_command_frames_[command_frame_index_];
    twist_msg.header.stamp = this->now();

    std::lock_guard<std::recursive_mutex> guard(twist_update_lock_);
    twist_msg.twist = *input_twist_;
    twist_pub_->publish(twist_msg);
  }
}

void JoystickToServo::inputTwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (twist_update_lock_.try_lock()) {
    input_twist_ = msg;
  }
}

void JoystickToServo::joystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // This call updates the frame for twist commands
  convertJoyToCmd(msg->axes, msg->buttons);
}

}  // namespace moveit_servo

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit_servo::JoystickToServo)
