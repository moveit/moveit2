/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
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

/*      Title       : servo_node.cpp
 *      Project     : moveit_servo
 *      Created     : 01/07/2023
 *      Author      : V Mohammed Ibrahim
 *
 */

#include <moveit_servo/servo_node.hpp>

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.servo_node");
}

namespace moveit_servo
{

ServoNode::~ServoNode()
{
  stop_servo_ = true;
  if (loop_thread_.joinable())
    loop_thread_.join();
}

ServoNode::ServoNode(const rclcpp::Node::SharedPtr& node) : node_(node)
{
  // Used to break the servo loop thread automatically if exiting
  stop_servo_ = false;
  servo_paused_ = false;
  new_joint_jog_ = false;
  new_twist_ = false;
  tacking_pose_ = false;

  servo_param_listener_ = std::make_shared<servo::ParamListener>(node_, "moveit_servo");
  servo_params_ = servo_param_listener_->get_params();

  // Create Servo instance
  planning_scene_monitor_ = createPlanningSceneMonitor(node_, servo_params_);
  servo_ = std::make_unique<Servo>(node_, servo_param_listener_, planning_scene_monitor_);

  // Create subscriber for jointjog
  joint_jog_subscriber_ = node_->create_subscription<control_msgs::msg::JointJog>(
      servo_params_.joint_command_in_topic, 10,
      [this](const control_msgs::msg::JointJog::SharedPtr msg) { jointJogCallback(msg); });

  // Create subscriber for twist
  twist_subscriber_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
      servo_params_.cartesian_command_in_topic, 10,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) { twistCallback(msg); });

  // Create subscriber for pose
  pose_subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      servo_params_.pose_command_in_topic, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { poseCallback(msg); });

  trajectory_publisher_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      servo_params_.command_out_topic, rclcpp::SystemDefaultsQoS());

  // Create service to enable switching command type
  switch_command_type_ = node_->create_service<moveit_msgs::srv::ServoCommandType>(
      "moveit_servo/switch_command_type",
      std::bind(&ServoNode::switchCommandType, this, std::placeholders::_1, std::placeholders::_2));

  pause_servo_ = node_->create_service<std_srvs::srv::SetBool>("moveit_servo/pause_servo",
                                                               std::bind(&ServoNode::pauseServo, this,
                                                                         std::placeholders::_1, std::placeholders::_2));

  loop_thread_ = std::thread(&ServoNode::ServoLoop, this);
}

void ServoNode::pauseServo(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                           const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  servo_paused_ = request->data;
  response->success = true;
  servo_paused_ ? response->message = "Servoing disabled" : response->message = "Servoing enabled";
}

void ServoNode::switchCommandType(const std::shared_ptr<moveit_msgs::srv::ServoCommandType::Request> request,
                                  const std::shared_ptr<moveit_msgs::srv::ServoCommandType::Response> response)
{
  const bool is_valid = request->command_type >= 0 && request->command_type <= 2;
  if (is_valid)
  {
    servo_->expectedCommandType(static_cast<CommandType>(request->command_type));
    response->expected_type = static_cast<int8_t>(servo_->expectedCommandType());
  }
  else
  {
    RCLCPP_WARN_STREAM(LOGGER, "Unknown command type " << request->command_type << "requested");
  }
}

void ServoNode::moveToPose()
{
  RCLCPP_INFO(LOGGER, "Start pose tracking..");
  Pose command = servo_->toPlanningFrame(poseFromPoseStamped(latest_pose_));
  rclcpp::WallRate tracking_frequency(50);
  std::chrono::seconds timeout_duration(3);
  std::chrono::seconds time_elapsed(0);
  auto start_time = std::chrono::steady_clock::now();
  KinematicState next_joint_states(7);
  tacking_pose_ = true;
  while (rclcpp::ok())
  {
    next_joint_states = servo_->getNextJointState(command);
    StatusCode status = servo_->getStatus();
    auto current_time = std::chrono::steady_clock::now();
    time_elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);

    if (time_elapsed > timeout_duration)
    {
      RCLCPP_INFO_STREAM(LOGGER, "Timed out while tracking.");
      break;
    }
    else if (status == StatusCode::POSE_ACHIEVED or status == StatusCode::INVALID)
    {
      RCLCPP_INFO_STREAM(LOGGER, servo_->getStatusMessage());
      tacking_pose_ = false;
      break;
    }
    else
    {
      trajectory_publisher_->publish(composeTrajectoryMessage(servo_params_, next_joint_states));
    }
    tracking_frequency.sleep();
  }
}

void ServoNode::ServoLoop()
{
  rclcpp::WallRate servo_rate(1 / servo_params_.publish_period);
  KinematicState next_joint_states(7);
  bool publish_command = false;

  while (rclcpp::ok() && !stop_servo_)
  {
    // Skip processing if servoing is disabled.
    if (servo_paused_)
      continue;

    CommandType expectedType = servo_->expectedCommandType();
    if (expectedType == CommandType::JOINT_JOG && new_joint_jog_)
    {
      servo_->expectedCommandType(CommandType::JOINT_JOG);
      Eigen::Map<Eigen::VectorXd> command(latest_joint_jog_.velocities.data(), latest_joint_jog_.velocities.size());
      next_joint_states = servo_->getNextJointState(command);
      publish_command = true;
      new_joint_jog_ = false;
    }
    else if (expectedType == CommandType::TWIST && new_twist_)
    {
      Eigen::Vector<double, 6> velocities{ latest_twist_.twist.linear.x,  latest_twist_.twist.linear.y,
                                           latest_twist_.twist.linear.z,  latest_twist_.twist.angular.x,
                                           latest_twist_.twist.angular.y, latest_twist_.twist.angular.z };
      Twist command{ servo_params_.planning_frame, velocities };
      next_joint_states = servo_->getNextJointState(command);
      publish_command = true;
      new_twist_ = false;
    }

    else if (!tacking_pose_ && expectedType == CommandType::POSE && new_pose_)
    {
      new_pose_ = false;
      if (pose_tracking_thread_.joinable())
        pose_tracking_thread_.join();
      pose_tracking_thread_ = std::thread(&ServoNode::moveToPose, this);
    }

    if (publish_command)
    {
      trajectory_publisher_->publish(composeTrajectoryMessage(servo_params_, next_joint_states));
      publish_command = false;
    }
  }

  servo_rate.sleep();
}

void ServoNode::jointJogCallback(const control_msgs::msg::JointJog::SharedPtr msg)
{
  latest_joint_jog_ = *msg;
  new_joint_jog_ = true;
}

void ServoNode::twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  latest_twist_ = *msg;
  new_twist_ = true;
}

void ServoNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  latest_pose_ = *msg;
  new_pose_ = true;
}

}  // namespace moveit_servo

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("moveit_servo");
  auto servo_node = std::make_unique<moveit_servo::ServoNode>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
