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
#include <realtime_tools/thread_priority.hpp>

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.servo_node");
}  // namespace

namespace moveit_servo
{

ServoNode::~ServoNode()
{
  stop_servo_ = true;
  if (servo_loop_thread_.joinable())
    servo_loop_thread_.join();
}

ServoNode::ServoNode(const rclcpp::Node::SharedPtr& node)
  : node_(node)
  , stop_servo_{ false }
  , servo_paused_{ false }
  , new_joint_jog_msg_{ false }
  , new_twist_msg_{ false }
  , new_pose_msg_{ false }
{
  std::shared_ptr<servo::ParamListener> servo_param_listener =
      std::make_shared<servo::ParamListener>(node_, "moveit_servo");
  // Create Servo instance
  planning_scene_monitor_ = createPlanningSceneMonitor(node_, servo_param_listener->get_params());
  servo_ = std::make_unique<Servo>(node_, servo_param_listener, planning_scene_monitor_);

  servo_params_ = servo_->getParams();

  // Create subscriber for jointjog
  joint_jog_subscriber_ = node_->create_subscription<control_msgs::msg::JointJog>(
      servo_params_.joint_command_in_topic, 10, std::bind(&ServoNode::jointJogCallback, this, std::placeholders::_1));

  // Create subscriber for twist
  twist_subscriber_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
      servo_params_.cartesian_command_in_topic, 10, std::bind(&ServoNode::twistCallback, this, std::placeholders::_1));

  // Create subscriber for pose
  pose_subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      servo_params_.pose_command_in_topic, 10, std::bind(&ServoNode::poseCallback, this, std::placeholders::_1));

  // Create publisher for joint trajectory message
  trajectory_publisher_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      servo_params_.command_out_topic, rclcpp::SystemDefaultsQoS());

  // Create status publisher
  status_publisher_ =
      node_->create_publisher<std_msgs::msg::Int8>(servo_params_.status_topic, rclcpp::SystemDefaultsQoS());

  // Create service to enable switching command type
  switch_command_type_ = node_->create_service<moveit_msgs::srv::ServoCommandType>(
      "moveit_servo/switch_command_type",
      std::bind(&ServoNode::switchCommandType, this, std::placeholders::_1, std::placeholders::_2));

  // Create service to pause/unpause servoing
  pause_servo_ = node_->create_service<std_srvs::srv::SetBool>("moveit_servo/pause_servo",
                                                               std::bind(&ServoNode::pauseServo, this,
                                                                         std::placeholders::_1, std::placeholders::_2));

  // Check if a realtime kernel is available
  if (realtime_tools::has_realtime_kernel())
  {
    if (realtime_tools::configure_sched_fifo(servo_params_.thread_priority))
      RCLCPP_INFO_STREAM(LOGGER, "Realtime kernel available, higher thread priority has been set.");
    else
      RCLCPP_WARN_STREAM(LOGGER, "Could not enable FIFO RT scheduling policy.");
  }
  else
  {
    RCLCPP_WARN_STREAM(LOGGER, "Realtime kernel is recommended for better performance.");
  }
  // Start the servoing loop
  servo_loop_thread_ = std::thread(&ServoNode::servoLoop, this);
}

void ServoNode::pauseServo(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                           const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  servo_paused_ = request->data;
  response->success = (servo_paused_ == request->data);
  if (servo_paused_)
  {
    servo_->setCollisionChecking(false);
    response->message = "Servoing disabled";
  }
  else
  {
    servo_->setCollisionChecking(true);
    response->message = "Servoing enabled";
  }
}

void ServoNode::switchCommandType(const std::shared_ptr<moveit_msgs::srv::ServoCommandType::Request> request,
                                  const std::shared_ptr<moveit_msgs::srv::ServoCommandType::Response> response)
{
  const bool is_valid = (request->command_type >= 0) && (request->command_type <= 2);
  if (is_valid)
  {
    servo_->expectedCommandType(static_cast<CommandType>(request->command_type));
    response->expected_type = static_cast<int8_t>(servo_->expectedCommandType());
  }
  else
  {
    RCLCPP_WARN_STREAM(LOGGER, "Unknown command type " << request->command_type << " requested");
  }
}

void ServoNode::jointJogCallback(const control_msgs::msg::JointJog::SharedPtr msg)
{
  latest_joint_jog_ = *msg;
  new_joint_jog_msg_ = true;
}

void ServoNode::twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  latest_twist_ = *msg;
  new_twist_msg_ = true;
}

void ServoNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  latest_pose_ = *msg;
  new_pose_msg_ = true;
}

std::optional<KinematicState> ServoNode::processJointJogCommand()
{
  std::optional<KinematicState> next_joint_state = std::nullopt;

  // Mark latest jointjog command as processed.
  // Reject any other command types that had arrived simultaneously.
  new_joint_jog_msg_ = new_twist_msg_ = new_pose_msg_ = false;

  const bool command_stale = (node_->now() - latest_joint_jog_.header.stamp) >=
                             rclcpp::Duration::from_seconds(servo_params_.incoming_command_timeout);
  if (!command_stale)
  {
    // JointJogCommand is an alias for VectorXd, so we can directly make a map and pass it.
    const Eigen::Map<JointJogCommand> command(latest_joint_jog_.velocities.data(), latest_joint_jog_.velocities.size());
    next_joint_state = servo_->getNextJointState(command);
  }
  else
  {
    RCLCPP_WARN(LOGGER, "JointJog command is stale, will not process.");
  }

  return next_joint_state;
}

std::optional<KinematicState> ServoNode::processTwistCommand()
{
  std::optional<KinematicState> next_joint_state = std::nullopt;

  // Mark latest twist command as processed.
  // Reject any other command types that had arrived simultaneously.
  new_joint_jog_msg_ = new_twist_msg_ = new_pose_msg_ = false;

  const bool command_stale = (node_->now() - latest_twist_.header.stamp) >=
                             rclcpp::Duration::from_seconds(servo_params_.incoming_command_timeout);
  if (!command_stale)
  {
    const Eigen::Vector<double, 6> velocities{ latest_twist_.twist.linear.x,  latest_twist_.twist.linear.y,
                                               latest_twist_.twist.linear.z,  latest_twist_.twist.angular.x,
                                               latest_twist_.twist.angular.y, latest_twist_.twist.angular.z };
    const TwistCommand command{ latest_twist_.header.frame_id, velocities };
    next_joint_state = servo_->getNextJointState(command);
  }
  else
  {
    RCLCPP_WARN(LOGGER, "Twist command is stale, will not process.");
  }

  return next_joint_state;
}

std::optional<KinematicState> ServoNode::processPoseCommand()
{
  std::optional<KinematicState> next_joint_state = std::nullopt;

  // Mark latest pose command as processed.
  // Reject any other command types that had arrived simultaneously.
  new_joint_jog_msg_ = new_twist_msg_ = new_pose_msg_ = false;

  const bool command_stale = (node_->now() - latest_pose_.header.stamp) >=
                             rclcpp::Duration::from_seconds(servo_params_.incoming_command_timeout);
  if (!command_stale)
  {
    const PoseCommand command = poseFromPoseStamped(latest_pose_);
    next_joint_state = servo_->getNextJointState(command);
  }
  else
  {
    RCLCPP_WARN(LOGGER, "Pose command is stale, will not process.");
  }

  return next_joint_state;
}

void ServoNode::servoLoop()
{
  std_msgs::msg::Int8 status_msg;
  std::optional<KinematicState> next_joint_state;
  rclcpp::WallRate servo_frequency(1 / servo_params_.publish_period);

  while (rclcpp::ok() && !stop_servo_)
  {
    // Skip processing if servoing is disabled.
    if (servo_paused_)
      continue;

    next_joint_state = std::nullopt;
    const CommandType expectedType = servo_->expectedCommandType();

    if (expectedType == CommandType::JOINT_JOG && new_joint_jog_msg_)
    {
      next_joint_state = processJointJogCommand();
    }
    else if (expectedType == CommandType::TWIST && new_twist_msg_)
    {
      next_joint_state = processTwistCommand();
    }
    else if (expectedType == CommandType::POSE && new_pose_msg_)
    {
      next_joint_state = processPoseCommand();
    }
    else if (new_joint_jog_msg_ || new_twist_msg_ || new_pose_msg_)
    {
      new_joint_jog_msg_ = new_twist_msg_ = new_pose_msg_ = false;
      RCLCPP_WARN_STREAM(LOGGER, "Command type has not been set, cannot accept input");
    }

    if (next_joint_state && (servo_->getStatus() != StatusCode::INVALID))
    {
      trajectory_publisher_->publish(composeTrajectoryMessage(servo_->getParams(), next_joint_state.value()));
    }

    status_msg.data = static_cast<int8_t>(servo_->getStatus());
    status_publisher_->publish(status_msg);
  }

  servo_frequency.sleep();
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
