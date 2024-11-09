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
 *      Created     : 12/31/2018
 *      Author      : Andy Zelenak, V Mohammed Ibrahim
 *
 */

#include <moveit_servo/servo_node.hpp>
#include <realtime_tools/thread_priority.hpp>
#include <moveit/utils/logger.hpp>

namespace moveit_servo
{

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr ServoNode::get_node_base_interface()
{
  return node_->get_node_base_interface();
}

ServoNode::~ServoNode()
{
  stop_servo_ = true;
  if (servo_loop_thread_.joinable())
    servo_loop_thread_.join();
}

ServoNode::ServoNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("servo_node", options) }
  , stop_servo_{ false }
  , servo_paused_{ false }
  , new_joint_jog_msg_{ false }
  , new_twist_msg_{ false }
  , new_pose_msg_{ false }
{
  moveit::setNodeLoggerName(node_->get_name());

  // Configure SCHED_FIFO and priority
  if (realtime_tools::configure_sched_fifo(servo_params_.thread_priority))
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Enabled SCHED_FIFO and higher thread priority.");
  }
  else
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Could not enable FIFO RT scheduling policy. Continuing with the default.");
  }

  // Check if a realtime kernel is available
  if (!realtime_tools::has_realtime_kernel())
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Realtime kernel is recommended for better performance.");
  }

  std::shared_ptr<servo::ParamListener> servo_param_listener =
      std::make_shared<servo::ParamListener>(node_, "moveit_servo");

  // Create Servo instance
  planning_scene_monitor_ = createPlanningSceneMonitor(node_, servo_param_listener->get_params());
  servo_ = std::make_unique<Servo>(node_, servo_param_listener, planning_scene_monitor_);

  servo_params_ = servo_->getParams();

  // Create subscriber for jointjog
  joint_jog_subscriber_ = node_->create_subscription<control_msgs::msg::JointJog>(
      servo_params_.joint_command_in_topic, rclcpp::SystemDefaultsQoS(),
      [this](const control_msgs::msg::JointJog::ConstSharedPtr& msg) { return jointJogCallback(msg); });

  // Create subscriber for twist
  twist_subscriber_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
      servo_params_.cartesian_command_in_topic, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg) { return twistCallback(msg); });

  // Create subscriber for pose
  pose_subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      servo_params_.pose_command_in_topic, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) { return poseCallback(msg); });

  if (servo_params_.command_out_type == "trajectory_msgs/JointTrajectory")
  {
    trajectory_publisher_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        servo_params_.command_out_topic, rclcpp::SystemDefaultsQoS());
  }
  else if (servo_params_.command_out_type == "std_msgs/Float64MultiArray")
  {
    multi_array_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(servo_params_.command_out_topic,
                                                                                       rclcpp::SystemDefaultsQoS());
  }
  // Create status publisher
  status_publisher_ =
      node_->create_publisher<moveit_msgs::msg::ServoStatus>(servo_params_.status_topic, rclcpp::SystemDefaultsQoS());

  // Create service to enable switching command type
  switch_command_type_ = node_->create_service<moveit_msgs::srv::ServoCommandType>(
      "~/switch_command_type", [this](const std::shared_ptr<moveit_msgs::srv::ServoCommandType::Request>& request,
                                      const std::shared_ptr<moveit_msgs::srv::ServoCommandType::Response>& response) {
        return switchCommandType(request, response);
      });

  // Create service to pause/unpause servoing
  pause_servo_ = node_->create_service<std_srvs::srv::SetBool>(
      "~/pause_servo", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
                              const std::shared_ptr<std_srvs::srv::SetBool::Response>& response) {
        return pauseServo(request, response);
      });

  // Start the servoing loop
  servo_loop_thread_ = std::thread(&ServoNode::servoLoop, this);
}

void ServoNode::pauseServo(const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
                           const std::shared_ptr<std_srvs::srv::SetBool::Response>& response)
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
    std::lock_guard<std::mutex> lock_guard(lock_);
    // Reset the smoothing plugin with the robot's current state in case the robot moved between pausing and unpausing.
    last_commanded_state_ = servo_->getCurrentRobotState(true /* block for current robot state */);
    servo_->resetSmoothing(last_commanded_state_);

    // clear out the command rolling window and reset last commanded state to be the current state
    joint_cmd_rolling_window_.clear();

    // reactivate collision checking
    servo_->setCollisionChecking(true);
    response->message = "Servoing enabled";
  }
}

void ServoNode::switchCommandType(const std::shared_ptr<moveit_msgs::srv::ServoCommandType::Request>& request,
                                  const std::shared_ptr<moveit_msgs::srv::ServoCommandType::Response>& response)
{
  const bool is_valid = (request->command_type >= static_cast<int8_t>(CommandType::MIN)) &&
                        (request->command_type <= static_cast<int8_t>(CommandType::MAX));
  if (is_valid)
  {
    servo_->setCommandType(static_cast<CommandType>(request->command_type));
  }
  else
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Unknown command type " << request->command_type << " requested");
  }
  response->success = (request->command_type == static_cast<int8_t>(servo_->getCommandType()));
}

void ServoNode::jointJogCallback(const control_msgs::msg::JointJog::ConstSharedPtr& msg)
{
  latest_joint_jog_ = *msg;
  new_joint_jog_msg_ = true;
}

void ServoNode::twistCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg)
{
  latest_twist_ = *msg;
  new_twist_msg_ = true;
}

void ServoNode::poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
{
  latest_pose_ = *msg;
  new_pose_msg_ = true;
}

std::optional<KinematicState> ServoNode::processJointJogCommand(const moveit::core::RobotStatePtr& robot_state)
{
  std::optional<KinematicState> next_joint_state = std::nullopt;
  // Reject any other command types that had arrived simultaneously.
  new_twist_msg_ = new_pose_msg_ = false;

  const bool command_stale = (node_->now() - latest_joint_jog_.header.stamp) >=
                             rclcpp::Duration::from_seconds(servo_params_.incoming_command_timeout);
  if (!command_stale)
  {
    JointJogCommand command{ latest_joint_jog_.joint_names, latest_joint_jog_.velocities };
    next_joint_state = servo_->getNextJointState(robot_state, command);
  }
  else
  {
    auto result = servo_->smoothHalt(last_commanded_state_);
    new_joint_jog_msg_ = !result.first;
    if (new_joint_jog_msg_)
    {
      next_joint_state = result.second;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), "Joint jog command timed out. Halting to a stop.");
    }
  }

  return next_joint_state;
}

std::optional<KinematicState> ServoNode::processTwistCommand(const moveit::core::RobotStatePtr& robot_state)
{
  std::optional<KinematicState> next_joint_state = std::nullopt;

  // Mark latest twist command as processed.
  // Reject any other command types that had arrived simultaneously.
  new_joint_jog_msg_ = new_pose_msg_ = false;

  const bool command_stale = (node_->now() - latest_twist_.header.stamp) >=
                             rclcpp::Duration::from_seconds(servo_params_.incoming_command_timeout);
  if (!command_stale)
  {
    const Eigen::Vector<double, 6> velocities{ latest_twist_.twist.linear.x,  latest_twist_.twist.linear.y,
                                               latest_twist_.twist.linear.z,  latest_twist_.twist.angular.x,
                                               latest_twist_.twist.angular.y, latest_twist_.twist.angular.z };
    const TwistCommand command{ latest_twist_.header.frame_id, velocities };
    next_joint_state = servo_->getNextJointState(robot_state, command);
  }
  else
  {
    auto result = servo_->smoothHalt(last_commanded_state_);
    new_twist_msg_ = !result.first;
    if (new_twist_msg_)
    {
      next_joint_state = result.second;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), "Twist command timed out. Halting to a stop.");
    }
  }

  return next_joint_state;
}

std::optional<KinematicState> ServoNode::processPoseCommand(const moveit::core::RobotStatePtr& robot_state)
{
  std::optional<KinematicState> next_joint_state = std::nullopt;

  // Mark latest pose command as processed.
  // Reject any other command types that had arrived simultaneously.
  new_joint_jog_msg_ = new_twist_msg_ = false;

  const bool command_stale = (node_->now() - latest_pose_.header.stamp) >=
                             rclcpp::Duration::from_seconds(servo_params_.incoming_command_timeout);
  if (!command_stale)
  {
    const PoseCommand command = poseFromPoseStamped(latest_pose_);
    next_joint_state = servo_->getNextJointState(robot_state, command);
  }
  else
  {
    auto result = servo_->smoothHalt(last_commanded_state_);
    new_pose_msg_ = !result.first;
    if (new_pose_msg_)
    {
      next_joint_state = result.second;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), "Pose command timed out. Halting to a stop.");
    }
  }

  return next_joint_state;
}

void ServoNode::servoLoop()
{
  moveit_msgs::msg::ServoStatus status_msg;
  std::optional<KinematicState> next_joint_state = std::nullopt;
  rclcpp::WallRate servo_frequency(1 / servo_params_.publish_period);

  // wait for first robot joint state update
  const auto servo_node_start = node_->now();
  while (planning_scene_monitor_->getLastUpdateTime().get_clock_type() != node_->get_clock()->get_clock_type() ||
         servo_node_start > planning_scene_monitor_->getLastUpdateTime())
  {
    RCLCPP_INFO(node_->get_logger(), "Waiting to receive robot state update.");
    rclcpp::sleep_for(std::chrono::seconds(1));
  }
  KinematicState current_state = servo_->getCurrentRobotState(true /* block for current robot state */);
  last_commanded_state_ = current_state;
  // Ensure the filter is up to date
  servo_->resetSmoothing(current_state);

  // Get the robot state and joint model group info.
  moveit::core::RobotStatePtr robot_state = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  const moveit::core::JointModelGroup* joint_model_group =
      robot_state->getJointModelGroup(servo_params_.move_group_name);

  while (rclcpp::ok() && !stop_servo_)
  {
    // Skip processing if servoing is disabled.
    if (servo_paused_)
    {
      servo_->resetSmoothing(current_state);
      servo_frequency.sleep();
      continue;
    }

    std::lock_guard<std::mutex> lock_guard(lock_);
    const bool use_trajectory = servo_params_.command_out_type == "trajectory_msgs/JointTrajectory";
    const auto cur_time = node_->now();

    if (use_trajectory && !joint_cmd_rolling_window_.empty() && joint_cmd_rolling_window_.back().time_stamp > cur_time)
    {
      current_state = joint_cmd_rolling_window_.back();
    }
    else
    {
      // if all joint_cmd_rolling_window_ is empty or all commands in it are outdated, use current robot state
      joint_cmd_rolling_window_.clear();
      current_state = servo_->getCurrentRobotState(false /* block for current robot state */);
      current_state.velocities *= 0.0;
    }

    // update robot state values
    robot_state->setJointGroupPositions(joint_model_group, current_state.positions);
    robot_state->setJointGroupVelocities(joint_model_group, current_state.velocities);

    next_joint_state = std::nullopt;
    const CommandType expected_type = servo_->getCommandType();

    if (expected_type == CommandType::JOINT_JOG && new_joint_jog_msg_)
    {
      next_joint_state = processJointJogCommand(robot_state);
    }
    else if (expected_type == CommandType::TWIST && new_twist_msg_)
    {
      next_joint_state = processTwistCommand(robot_state);
    }
    else if (expected_type == CommandType::POSE && new_pose_msg_)
    {
      next_joint_state = processPoseCommand(robot_state);
    }
    else if (new_joint_jog_msg_ || new_twist_msg_ || new_pose_msg_)
    {
      new_joint_jog_msg_ = new_twist_msg_ = new_pose_msg_ = false;
      RCLCPP_WARN_STREAM(node_->get_logger(), "Command type has not been set, cannot accept input");
    }

    if (next_joint_state && (servo_->getStatus() != StatusCode::INVALID) &&
        (servo_->getStatus() != StatusCode::HALT_FOR_COLLISION))
    {
      if (use_trajectory)
      {
        auto& next_joint_state_value = next_joint_state.value();
        updateSlidingWindow(next_joint_state_value, joint_cmd_rolling_window_, servo_params_.max_expected_latency,
                            cur_time);
        if (const auto msg = composeTrajectoryMessage(servo_params_, joint_cmd_rolling_window_))
        {
          trajectory_publisher_->publish(msg.value());
        }
      }
      else
      {
        multi_array_publisher_->publish(composeMultiArrayMessage(servo_->getParams(), next_joint_state.value()));
      }
      last_commanded_state_ = next_joint_state.value();
    }
    else
    {
      // if no new command was created, use current robot state
      updateSlidingWindow(current_state, joint_cmd_rolling_window_, servo_params_.max_expected_latency, cur_time);
      servo_->resetSmoothing(current_state);
    }

    status_msg.code = static_cast<int8_t>(servo_->getStatus());
    status_msg.message = servo_->getStatusMessage();
    status_publisher_->publish(status_msg);

    servo_frequency.sleep();
  }
}

}  // namespace moveit_servo

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(moveit_servo::ServoNode)
