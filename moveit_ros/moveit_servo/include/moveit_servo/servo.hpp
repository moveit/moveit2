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

/*      Title       : servo.hpp
 *      Project     : moveit_servo
 *      Created     : 05/17/2023
 *      Author      : Brian O'Neil, Andy Zelenak, Blake Anderson, V Mohammed Ibrahim
 *
 *      Description : The core servoing logic.
 */

#pragma once

// Standard Library
#include <variant>

// ROS
#include <pluginlib/class_loader.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

// MoveIt
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/online_signal_smoothing/smoothing_base_class.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_servo/collision_monitor.hpp>
#include <moveit_servo/utils/datatypes.hpp>
#include <moveit_servo/utils/command.hpp>
#include <moveit_servo_lib_parameters.hpp>

namespace moveit_servo
{

class Servo
{
public:
  Servo(const rclcpp::Node::SharedPtr& node, std::shared_ptr<const servo::ParamListener> servo_param_listener,
        const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);

  ~Servo();

  /**
   * \brief Computes the required joint state to follow the given command.
   * @param command The command to follow, std::variant type, can handle JointJog, Twist and Pose.
   * @return The required joint state.
   */
  KinematicState getNextJointState(const ServoInput& command);

  /**
   * \brief Set the type of incoming servo command.
   * @param command_type The type of command servo should expect.
   */
  void expectedCommandType(const CommandType& command_type);

  /**
   * \brief Get the type of command that servo is currently expecting.
   * @return The type of command.
   */
  CommandType expectedCommandType();

  /**
   * \brief Get the current status of servo.
   * @return The current status.
   */
  StatusCode getStatus();

  /**
   * \brief Get the message associated with the current servo status.
   * @return The status message.
   */
  const std::string getStatusMessage();

  /**
   * \brief Returns the end effector pose in planning frame
   */
  const Eigen::Isometry3d getEndEffectorPose();

  /**
   * \brief Start/Stop collision checking thread.
   * @param check_collision Stops collision checking thread if false, starts it if true.
   */
  void setCollisionChecking(const bool check_collision);

private:
  /**
   * \brief Compute the change in joint position for the received command.
   * @param command The incoming servo command.
   * @return The joint position change required (delta).
   */
  Eigen::VectorXd jointDeltaFromCommand(const ServoInput& command);

  /**
   * \brief Converts the given pose to planning frame.
   * @param command The pose to be converted to planning frame.
   */
  Pose toPlanningFrame(const Pose& command);

  /**
   * \brief Converts the given twist to planning frame.
   * @param command The twist to be converted to planning frame.
   */
  Twist toPlanningFrame(const Twist& command);

  /**
   * \brief Make sure that IK solver exists.
   */
  void checkIKSolver();

  /**
   * \brief Validate the servo parameters
   * @param servo_params The servo parameters
   */
  void validateParams(const servo::Params& servo_params);

  /**
   * \brief create and initialize the smoothing plugin to be used by servo.
   */
  void setSmoothingPlugin();

  /**
   * \brief Updates the servo parameters and performs some validations.
   */
  void updateParams();

  /**
   * \brief Apply halting logic to specified joints.
   * @param joints_to_halt The indices of joints to be halted.
   * @param current_state The current kinematic state.
   * @param target_state The target kinematic state.
   * @return The bounded kinematic state.
   */
  KinematicState haltJoints(const std::vector<int>& joints_to_halt, const KinematicState& current_state,
                            const KinematicState& target_state);

  // Variables

  const rclcpp::Node::SharedPtr node_;
  // This needs to be threadsafe so it can be updated in realtime with Dynamic Reconfigure
  std::atomic<CommandType> expected_command_type_;

  servo::Params servo_params_;
  std::shared_ptr<const servo::ParamListener> servo_param_listener_;

  moveit::core::RobotStatePtr robot_state_;
  const moveit::core::JointModelGroup* joint_model_group_;

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  std::atomic<double> collision_velocity_scale_ = 1.0;
  std::unique_ptr<CollisionMonitor> collision_monitor_;
  pluginlib::UniquePtr<online_signal_smoothing::SmoothingBaseClass> smoother_;

  size_t num_joints_;
  std::vector<std::string> joint_names_;
  moveit::core::JointBoundsVector joint_bounds_;

  StatusCode servo_status_;

  tf2_ros::Buffer transform_buffer_;
  tf2_ros::TransformListener transform_listener_;
};

}  // namespace moveit_servo
