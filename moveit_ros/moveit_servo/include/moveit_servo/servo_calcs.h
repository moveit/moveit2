/*******************************************************************************
 *      Title     : servo_calcs.h
 *      Project   : moveit_servo
 *      Created   : 1/11/2019
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 *
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

#pragma once

// C++
#include <condition_variable>
#include <mutex>
#include <thread>
#include <atomic>

// ROS
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/srv/change_drift_dimensions.hpp>
#include <moveit_msgs/srv/change_control_dimensions.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_srvs/srv/empty.hpp>
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// moveit_core
#include <moveit/kinematics_base/kinematics_base.h>

// moveit_servo
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/status_codes.h>
#include <moveit/online_signal_smoothing/smoothing_base_class.h>

namespace moveit_servo
{
enum class ServoType
{
  CARTESIAN_SPACE,
  JOINT_SPACE
};

class ServoCalcs
{
public:
  ServoCalcs(rclcpp::Node::SharedPtr node, const std::shared_ptr<const moveit_servo::ServoParameters>& parameters,
             const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);

  ~ServoCalcs();

  /** \brief Start the timer where we do work and publish outputs */
  void start();

  /**
   * Get the MoveIt planning link transform.
   * The transform from the MoveIt planning frame to robot_link_command_frame
   *
   * @param transform the transform that will be calculated
   * @return true if a valid transform was available
   */
  bool getCommandFrameTransform(Eigen::Isometry3d& transform);
  bool getCommandFrameTransform(geometry_msgs::msg::TransformStamped& transform);

  /**
   * Get the End Effector link transform.
   * The transform from the MoveIt planning frame to EE link
   *
   * @param transform the transform that will be calculated
   * @return true if a valid transform was available
   */
  bool getEEFrameTransform(Eigen::Isometry3d& transform);
  bool getEEFrameTransform(geometry_msgs::msg::TransformStamped& transform);

  /** \brief Pause or unpause processing servo commands while keeping the timers alive */
  void setPaused(bool paused);

protected:
  /** \brief Run the main calculation loop */
  void mainCalcLoop();

  /** \brief Do calculations for a single iteration. Publish one outgoing command */
  void calculateSingleIteration();

  /** \brief Stop the currently running thread */
  void stop();

  /** \brief Do servoing calculations for Cartesian twist commands. */
  bool cartesianServoCalcs(geometry_msgs::msg::TwistStamped& cmd,
                           trajectory_msgs::msg::JointTrajectory& joint_trajectory);

  /** \brief Do servoing calculations for direct commands to a joint. */
  bool jointServoCalcs(const control_msgs::msg::JointJog& cmd, trajectory_msgs::msg::JointTrajectory& joint_trajectory);

  /** \brief Parse the incoming joint msg for the joints of our MoveGroup */
  void updateJoints();

  /**
   * Checks a JointJog msg for valid (non-NaN) velocities
   * @param cmd the desired joint servo command
   * @return true if this represents a valid joint servo command, false otherwise
   */
  bool checkValidCommand(const control_msgs::msg::JointJog& cmd);

  /**
   * Checks a TwistStamped msg for valid (non-NaN) velocities
   * @param cmd the desired twist servo command
   * @return true if this represents a valid servo twist command, false otherwise
   */
  bool checkValidCommand(const geometry_msgs::msg::TwistStamped& cmd);

  /** \brief If incoming velocity commands are from a unitless joystick, scale them to physical units.
   * Also, multiply by timestep to calculate a position change.
   * @return a vector of position deltas
   */
  Eigen::VectorXd scaleCartesianCommand(const geometry_msgs::msg::TwistStamped& command);

  /** \brief If incoming velocity commands are from a unitless joystick, scale them to physical units.
   * Also, multiply by timestep to calculate a position change.
   * @return a vector of position deltas
   */
  Eigen::VectorXd scaleJointCommand(const control_msgs::msg::JointJog& command);

  /** \brief Come to a halt in a smooth way. Apply a smoothing plugin, if one is configured.
   */
  void filteredHalt(trajectory_msgs::msg::JointTrajectory& joint_trajectory);

  /** \brief Suddenly halt for a joint limit or other critical issue.
   * Is handled differently for position vs. velocity control.
   */
  void suddenHalt(sensor_msgs::msg::JointState& joint_state,
                  const std::vector<const moveit::core::JointModel*>& joints_to_halt) const;

  /** \brief Avoid overshooting joint limits
      \return Vector of the joints that would move farther past position margin limits
   */
  std::vector<const moveit::core::JointModel*> enforcePositionLimits(sensor_msgs::msg::JointState& joint_state) const;

  /** \brief Possibly calculate a velocity scaling factor, due to proximity of
   * singularity and direction of motion
   */
  double velocityScalingFactorForSingularity(const Eigen::VectorXd& commanded_velocity,
                                             const Eigen::JacobiSVD<Eigen::MatrixXd>& svd,
                                             const Eigen::MatrixXd& pseudo_inverse);

  /** \brief Compose the outgoing JointTrajectory message */
  void composeJointTrajMessage(const sensor_msgs::msg::JointState& joint_state,
                               trajectory_msgs::msg::JointTrajectory& joint_trajectory);

  /** \brief Set the filters to the specified values */
  void resetLowPassFilters(const sensor_msgs::msg::JointState& joint_state);

  /** \brief Handles all aspects of the servoing after the desired joint commands are established
   * Joint and Cartesian calcs feed into here
   * Handles limit enforcement, internal state updated, collision scaling, and publishing the commands
   * @param delta_theta Eigen vector of joint delta's, from joint or Cartesian servo calcs
   * @param joint_trajectory Output trajectory message
   */
  bool internalServoUpdate(Eigen::ArrayXd& delta_theta, trajectory_msgs::msg::JointTrajectory& joint_trajectory,
                           const ServoType servo_type);

  /** \brief Joint-wise update of a sensor_msgs::msg::JointState with given delta's
   * Also filters and calculates the previous velocity
   * @param delta_theta Eigen vector of joint delta's
   * @param joint_state The joint state msg being updated
   * @param previous_vel Eigen vector of previous velocities being updated
   * @return Returns false if there is a problem, true otherwise
   */
  bool applyJointUpdate(const Eigen::ArrayXd& delta_theta, sensor_msgs::msg::JointState& joint_state);

  /** \brief Gazebo simulations have very strict message timestamp requirements.
   * Satisfy Gazebo by stuffing multiple messages into one.
   */
  void insertRedundantPointsIntoTrajectory(trajectory_msgs::msg::JointTrajectory& joint_trajectory, int count) const;

  /**
   * Remove the Jacobian row and the delta-x element of one Cartesian dimension, to take advantage of task redundancy
   *
   * @param matrix The Jacobian matrix.
   * @param delta_x Vector of Cartesian delta commands, should be the same size as matrix.rows()
   * @param row_to_remove Dimension that will be allowed to drift, e.g. row_to_remove = 2 allows z-translation drift.
   */
  void removeDimension(Eigen::MatrixXd& matrix, Eigen::VectorXd& delta_x, unsigned int row_to_remove) const;

  /**
   * Removes all of the drift dimensions from the jacobian and delta-x element
   *
   * @param matrix The Jacobian matrix.
   * @param delta_x Vector of Cartesian delta commands, should be the same size as matrix.rows()
   */
  void removeDriftDimensions(Eigen::MatrixXd& matrix, Eigen::VectorXd& delta_x);

  /**
   * Uses control_dimensions_ to set the incoming twist command values to 0 in uncontrolled directions
   *
   * @param command TwistStamped msg being used in the Cartesian calcs process
   */
  void enforceControlDimensions(geometry_msgs::msg::TwistStamped& command);

  /* \brief Callback for joint subsription */
  void jointStateCB(const sensor_msgs::msg::JointState::SharedPtr msg);

  /* \brief Command callbacks */
  void twistStampedCB(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void jointCmdCB(const control_msgs::msg::JointJog::SharedPtr msg);
  void collisionVelocityScaleCB(const std_msgs::msg::Float64::SharedPtr msg);

  /**
   * Allow drift in certain dimensions. For example, may allow the wrist to rotate freely.
   * This can help avoid singularities.
   *
   * @param request the service request
   * @param response the service response
   * @return true if the adjustment was made
   */
  void changeDriftDimensions(const std::shared_ptr<moveit_msgs::srv::ChangeDriftDimensions::Request> req,
                             std::shared_ptr<moveit_msgs::srv::ChangeDriftDimensions::Response> res);

  /** \brief Start the main calculation timer */
  // Service callback for changing servoing dimensions
  void changeControlDimensions(const std::shared_ptr<moveit_msgs::srv::ChangeControlDimensions::Request> req,
                               std::shared_ptr<moveit_msgs::srv::ChangeControlDimensions::Response> res);

  /** \brief Service callback to reset Servo status, e.g. so the arm can move again after a collision */
  bool resetServoStatus(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                        std::shared_ptr<std_srvs::srv::Empty::Response> res);

  // Pointer to the ROS node
  std::shared_ptr<rclcpp::Node> node_;

  // Parameters from yaml
  const std::shared_ptr<const moveit_servo::ServoParameters> parameters_;

  // Pointer to the collision environment
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Track the number of cycles during which motion has not occurred.
  // Will avoid re-publishing zero velocities endlessly.
  int zero_velocity_count_ = 0;

  // Flag for staying inactive while there are no incoming commands
  bool wait_for_servo_commands_ = true;

  // Flag saying if the filters were updated during the timer callback
  bool updated_filters_ = false;

  // Nonzero status flags
  bool have_nonzero_twist_stamped_ = false;
  bool have_nonzero_joint_command_ = false;
  bool have_nonzero_command_ = false;

  // Incoming command messages
  geometry_msgs::msg::TwistStamped twist_stamped_cmd_;
  control_msgs::msg::JointJog joint_servo_cmd_;

  const moveit::core::JointModelGroup* joint_model_group_;

  moveit::core::RobotStatePtr current_state_;

  // (mutex protected below)
  // internal_joint_state_ is used in servo calculations. It shouldn't be relied on to be accurate.
  // original_joint_state_ is the same as incoming_joint_state_ except it only contains the joints the servo node acts
  // on.
  sensor_msgs::msg::JointState internal_joint_state_, original_joint_state_;
  std::map<std::string, std::size_t> joint_state_name_map_;

  // Smoothing algorithm (loads a plugin)
  std::shared_ptr<online_signal_smoothing::SmoothingBaseClass> smoother_;

  trajectory_msgs::msg::JointTrajectory::SharedPtr last_sent_command_;

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_sub_;
  rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr joint_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr collision_velocity_scale_sub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr worst_case_stop_time_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_outgoing_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr multiarray_outgoing_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr condition_pub_;
  rclcpp::Service<moveit_msgs::srv::ChangeControlDimensions>::SharedPtr control_dimensions_server_;
  rclcpp::Service<moveit_msgs::srv::ChangeDriftDimensions>::SharedPtr drift_dimensions_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_servo_status_;

  // Main tracking / result publisher loop
  std::thread thread_;
  bool stop_requested_;
  std::atomic<bool> done_stopping_;

  // Status
  StatusCode status_ = StatusCode::NO_WARNING;
  std::atomic<bool> paused_;
  bool twist_command_is_stale_ = false;
  bool joint_command_is_stale_ = false;
  bool ok_to_publish_ = false;
  double collision_velocity_scale_ = 1.0;

  // Use ArrayXd type to enable more coefficient-wise operations
  Eigen::ArrayXd delta_theta_;

  const int gazebo_redundant_message_count_ = 30;

  unsigned int num_joints_;

  // True -> allow drift in this dimension. In the command frame. [x, y, z, roll, pitch, yaw]
  std::array<bool, 6> drift_dimensions_ = { { false, false, false, false, false, false } };

  // The dimensions to control. In the command frame. [x, y, z, roll, pitch, yaw]
  std::array<bool, 6> control_dimensions_ = { { true, true, true, true, true, true } };

  // main_loop_mutex_ is used to protect the input state and dynamic parameters
  mutable std::mutex main_loop_mutex_;
  Eigen::Isometry3d tf_moveit_to_robot_cmd_frame_;
  Eigen::Isometry3d tf_moveit_to_ee_frame_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr latest_twist_stamped_;
  control_msgs::msg::JointJog::ConstSharedPtr latest_joint_cmd_;
  rclcpp::Time latest_twist_command_stamp_ = rclcpp::Time(0., RCL_ROS_TIME);
  rclcpp::Time latest_joint_command_stamp_ = rclcpp::Time(0., RCL_ROS_TIME);
  bool latest_twist_cmd_is_nonzero_ = false;
  bool latest_joint_cmd_is_nonzero_ = false;

  // input condition variable used for low latency mode
  std::condition_variable input_cv_;
  bool new_input_cmd_ = false;

  // dynamic parameters
  std::string robot_link_command_frame_;
  rcl_interfaces::msg::SetParametersResult robotLinkCommandFrameCallback(const rclcpp::Parameter& parameter);

  // Load a smoothing plugin
  pluginlib::ClassLoader<online_signal_smoothing::SmoothingBaseClass> smoothing_loader_;

  kinematics::KinematicsBaseConstPtr ik_solver_;
  Eigen::Isometry3d ik_base_to_tip_frame_;
  bool use_inv_jacobian_ = false;
};
}  // namespace moveit_servo
