/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
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
 *   * Neither the name of PickNik Inc. nor the names of its
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

#include "moveit_servo/pose_tracking.h"
#include "moveit_servo/servo_parameters.h"

#include <chrono>
using namespace std::literals;

namespace
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.pose_tracking");
constexpr size_t LOG_THROTTLE_PERIOD = 10;  // sec

// Helper template for declaring and getting ros param
template <typename T>
void declareOrGetParam(T& output_value, const std::string& param_name, const rclcpp::Node::SharedPtr& node,
                       const rclcpp::Logger& logger, const T default_value = T{})
{
  try
  {
    if (node->has_parameter(param_name))
    {
      node->get_parameter<T>(param_name, output_value);
    }
    else
    {
      output_value = node->declare_parameter<T>(param_name, default_value);
    }
  }
  catch (const rclcpp::exceptions::InvalidParameterTypeException& e)
  {
    RCLCPP_WARN_STREAM(logger, "InvalidParameterTypeException(" << param_name << "): " << e.what());
    RCLCPP_ERROR_STREAM(logger, "Error getting parameter \'" << param_name << "\', check parameter type in YAML file");
    throw e;
  }

  RCLCPP_INFO_STREAM(logger, "Found parameter - " << param_name << ": " << output_value);
}
}  // namespace

namespace moveit_servo
{
PoseTracking::PoseTracking(const rclcpp::Node::SharedPtr& node, const ServoParameters::SharedConstPtr& servo_parameters,
                           const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
  : node_(node)
  , servo_parameters_(servo_parameters)
  , planning_scene_monitor_(planning_scene_monitor)
  , loop_rate_(1.0 / servo_parameters->publish_period)
  , transform_buffer_(node_->get_clock())
  , transform_listener_(transform_buffer_)
  , stop_requested_(false)
{
  readROSParams();

  robot_model_ = planning_scene_monitor_->getRobotModel();

  // Initialize PID controllers
  initializePID(x_pid_config_, cartesian_position_pids_);
  initializePID(y_pid_config_, cartesian_position_pids_);
  initializePID(z_pid_config_, cartesian_position_pids_);
  initializePID(angular_pid_config_, cartesian_orientation_pids_);

  // Use the C++ interface that Servo provides
  servo_ = std::make_unique<moveit_servo::Servo>(node_, servo_parameters_, planning_scene_monitor_);
  servo_->start();

  // Connect to Servo ROS interfaces
  target_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "target_pose", rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { return targetPoseCallback(msg); });

  // Publish outgoing twist commands to the Servo object
  twist_stamped_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
      servo_->getParameters()->cartesian_command_in_topic, rclcpp::SystemDefaultsQoS());
}

PoseTrackingStatusCode PoseTracking::moveToPose(const Eigen::Vector3d& positional_tolerance,
                                                const double angular_tolerance, const double target_pose_timeout)
{
  // Reset stop requested flag before starting motions
  stop_requested_ = false;
  // Wait a bit for a target pose message to arrive.
  // The target pose may get updated by new messages as the robot moves (in a callback function).
  const rclcpp::Time start_time = node_->now();

  while ((!haveRecentTargetPose(target_pose_timeout) || !haveRecentEndEffectorPose(target_pose_timeout)) &&
         ((node_->now() - start_time).seconds() < target_pose_timeout))
  {
    if (servo_->getCommandFrameTransform(command_frame_transform_))
    {
      command_frame_transform_stamp_ = node_->now();
    }
    std::this_thread::sleep_for(1ms);
  }

  if (!haveRecentTargetPose(target_pose_timeout))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "The target pose was not updated recently. Aborting.");
    return PoseTrackingStatusCode::NO_RECENT_TARGET_POSE;
  }

  // Continue sending PID controller output to Servo until one of the following conditions is met:
  // - Goal tolerance is satisfied
  // - Target pose becomes outdated
  // - Command frame transform becomes outdated
  // - Another thread requested a stop
  while (rclcpp::ok())
  {
    if (satisfiesPoseTolerance(positional_tolerance, angular_tolerance))
    {
      RCLCPP_INFO_STREAM(LOGGER, "The target pose is achieved!");
      break;
    }
    // Attempt to update robot pose
    if (servo_->getCommandFrameTransform(command_frame_transform_))
    {
      command_frame_transform_stamp_ = node_->now();
    }

    // Check that end-effector pose (command frame transform) is recent enough.
    if (!haveRecentEndEffectorPose(target_pose_timeout))
    {
      RCLCPP_ERROR_STREAM(LOGGER, "The end effector pose was not updated in time. Aborting.");
      doPostMotionReset();
      return PoseTrackingStatusCode::NO_RECENT_END_EFFECTOR_POSE;
    }

    if (stop_requested_)
    {
      RCLCPP_INFO_STREAM(LOGGER, "Halting servo motion, a stop was requested.");
      doPostMotionReset();
      return PoseTrackingStatusCode::STOP_REQUESTED;
    }

    // Compute servo command from PID controller output and send it to the Servo object, for execution
    twist_stamped_pub_->publish(*calculateTwistCommand());

    if (!loop_rate_.sleep())
    {
      RCLCPP_WARN_STREAM_THROTTLE(LOGGER, *node_->get_clock(), LOG_THROTTLE_PERIOD, "Target control rate was missed");
    }
  }

  doPostMotionReset();
  return PoseTrackingStatusCode::SUCCESS;
}

void PoseTracking::readROSParams()
{
  const std::string ns = "moveit_servo";

  declareOrGetParam(planning_frame_, ns + ".planning_frame", node_, LOGGER);
  declareOrGetParam(move_group_name_, ns + ".move_group_name", node_, LOGGER);

  if (!planning_scene_monitor_->getRobotModel()->hasJointModelGroup(move_group_name_))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Unable to find the specified joint model group: " << move_group_name_);
  }

  double publish_period;
  declareOrGetParam(publish_period, ns + ".publish_period", node_, LOGGER);

  x_pid_config_.dt = publish_period;
  y_pid_config_.dt = publish_period;
  z_pid_config_.dt = publish_period;
  angular_pid_config_.dt = publish_period;

  double windup_limit;
  declareOrGetParam(windup_limit, ns + ".windup_limit", node_, LOGGER);
  x_pid_config_.windup_limit = windup_limit;
  y_pid_config_.windup_limit = windup_limit;
  z_pid_config_.windup_limit = windup_limit;
  angular_pid_config_.windup_limit = windup_limit;

  declareOrGetParam(x_pid_config_.k_p, ns + ".x_proportional_gain", node_, LOGGER);
  declareOrGetParam(x_pid_config_.k_p, ns + ".x_proportional_gain", node_, LOGGER);
  declareOrGetParam(y_pid_config_.k_p, ns + ".y_proportional_gain", node_, LOGGER);
  declareOrGetParam(z_pid_config_.k_p, ns + ".z_proportional_gain", node_, LOGGER);
  declareOrGetParam(x_pid_config_.k_i, ns + ".x_integral_gain", node_, LOGGER);
  declareOrGetParam(y_pid_config_.k_i, ns + ".y_integral_gain", node_, LOGGER);
  declareOrGetParam(z_pid_config_.k_i, ns + ".z_integral_gain", node_, LOGGER);
  declareOrGetParam(x_pid_config_.k_d, ns + ".x_derivative_gain", node_, LOGGER);
  declareOrGetParam(y_pid_config_.k_d, ns + ".y_derivative_gain", node_, LOGGER);
  declareOrGetParam(z_pid_config_.k_d, ns + ".z_derivative_gain", node_, LOGGER);

  declareOrGetParam(angular_pid_config_.k_p, ns + ".angular_proportional_gain", node_, LOGGER);
  declareOrGetParam(angular_pid_config_.k_i, ns + ".angular_integral_gain", node_, LOGGER);
  declareOrGetParam(angular_pid_config_.k_d, ns + ".angular_derivative_gain", node_, LOGGER);
}

void PoseTracking::initializePID(const PIDConfig& pid_config, std::vector<control_toolbox::Pid>& pid_vector)
{
  bool use_anti_windup = true;
  pid_vector.push_back(control_toolbox::Pid(pid_config.k_p, pid_config.k_i, pid_config.k_d, pid_config.windup_limit,
                                            -pid_config.windup_limit, use_anti_windup));
}

bool PoseTracking::haveRecentTargetPose(const double timespan)
{
  std::lock_guard<std::mutex> lock(target_pose_mtx_);
  return ((node_->now() - target_pose_.header.stamp).seconds() < timespan);
}

bool PoseTracking::haveRecentEndEffectorPose(const double timespan)
{
  return ((node_->now() - command_frame_transform_stamp_).seconds() < timespan);
}

bool PoseTracking::satisfiesPoseTolerance(const Eigen::Vector3d& positional_tolerance, const double angular_tolerance)
{
  std::lock_guard<std::mutex> lock(target_pose_mtx_);
  double x_error = target_pose_.pose.position.x - command_frame_transform_.translation()(0);
  double y_error = target_pose_.pose.position.y - command_frame_transform_.translation()(1);
  double z_error = target_pose_.pose.position.z - command_frame_transform_.translation()(2);

  // If uninitialized, likely haven't received the target pose yet.
  if (!angular_error_)
    return false;

  return ((std::abs(x_error) < positional_tolerance(0)) && (std::abs(y_error) < positional_tolerance(1)) &&
          (std::abs(z_error) < positional_tolerance(2)) && (std::abs(*angular_error_) < angular_tolerance));
}

void PoseTracking::targetPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(target_pose_mtx_);
  target_pose_ = *msg;
  // If the target pose is not defined in planning frame, transform the target pose.
  if (target_pose_.header.frame_id != planning_frame_)
  {
    try
    {
      geometry_msgs::msg::TransformStamped target_to_planning_frame = transform_buffer_.lookupTransform(
          planning_frame_, target_pose_.header.frame_id, rclcpp::Time(0), rclcpp::Duration(100ms));
      tf2::doTransform(target_pose_, target_pose_, target_to_planning_frame);

      // Prevent doTransform from copying a stamp of 0, which will cause the haveRecentTargetPose check to fail servo motions
      target_pose_.header.stamp = node_->now();
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_WARN_STREAM(LOGGER, ex.what());
      return;
    }
  }
}

geometry_msgs::msg::TwistStamped::ConstSharedPtr PoseTracking::calculateTwistCommand()
{
  // use the shared pool to create a message more efficiently
  auto msg = moveit::util::make_shared_from_pool<geometry_msgs::msg::TwistStamped>();

  // Get twist components from PID controllers
  geometry_msgs::msg::Twist& twist = msg->twist;
  Eigen::Quaterniond q_desired;

  // Scope mutex locking only to operations which require access to target pose.
  {
    std::lock_guard<std::mutex> lock(target_pose_mtx_);
    msg->header.frame_id = target_pose_.header.frame_id;

    // Position
    twist.linear.x = cartesian_position_pids_[0].computeCommand(
        target_pose_.pose.position.x - command_frame_transform_.translation()(0), loop_rate_.period().count());
    twist.linear.y = cartesian_position_pids_[1].computeCommand(
        target_pose_.pose.position.y - command_frame_transform_.translation()(1), loop_rate_.period().count());
    twist.linear.z = cartesian_position_pids_[2].computeCommand(
        target_pose_.pose.position.z - command_frame_transform_.translation()(2), loop_rate_.period().count());

    // Orientation algorithm:
    // - Find the orientation error as a quaternion: q_error = q_desired * q_current ^ -1
    // - Use the angle-axis PID controller to calculate an angular rate
    // - Convert to angular velocity for the TwistStamped message
    q_desired = Eigen::Quaterniond(target_pose_.pose.orientation.w, target_pose_.pose.orientation.x,
                                   target_pose_.pose.orientation.y, target_pose_.pose.orientation.z);
  }

  Eigen::Quaterniond q_current(command_frame_transform_.rotation());
  Eigen::Quaterniond q_error = q_desired * q_current.inverse();

  // Convert axis-angle to angular velocity
  Eigen::AngleAxisd axis_angle(q_error);
  // Cache the angular error, for rotation tolerance checking
  angular_error_ = axis_angle.angle();

  double ang_vel_magnitude =
      cartesian_orientation_pids_[0].computeCommand(*angular_error_, loop_rate_.period().count());
  twist.angular.x = ang_vel_magnitude * axis_angle.axis()[0];
  twist.angular.y = ang_vel_magnitude * axis_angle.axis()[1];
  twist.angular.z = ang_vel_magnitude * axis_angle.axis()[2];

  msg->header.stamp = node_->now();

  return msg;
}

void PoseTracking::stopMotion()
{
  stop_requested_ = true;

  // Send a 0 command to Servo to halt arm motion
  auto msg = moveit::util::make_shared_from_pool<geometry_msgs::msg::TwistStamped>();
  {
    std::lock_guard<std::mutex> lock(target_pose_mtx_);
    msg->header.frame_id = target_pose_.header.frame_id;
  }
  msg->header.stamp = node_->now();
  twist_stamped_pub_->publish(*msg);
}

void PoseTracking::doPostMotionReset()
{
  stopMotion();
  stop_requested_ = false;
  angular_error_ = {};

  // Reset error integrals and previous errors of PID controllers
  cartesian_position_pids_[0].reset();
  cartesian_position_pids_[1].reset();
  cartesian_position_pids_[2].reset();
  cartesian_orientation_pids_[0].reset();
}

void PoseTracking::updatePIDConfig(const double x_proportional_gain, const double x_integral_gain,
                                   const double x_derivative_gain, const double y_proportional_gain,
                                   const double y_integral_gain, const double y_derivative_gain,
                                   const double z_proportional_gain, const double z_integral_gain,
                                   const double z_derivative_gain, const double angular_proportional_gain,
                                   const double angular_integral_gain, const double angular_derivative_gain)
{
  stopMotion();

  x_pid_config_.k_p = x_proportional_gain;
  x_pid_config_.k_i = x_integral_gain;
  x_pid_config_.k_d = x_derivative_gain;
  y_pid_config_.k_p = y_proportional_gain;
  y_pid_config_.k_i = y_integral_gain;
  y_pid_config_.k_d = y_derivative_gain;
  z_pid_config_.k_p = z_proportional_gain;
  z_pid_config_.k_i = z_integral_gain;
  z_pid_config_.k_d = z_derivative_gain;

  angular_pid_config_.k_p = angular_proportional_gain;
  angular_pid_config_.k_i = angular_integral_gain;
  angular_pid_config_.k_d = angular_derivative_gain;

  cartesian_position_pids_.clear();
  cartesian_orientation_pids_.clear();
  initializePID(x_pid_config_, cartesian_position_pids_);
  initializePID(y_pid_config_, cartesian_position_pids_);
  initializePID(z_pid_config_, cartesian_position_pids_);
  initializePID(angular_pid_config_, cartesian_orientation_pids_);

  doPostMotionReset();
}

void PoseTracking::getPIDErrors(double& x_error, double& y_error, double& z_error, double& orientation_error)
{
  double dummy1, dummy2;
  cartesian_position_pids_.at(0).getCurrentPIDErrors(x_error, dummy1, dummy2);
  cartesian_position_pids_.at(1).getCurrentPIDErrors(y_error, dummy1, dummy2);
  cartesian_position_pids_.at(2).getCurrentPIDErrors(z_error, dummy1, dummy2);
  cartesian_orientation_pids_.at(0).getCurrentPIDErrors(orientation_error, dummy1, dummy2);
}

void PoseTracking::resetTargetPose()
{
  std::lock_guard<std::mutex> lock(target_pose_mtx_);
  target_pose_ = geometry_msgs::msg::PoseStamped();
  target_pose_.header.stamp = rclcpp::Time(RCL_ROS_TIME);
}

bool PoseTracking::getCommandFrameTransform(geometry_msgs::msg::TransformStamped& transform)
{
  return servo_->getCommandFrameTransform(transform);
}
}  // namespace moveit_servo
