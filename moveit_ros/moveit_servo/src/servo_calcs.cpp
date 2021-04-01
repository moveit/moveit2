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

/*      Title     : servo_calcs.cpp
 *      Project   : moveit_servo
 *      Created   : 1/11/2019
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 */

#include <cassert>
#include <thread>
#include <chrono>
#include <mutex>

#include <std_msgs/msg/bool.h>

// #include <moveit_servo/make_shared_from_pool.h> // TODO(adamp): create an issue about this
#include <moveit_servo/servo_calcs.h>

using namespace std::chrono_literals;  // for s, ms, etc.

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.servo_calcs");
constexpr auto ROS_LOG_THROTTLE_PERIOD = std::chrono::nanoseconds(30ms).count();

namespace moveit_servo
{
namespace
{
// Helper function for detecting zeroed message
bool isNonZero(const geometry_msgs::msg::TwistStamped& msg)
{
  return msg.twist.linear.x != 0.0 || msg.twist.linear.y != 0.0 || msg.twist.linear.z != 0.0 ||
         msg.twist.angular.x != 0.0 || msg.twist.angular.y != 0.0 || msg.twist.angular.z != 0.0;
}

// Helper function for detecting zeroed message
bool isNonZero(const control_msgs::msg::JointJog& msg)
{
  bool all_zeros = true;
  for (double delta : msg.velocities)
  {
    all_zeros &= (delta == 0.0);
  };
  return !all_zeros;
}

// Helper function for converting Eigen::Isometry3d to geometry_msgs/TransformStamped
geometry_msgs::msg::TransformStamped convertIsometryToTransform(const Eigen::Isometry3d& eigen_tf,
                                                                const std::string& parent_frame,
                                                                const std::string& child_frame)
{
  geometry_msgs::msg::TransformStamped output = tf2::eigenToTransform(eigen_tf);
  output.header.frame_id = parent_frame;
  output.child_frame_id = child_frame;

  return output;
}
}  // namespace

// Constructor for the class that handles servoing calculations
ServoCalcs::ServoCalcs(rclcpp::Node::SharedPtr node,
                       const std::shared_ptr<const moveit_servo::ServoParameters>& parameters,
                       const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
  : node_(node)
  , parameters_(parameters)
  , planning_scene_monitor_(planning_scene_monitor)
  , stop_requested_(true)
  , paused_(false)
  , robot_link_command_frame_(parameters->robot_link_command_frame)
{
  // Register callback for changes in robot_link_command_frame
  parameters_->registerSetParameterCallback(parameters->ns + ".robot_link_command_frame",
                                            std::bind(&ServoCalcs::robotLinkCommandFrameCallback, this,
                                                      std::placeholders::_1));

  // MoveIt Setup
  current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  joint_model_group_ = current_state_->getJointModelGroup(parameters_->move_group_name);
  prev_joint_velocity_ = Eigen::ArrayXd::Zero(joint_model_group_->getActiveJointModels().size());

  // Subscribe to command topics
  using std::placeholders::_1;
  using std::placeholders::_2;
  twist_stamped_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
      parameters_->cartesian_command_in_topic, ROS_QUEUE_SIZE, std::bind(&ServoCalcs::twistStampedCB, this, _1));

  joint_cmd_sub_ = node_->create_subscription<control_msgs::msg::JointJog>(
      parameters_->joint_command_in_topic, ROS_QUEUE_SIZE, std::bind(&ServoCalcs::jointCmdCB, this, _1));

  // ROS Server for allowing drift in some dimensions
  drift_dimensions_server_ = node_->create_service<moveit_msgs::srv::ChangeDriftDimensions>(
      "~/change_drift_dimensions", std::bind(&ServoCalcs::changeDriftDimensions, this, _1, _2));

  // ROS Server for changing the control dimensions
  control_dimensions_server_ = node_->create_service<moveit_msgs::srv::ChangeControlDimensions>(
      "~/change_control_dimensions", std::bind(&ServoCalcs::changeControlDimensions, this, _1, _2));

  // ROS Server to reset the status, e.g. so the arm can move again after a collision
  reset_servo_status_ = node_->create_service<std_srvs::srv::Empty>(
      "~/reset_servo_status", std::bind(&ServoCalcs::resetServoStatus, this, _1, _2));

  // Subscribe to the collision_check topic
  collision_velocity_scale_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      "~/collision_velocity_scale", ROS_QUEUE_SIZE, std::bind(&ServoCalcs::collisionVelocityScaleCB, this, _1));

  // Publish to collision_check for worst stop time
  worst_case_stop_time_pub_ = node_->create_publisher<std_msgs::msg::Float64>("~/worst_case_stop_time", ROS_QUEUE_SIZE);

  // Publish freshly-calculated joints to the robot.
  // Put the outgoing msg in the right format (trajectory_msgs/JointTrajectory or std_msgs/Float64MultiArray).
  if (parameters_->command_out_type == "trajectory_msgs/JointTrajectory")
  {
    trajectory_outgoing_cmd_pub_ =
        node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(parameters_->command_out_topic, ROS_QUEUE_SIZE);
  }
  else if (parameters_->command_out_type == "std_msgs/Float64MultiArray")
  {
    multiarray_outgoing_cmd_pub_ =
        node_->create_publisher<std_msgs::msg::Float64MultiArray>(parameters_->command_out_topic, ROS_QUEUE_SIZE);
  }

  // Publish status
  status_pub_ = node_->create_publisher<std_msgs::msg::Int8>(parameters_->status_topic, ROS_QUEUE_SIZE);

  internal_joint_state_.name = joint_model_group_->getActiveJointModelNames();
  num_joints_ = internal_joint_state_.name.size();
  internal_joint_state_.position.resize(num_joints_);
  internal_joint_state_.velocity.resize(num_joints_);

  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    // A map for the indices of incoming joint commands
    joint_state_name_map_[internal_joint_state_.name[i]] = i;

    // Low-pass filters for the joint positions
    position_filters_.emplace_back(parameters_->low_pass_filter_coeff);
  }

  // A matrix of all zeros is used to check whether matrices have been initialized
  Eigen::Matrix3d empty_matrix;
  empty_matrix.setZero();
  tf_moveit_to_ee_frame_ = empty_matrix;
  tf_moveit_to_robot_cmd_frame_ = empty_matrix;
}

ServoCalcs::~ServoCalcs()
{
  stop();
}

void ServoCalcs::start()
{
  // Stop the thread if we are currently running
  stop();

  // Set up the "last" published message, in case we need to send it first
  auto initial_joint_trajectory = std::make_unique<trajectory_msgs::msg::JointTrajectory>();
  initial_joint_trajectory->header.stamp = node_->now();
  initial_joint_trajectory->header.frame_id = parameters_->planning_frame;
  initial_joint_trajectory->joint_names = internal_joint_state_.name;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(parameters_->publish_period);
  if (parameters_->publish_joint_positions)
    planning_scene_monitor_->getStateMonitor()->getCurrentState()->copyJointGroupPositions(joint_model_group_,
                                                                                           point.positions);
  if (parameters_->publish_joint_velocities)
  {
    std::vector<double> velocity(num_joints_);
    point.velocities = velocity;
  }
  if (parameters_->publish_joint_accelerations)
  {
    // I do not know of a robot that takes acceleration commands.
    // However, some controllers check that this data is non-empty.
    // Send all zeros, for now.
    point.accelerations.resize(num_joints_);
  }
  initial_joint_trajectory->points.push_back(point);
  last_sent_command_ = std::move(initial_joint_trajectory);

  current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  tf_moveit_to_ee_frame_ = current_state_->getGlobalLinkTransform(parameters_->planning_frame).inverse() *
                           current_state_->getGlobalLinkTransform(parameters_->ee_frame_name);
  tf_moveit_to_robot_cmd_frame_ = current_state_->getGlobalLinkTransform(parameters_->planning_frame).inverse() *
                                  current_state_->getGlobalLinkTransform(robot_link_command_frame_);

  stop_requested_ = false;
  thread_ = std::thread([this] { mainCalcLoop(); });
  new_input_cmd_ = false;
}

void ServoCalcs::stop()
{
  // Request stop
  stop_requested_ = true;

  // Notify condition variable in case the thread is blocked on it
  {
    // scope so the mutex is unlocked after so the thread can continue
    // and therefore be joinable
    const std::lock_guard<std::mutex> lock(main_loop_mutex_);
    new_input_cmd_ = false;
    input_cv_.notify_all();
  }

  // Join the thread
  if (thread_.joinable())
  {
    thread_.join();
  }
}

void ServoCalcs::mainCalcLoop()
{
  rclcpp::Rate rate(1.0 / parameters_->publish_period);

  while (rclcpp::ok() && !stop_requested_)
  {
    // lock the input state mutex
    std::unique_lock<std::mutex> main_loop_lock(main_loop_mutex_);

    // low latency mode -- begin calculations as soon as a new command is received.
    if (parameters_->low_latency_mode)
    {
      input_cv_.wait(main_loop_lock, [this] { return (new_input_cmd_ || stop_requested_); });
    }

    // reset new_input_cmd_ flag
    new_input_cmd_ = false;

    // run servo calcs
    const auto start_time = node_->now();
    calculateSingleIteration();
    const auto run_duration = node_->now() - start_time;

    // Log warning when the run duration was longer than the period
    if (run_duration.seconds() > parameters_->publish_period)
    {
      rclcpp::Clock& clock = *node_->get_clock();
      RCLCPP_WARN_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD,
                                  "run_duration: " << run_duration.seconds() << " (" << parameters_->publish_period
                                                   << ")");
    }

    // normal mode, unlock input mutex and wait for the period of the loop
    if (!parameters_->low_latency_mode)
    {
      main_loop_lock.unlock();
      rate.sleep();
    }
  }
}

void ServoCalcs::calculateSingleIteration()
{
  // Publish status each loop iteration
  auto status_msg = std::make_unique<std_msgs::msg::Int8>();
  status_msg->data = static_cast<int8_t>(status_);
  status_pub_->publish(std::move(status_msg));

  // After we publish, status, reset it back to no warnings
  status_ = StatusCode::NO_WARNING;

  // Always update the joints and end-effector transform for 2 reasons:
  // 1) in case the getCommandFrameTransform() method is being used
  // 2) so the low-pass filters are up to date and don't cause a jump
  updateJoints();

  // Calculate and publish worst stop time for collision checker
  if (parameters_->check_collisions && parameters_->collision_check_type == "stop_distance")
    calculateWorstCaseStopTime();

  // Update from latest state
  current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();

  if (latest_twist_stamped_)
    twist_stamped_cmd_ = *latest_twist_stamped_;
  if (latest_joint_cmd_)
    joint_servo_cmd_ = *latest_joint_cmd_;

  // Check for stale cmds
  twist_command_is_stale_ = ((node_->now() - latest_twist_command_stamp_) >=
                             rclcpp::Duration::from_seconds(parameters_->incoming_command_timeout));
  joint_command_is_stale_ = ((node_->now() - latest_joint_command_stamp_) >=
                             rclcpp::Duration::from_seconds(parameters_->incoming_command_timeout));

  have_nonzero_twist_stamped_ = latest_nonzero_twist_stamped_;
  have_nonzero_joint_command_ = latest_nonzero_joint_cmd_;

  // Get the transform from MoveIt planning frame to servoing command frame
  // Calculate this transform to ensure it is available via C++ API
  // We solve (planning_frame -> base -> robot_link_command_frame)
  // by computing (base->planning_frame)^-1 * (base->robot_link_command_frame)
  tf_moveit_to_robot_cmd_frame_ = current_state_->getGlobalLinkTransform(parameters_->planning_frame).inverse() *
                                  current_state_->getGlobalLinkTransform(robot_link_command_frame_);

  // Calculate the transform from MoveIt planning frame to End Effector frame
  // Calculate this transform to ensure it is available via C++ API
  tf_moveit_to_ee_frame_ = current_state_->getGlobalLinkTransform(parameters_->planning_frame).inverse() *
                           current_state_->getGlobalLinkTransform(parameters_->ee_frame_name);

  have_nonzero_command_ = have_nonzero_twist_stamped_ || have_nonzero_joint_command_;

  // Don't end this function without updating the filters
  updated_filters_ = false;

  // If paused or while waiting for initial servo commands, just keep the low-pass filters up to date with current
  // joints so a jump doesn't occur when restarting
  if (wait_for_servo_commands_ || paused_)
  {
    resetLowPassFilters(original_joint_state_);

    // Check if there are any new commands with valid timestamp
    wait_for_servo_commands_ =
        twist_stamped_cmd_.header.stamp == rclcpp::Time(0.) && joint_servo_cmd_.header.stamp == rclcpp::Time(0.);

    // Early exit
    return;
  }

  // If not waiting for initial command, and not paused.
  // Do servoing calculations only if the robot should move, for efficiency
  // Create new outgoing joint trajectory command message
  auto joint_trajectory = std::make_unique<trajectory_msgs::msg::JointTrajectory>();

  // Prioritize cartesian servoing above joint servoing
  // Only run commands if not stale and nonzero
  if (have_nonzero_twist_stamped_ && !twist_command_is_stale_)
  {
    if (!cartesianServoCalcs(twist_stamped_cmd_, *joint_trajectory))
    {
      resetLowPassFilters(original_joint_state_);
      return;
    }
  }
  else if (have_nonzero_joint_command_ && !joint_command_is_stale_)
  {
    if (!jointServoCalcs(joint_servo_cmd_, *joint_trajectory))
    {
      resetLowPassFilters(original_joint_state_);
      return;
    }
  }
  else
  {
    // Joint trajectory is not populated with anything, so set it to the last positions and 0 velocity
    *joint_trajectory = *last_sent_command_;
    for (auto& point : joint_trajectory->points)
    {
      point.velocities.assign(point.velocities.size(), 0);
    }
  }

  // Print a warning to the user if both are stale
  if (twist_command_is_stale_ && joint_command_is_stale_)
  {
    rclcpp::Clock& clock = *node_->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD,
                                "Stale command. Try a larger 'incoming_command_timeout' parameter?");
  }

  // If we should halt
  if (!have_nonzero_command_)
  {
    suddenHalt(*joint_trajectory);
    have_nonzero_twist_stamped_ = false;
    have_nonzero_joint_command_ = false;
  }

  // Skip the servoing publication if all inputs have been zero for several cycles in a row.
  // num_outgoing_halt_msgs_to_publish == 0 signifies that we should keep republishing forever.
  if (!have_nonzero_command_ && (parameters_->num_outgoing_halt_msgs_to_publish != 0) &&
      (zero_velocity_count_ > parameters_->num_outgoing_halt_msgs_to_publish))
  {
    ok_to_publish_ = false;
    rclcpp::Clock& clock = *node_->get_clock();
    RCLCPP_DEBUG_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD, "All-zero command. Doing nothing.");
  }
  else
  {
    ok_to_publish_ = true;
  }

  // Store last zero-velocity message flag to prevent superfluous warnings.
  // Cartesian and joint commands must both be zero.
  if (!have_nonzero_command_)
  {
    // Avoid overflow
    if (zero_velocity_count_ < std::numeric_limits<int>::max())
      ++zero_velocity_count_;
  }
  else
  {
    zero_velocity_count_ = 0;
  }

  if (ok_to_publish_ && !paused_)
  {
    // Put the outgoing msg in the right format
    // (trajectory_msgs/JointTrajectory or std_msgs/Float64MultiArray).
    if (parameters_->command_out_type == "trajectory_msgs/JointTrajectory")
    {
      // When a joint_trajectory_controller receives a new command, a stamp of 0 indicates "begin immediately"
      // See http://wiki.ros.org/joint_trajectory_controller#Trajectory_replacement
      joint_trajectory->header.stamp = rclcpp::Time(0);
      *last_sent_command_ = *joint_trajectory;
      trajectory_outgoing_cmd_pub_->publish(std::move(joint_trajectory));
    }
    else if (parameters_->command_out_type == "std_msgs/Float64MultiArray")
    {
      auto joints = std::make_unique<std_msgs::msg::Float64MultiArray>();
      if (parameters_->publish_joint_positions && !joint_trajectory->points.empty())
        joints->data = joint_trajectory->points[0].positions;
      else if (parameters_->publish_joint_velocities && !joint_trajectory->points.empty())
        joints->data = joint_trajectory->points[0].velocities;
      *last_sent_command_ = *joint_trajectory;
      multiarray_outgoing_cmd_pub_->publish(std::move(joints));
    }
  }

  // Update the filters if we haven't yet
  if (!updated_filters_)
    resetLowPassFilters(original_joint_state_);
}

rcl_interfaces::msg::SetParametersResult ServoCalcs::robotLinkCommandFrameCallback(const rclcpp::Parameter& parameter)
{
  const std::lock_guard<std::mutex> lock(main_loop_mutex_);
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  robot_link_command_frame_ = parameter.as_string();
  RCLCPP_INFO(LOGGER, "robot_link_command_frame changed to: " + robot_link_command_frame_);
  return result;
};

// Perform the servoing calculations
bool ServoCalcs::cartesianServoCalcs(geometry_msgs::msg::TwistStamped& cmd,
                                     trajectory_msgs::msg::JointTrajectory& joint_trajectory)
{
  // Check for nan's in the incoming command
  if (!checkValidCommand(cmd))
    return false;

  // Set uncontrolled dimensions to 0 in command frame
  enforceControlDimensions(cmd);

  // Transform the command to the MoveGroup planning frame
  if (cmd.header.frame_id != parameters_->planning_frame)
  {
    Eigen::Vector3d translation_vector(cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.linear.z);
    Eigen::Vector3d angular_vector(cmd.twist.angular.x, cmd.twist.angular.y, cmd.twist.angular.z);

    // If the incoming frame is empty or is the command frame, we use the previously calculated tf
    if (cmd.header.frame_id.empty() || cmd.header.frame_id == robot_link_command_frame_)
    {
      translation_vector = tf_moveit_to_robot_cmd_frame_.linear() * translation_vector;
      angular_vector = tf_moveit_to_robot_cmd_frame_.linear() * angular_vector;
    }
    else if (cmd.header.frame_id == parameters_->ee_frame_name)
    {
      // If the frame is the EE frame, we already have that transform as well
      translation_vector = tf_moveit_to_ee_frame_.linear() * translation_vector;
      angular_vector = tf_moveit_to_ee_frame_.linear() * angular_vector;
    }
    else
    {
      // We solve (planning_frame -> base -> cmd.header.frame_id)
      // by computing (base->planning_frame)^-1 * (base->cmd.header.frame_id)
      const auto tf_moveit_to_incoming_cmd_frame =
          current_state_->getGlobalLinkTransform(parameters_->planning_frame).inverse() *
          current_state_->getGlobalLinkTransform(cmd.header.frame_id);

      translation_vector = tf_moveit_to_incoming_cmd_frame.linear() * translation_vector;
      angular_vector = tf_moveit_to_incoming_cmd_frame.linear() * angular_vector;
    }

    // Put these components back into a TwistStamped
    cmd.header.frame_id = parameters_->planning_frame;
    cmd.twist.linear.x = translation_vector(0);
    cmd.twist.linear.y = translation_vector(1);
    cmd.twist.linear.z = translation_vector(2);
    cmd.twist.angular.x = angular_vector(0);
    cmd.twist.angular.y = angular_vector(1);
    cmd.twist.angular.z = angular_vector(2);
  }

  Eigen::VectorXd delta_x = scaleCartesianCommand(cmd);

  // Convert from cartesian commands to joint commands
  Eigen::MatrixXd jacobian = current_state_->getJacobian(joint_model_group_);

  removeDriftDimensions(jacobian, delta_x);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd =
      Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd matrix_s = svd.singularValues().asDiagonal();
  Eigen::MatrixXd pseudo_inverse = svd.matrixV() * matrix_s.inverse() * svd.matrixU().transpose();

  delta_theta_ = pseudo_inverse * delta_x;
  delta_theta_ *= velocityScalingFactorForSingularity(delta_x, svd, pseudo_inverse);

  return internalServoUpdate(delta_theta_, joint_trajectory);
}

bool ServoCalcs::jointServoCalcs(const control_msgs::msg::JointJog& cmd,
                                 trajectory_msgs::msg::JointTrajectory& joint_trajectory)
{
  // Check for nan's
  if (!checkValidCommand(cmd))
    return false;

  // Apply user-defined scaling
  delta_theta_ = scaleJointCommand(cmd);

  // Perform interal servo with the command
  return internalServoUpdate(delta_theta_, joint_trajectory);
}

bool ServoCalcs::internalServoUpdate(Eigen::ArrayXd& delta_theta,
                                     trajectory_msgs::msg::JointTrajectory& joint_trajectory)
{
  // Set internal joint state from original
  internal_joint_state_ = original_joint_state_;

  // Enforce SRDF Velocity, Acceleration limits
  enforceVelLimits(delta_theta);

  // Apply collision scaling
  double collision_scale = collision_velocity_scale_;
  if (collision_scale > 0 && collision_scale < 1)
  {
    status_ = StatusCode::DECELERATE_FOR_COLLISION;
    rclcpp::Clock& clock = *node_->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD, SERVO_STATUS_CODE_MAP.at(status_));
  }
  else if (collision_scale == 0)
  {
    status_ = StatusCode::HALT_FOR_COLLISION;
    rclcpp::Clock& clock = *node_->get_clock();
    RCLCPP_ERROR_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD, "Halting for collision!");
  }
  delta_theta *= collision_scale;

  // Loop thru joints and update them, calculate velocities, and filter
  if (!applyJointUpdate(delta_theta, internal_joint_state_, prev_joint_velocity_))
    return false;

  // Mark the lowpass filters as updated for this cycle
  updated_filters_ = true;

  // compose outgoing message
  composeJointTrajMessage(internal_joint_state_, joint_trajectory);

  // Enforce SRDF position limits, might halt if needed, set prev_vel to 0
  if (!enforcePositionLimits())
  {
    suddenHalt(joint_trajectory);
    status_ = StatusCode::JOINT_BOUND;
    prev_joint_velocity_.setZero();
  }

  // Modify the output message if we are using gazebo
  if (parameters_->use_gazebo)
  {
    insertRedundantPointsIntoTrajectory(joint_trajectory, gazebo_redundant_message_count_);
  }

  return true;
}

bool ServoCalcs::applyJointUpdate(const Eigen::ArrayXd& delta_theta, sensor_msgs::msg::JointState& joint_state,
                                  Eigen::ArrayXd& previous_vel)
{
  // All the sizes must match
  if (joint_state.position.size() != static_cast<std::size_t>(delta_theta.size()) ||
      joint_state.velocity.size() != joint_state.position.size() ||
      static_cast<std::size_t>(previous_vel.size()) != joint_state.position.size())
  {
    rclcpp::Clock& clock = *node_->get_clock();
    RCLCPP_ERROR_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD,
                                 "Lengths of output and increments do not match.");
    return false;
  }

  for (std::size_t i = 0; i < joint_state.position.size(); ++i)
  {
    // Increment joint
    joint_state.position[i] += delta_theta[i];

    // Lowpass filter position
    joint_state.position[i] = position_filters_[i].filter(joint_state.position[i]);

    // Calculate joint velocity
    joint_state.velocity[i] = delta_theta[i] / parameters_->publish_period;

    // Save this velocity for future accel calculations
    previous_vel[i] = joint_state.velocity[i];
  }
  return true;
}

// Spam several redundant points into the trajectory. The first few may be skipped if the
// time stamp is in the past when it reaches the client. Needed for gazebo simulation.
// Start from 1 because the first point's timestamp is already 1*parameters_->publish_period
void ServoCalcs::insertRedundantPointsIntoTrajectory(trajectory_msgs::msg::JointTrajectory& joint_trajectory,
                                                     int count) const
{
  if (count < 2)
    return;
  joint_trajectory.points.resize(count);
  auto point = joint_trajectory.points[0];
  // Start from 1 because we already have the first point. End at count+1 so (total #) == count
  for (int i = 1; i < count; ++i)
  {
    point.time_from_start = rclcpp::Duration::from_seconds((i + 1) * parameters_->publish_period);
    joint_trajectory.points[i] = point;
  }
}

void ServoCalcs::resetLowPassFilters(const sensor_msgs::msg::JointState& joint_state)
{
  for (std::size_t i = 0; i < position_filters_.size(); ++i)
  {
    position_filters_[i].reset(joint_state.position[i]);
  }

  updated_filters_ = true;
}

void ServoCalcs::composeJointTrajMessage(const sensor_msgs::msg::JointState& joint_state,
                                         trajectory_msgs::msg::JointTrajectory& joint_trajectory)
{
  // When a joint_trajectory_controller receives a new command, a stamp of 0 indicates "begin immediately"
  // See http://wiki.ros.org/joint_trajectory_controller#Trajectory_replacement
  joint_trajectory.header.stamp = rclcpp::Time(0);
  joint_trajectory.header.frame_id = parameters_->planning_frame;
  joint_trajectory.joint_names = joint_state.name;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(parameters_->publish_period);
  if (parameters_->publish_joint_positions)
    point.positions = joint_state.position;
  if (parameters_->publish_joint_velocities)
    point.velocities = joint_state.velocity;
  if (parameters_->publish_joint_accelerations)
  {
    // I do not know of a robot that takes acceleration commands.
    // However, some controllers check that this data is non-empty.
    // Send all zeros, for now.
    std::vector<double> acceleration(num_joints_);
    point.accelerations = acceleration;
  }
  joint_trajectory.points.push_back(point);
}

// Possibly calculate a velocity scaling factor, due to proximity of singularity and direction of motion
double ServoCalcs::velocityScalingFactorForSingularity(const Eigen::VectorXd& commanded_velocity,
                                                       const Eigen::JacobiSVD<Eigen::MatrixXd>& svd,
                                                       const Eigen::MatrixXd& pseudo_inverse)
{
  double velocity_scale = 1;
  std::size_t num_dimensions = commanded_velocity.size();

  // Find the direction away from nearest singularity.
  // The last column of U from the SVD of the Jacobian points directly toward or away from the singularity.
  // The sign can flip at any time, so we have to do some extra checking.
  // Look ahead to see if the Jacobian's condition will decrease.
  Eigen::VectorXd vector_toward_singularity = svd.matrixU().col(num_dimensions - 1);

  double ini_condition = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);

  // This singular vector tends to flip direction unpredictably. See R. Bro,
  // "Resolving the Sign Ambiguity in the Singular Value Decomposition".
  // Look ahead to see if the Jacobian's condition will decrease in this
  // direction. Start with a scaled version of the singular vector
  Eigen::VectorXd delta_x(num_dimensions);
  double scale = 100;
  delta_x = vector_toward_singularity / scale;

  // Calculate a small change in joints
  Eigen::VectorXd new_theta;
  current_state_->copyJointGroupPositions(joint_model_group_, new_theta);
  new_theta += pseudo_inverse * delta_x;
  current_state_->setJointGroupPositions(joint_model_group_, new_theta);
  Eigen::MatrixXd new_jacobian = current_state_->getJacobian(joint_model_group_);

  Eigen::JacobiSVD<Eigen::MatrixXd> new_svd(new_jacobian);
  double new_condition = new_svd.singularValues()(0) / new_svd.singularValues()(new_svd.singularValues().size() - 1);
  // If new_condition < ini_condition, the singular vector does point towards a
  // singularity. Otherwise, flip its direction.
  if (ini_condition >= new_condition)
  {
    vector_toward_singularity *= -1;
  }

  // If this dot product is positive, we're moving toward singularity ==> decelerate
  double dot = vector_toward_singularity.dot(commanded_velocity);
  if (dot > 0)
  {
    // Ramp velocity down linearly when the Jacobian condition is between lower_singularity_threshold and
    // hard_stop_singularity_threshold, and we're moving towards the singularity
    if ((ini_condition > parameters_->lower_singularity_threshold) &&
        (ini_condition < parameters_->hard_stop_singularity_threshold))
    {
      velocity_scale =
          1. - (ini_condition - parameters_->lower_singularity_threshold) /
                   (parameters_->hard_stop_singularity_threshold - parameters_->lower_singularity_threshold);
      status_ = StatusCode::DECELERATE_FOR_SINGULARITY;
      rclcpp::Clock& clock = *node_->get_clock();
      RCLCPP_WARN_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD, SERVO_STATUS_CODE_MAP.at(status_));
    }

    // Very close to singularity, so halt.
    else if (ini_condition > parameters_->hard_stop_singularity_threshold)
    {
      velocity_scale = 0;
      status_ = StatusCode::HALT_FOR_SINGULARITY;
      rclcpp::Clock& clock = *node_->get_clock();
      RCLCPP_WARN_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD, SERVO_STATUS_CODE_MAP.at(status_));
    }
  }

  return velocity_scale;
}

void ServoCalcs::enforceVelLimits(Eigen::ArrayXd& delta_theta)
{
  Eigen::ArrayXd velocity = delta_theta / parameters_->publish_period;

  std::size_t joint_delta_index = 0;
  // Track the smallest velocity scaling factor required, across all joints
  double velocity_limit_scaling_factor = 1;

  for (auto joint : joint_model_group_->getActiveJointModels())
  {
    // Some joints do not have bounds defined
    const auto bound = joint->getVariableBounds(joint->getName());

    if (bound.velocity_bounded_)
    {
      velocity(joint_delta_index) = delta_theta(joint_delta_index) / parameters_->publish_period;

      bool clip_velocity = false;
      double velocity_limit = 0.0;
      if (velocity(joint_delta_index) < bound.min_velocity_)
      {
        clip_velocity = true;
        velocity_limit = bound.min_velocity_;
      }
      else if (velocity(joint_delta_index) > bound.max_velocity_)
      {
        clip_velocity = true;
        velocity_limit = bound.max_velocity_;
      }

      // Apply velocity bounds
      if (clip_velocity)
      {
        const double scaling_factor =
            fabs(velocity_limit * parameters_->publish_period) / fabs(delta_theta(joint_delta_index));

        // Store the scaling factor if it's the smallest yet
        if (scaling_factor < velocity_limit_scaling_factor)
          velocity_limit_scaling_factor = scaling_factor;
      }
    }
    ++joint_delta_index;
  }

  // Apply the velocity scaling to all joints
  if (velocity_limit_scaling_factor < 1)
  {
    for (joint_delta_index = 0; joint_delta_index < joint_model_group_->getActiveJointModels().size();
         ++joint_delta_index)
    {
      delta_theta(joint_delta_index) = velocity_limit_scaling_factor * delta_theta(joint_delta_index);
      velocity(joint_delta_index) = velocity_limit_scaling_factor * velocity(joint_delta_index);
    }
  }
}

bool ServoCalcs::enforcePositionLimits()
{
  bool halting = false;

  for (auto joint : joint_model_group_->getActiveJointModels())
  {
    // Halt if we're past a joint margin and joint velocity is moving even farther past
    double joint_angle = 0;
    for (std::size_t c = 0; c < original_joint_state_.name.size(); ++c)
    {
      if (original_joint_state_.name[c] == joint->getName())
      {
        joint_angle = original_joint_state_.position.at(c);
        break;
      }
    }
    if (!current_state_->satisfiesPositionBounds(joint, -parameters_->joint_limit_margin))
    {
      const std::vector<moveit_msgs::msg::JointLimits> limits = joint->getVariableBoundsMsg();

      // Joint limits are not defined for some joints. Skip them.
      if (!limits.empty())
      {
        if ((current_state_->getJointVelocities(joint)[0] < 0 &&
             (joint_angle < (limits[0].min_position + parameters_->joint_limit_margin))) ||
            (current_state_->getJointVelocities(joint)[0] > 0 &&
             (joint_angle > (limits[0].max_position - parameters_->joint_limit_margin))))
        {
          rclcpp::Clock& clock = *node_->get_clock();
          RCLCPP_WARN_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD,
                                      node_->get_name()
                                          << " " << joint->getName() << " close to a position limit. Halting.");
          halting = true;
        }
      }
    }
  }
  return !halting;
}

// Suddenly halt for a joint limit or other critical issue.
// Is handled differently for position vs. velocity control.
void ServoCalcs::suddenHalt(trajectory_msgs::msg::JointTrajectory& joint_trajectory) const
{
  // Prepare the joint trajectory message to stop the robot
  joint_trajectory.points.clear();
  joint_trajectory.points.emplace_back();
  trajectory_msgs::msg::JointTrajectoryPoint& point = joint_trajectory.points.front();

  // When sending out trajectory_msgs/JointTrajectory type messages, the "trajectory" is just a single point.
  // That point cannot have the same timestamp as the start of trajectory execution since that would mean the
  // arm has to reach the first trajectory point the moment execution begins. To prevent errors about points
  // being 0 seconds in the past, the smallest supported timestep is added as time from start to the trajectory point.
  point.time_from_start = rclcpp::Duration(0, 1);

  if (parameters_->publish_joint_positions)
    point.positions.resize(num_joints_);
  if (parameters_->publish_joint_velocities)
    point.velocities.resize(num_joints_);

  // Assert the following loop is safe to execute
  assert(original_joint_state_.position.size() >= num_joints_);

  // Set the positions and velocities vectors
  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    // For position-controlled robots, can reset the joints to a known, good state
    if (parameters_->publish_joint_positions)
      point.positions[i] = original_joint_state_.position[i];

    // For velocity-controlled robots, stop
    if (parameters_->publish_joint_velocities)
      point.velocities[i] = 0;
  }
}

// Parse the incoming joint msg for the joints of our MoveGroup
void ServoCalcs::updateJoints()
{
  // Get the latest joint group positions
  current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  current_state_->copyJointGroupPositions(joint_model_group_, internal_joint_state_.position);
  current_state_->copyJointGroupVelocities(joint_model_group_, internal_joint_state_.velocity);

  // Cache the original joints in case they need to be reset
  original_joint_state_ = internal_joint_state_;
}

// Calculate worst case joint stop time, for collision checking
void ServoCalcs::calculateWorstCaseStopTime()
{
  std::string joint_name = "";
  moveit::core::JointModel::Bounds kinematic_bounds;
  double accel_limit = 0;
  double joint_velocity = 0;
  double worst_case_stop_time = 0;
  for (size_t jt_state_idx = 0; jt_state_idx < internal_joint_state_.velocity.size(); ++jt_state_idx)
  {
    joint_name = internal_joint_state_.name[jt_state_idx];

    // Get acceleration limit for this joint
    for (auto joint_model : joint_model_group_->getActiveJointModels())
    {
      if (joint_model->getName() == joint_name)
      {
        kinematic_bounds = joint_model->getVariableBounds();
        // Some joints do not have acceleration limits
        if (kinematic_bounds[0].acceleration_bounded_)
        {
          // Be conservative when calculating overall acceleration limit from min and max limits
          accel_limit =
              std::min(fabs(kinematic_bounds[0].min_acceleration_), fabs(kinematic_bounds[0].max_acceleration_));
        }
        else
        {
          rclcpp::Clock& clock = *node_->get_clock();
          RCLCPP_ERROR_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD,
                                       "An acceleration limit is not defined for this joint; minimum stop distance "
                                       "should not be used for collision checking");

          // TODO(adamp): figure out what to do here. We definitely don't want to allow 'stop_distance' collision
          // checking with no acceleration limits defined.
        }
        break;
      }
    }

    // Get the current joint velocity
    joint_velocity = internal_joint_state_.velocity[jt_state_idx];

    // Calculate worst case stop time
    worst_case_stop_time = std::max(worst_case_stop_time, fabs(joint_velocity / accel_limit));
  }

  // publish message
  {
    auto msg = std::make_unique<std_msgs::msg::Float64>();
    msg->data = worst_case_stop_time;
    worst_case_stop_time_pub_->publish(std::move(msg));
  }
}

bool ServoCalcs::checkValidCommand(const control_msgs::msg::JointJog& cmd)
{
  for (double velocity : cmd.velocities)
  {
    if (std::isnan(velocity))
    {
      rclcpp::Clock& clock = *node_->get_clock();
      RCLCPP_WARN_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD,
                                  "nan in incoming command. Skipping this datapoint.");
      return false;
    }
  }
  return true;
}

bool ServoCalcs::checkValidCommand(const geometry_msgs::msg::TwistStamped& cmd)
{
  if (std::isnan(cmd.twist.linear.x) || std::isnan(cmd.twist.linear.y) || std::isnan(cmd.twist.linear.z) ||
      std::isnan(cmd.twist.angular.x) || std::isnan(cmd.twist.angular.y) || std::isnan(cmd.twist.angular.z))
  {
    rclcpp::Clock& clock = *node_->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD,
                                "nan in incoming command. Skipping this datapoint.");
    return false;
  }

  // If incoming commands should be in the range [-1:1], check for |delta|>1
  if (parameters_->command_in_type == "unitless")
  {
    if ((fabs(cmd.twist.linear.x) > 1) || (fabs(cmd.twist.linear.y) > 1) || (fabs(cmd.twist.linear.z) > 1) ||
        (fabs(cmd.twist.angular.x) > 1) || (fabs(cmd.twist.angular.y) > 1) || (fabs(cmd.twist.angular.z) > 1))
    {
      rclcpp::Clock& clock = *node_->get_clock();
      RCLCPP_WARN_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD,
                                  "Component of incoming command is >1. Skipping this datapoint.");
      return false;
    }
  }

  return true;
}

// Scale the incoming jog command
Eigen::VectorXd ServoCalcs::scaleCartesianCommand(const geometry_msgs::msg::TwistStamped& command)
{
  Eigen::VectorXd result(6);
  result.setZero();  // Or the else case below leads to misery

  // Apply user-defined scaling if inputs are unitless [-1:1]
  if (parameters_->command_in_type == "unitless")
  {
    result[0] = parameters_->linear_scale * parameters_->publish_period * command.twist.linear.x;
    result[1] = parameters_->linear_scale * parameters_->publish_period * command.twist.linear.y;
    result[2] = parameters_->linear_scale * parameters_->publish_period * command.twist.linear.z;
    result[3] = parameters_->rotational_scale * parameters_->publish_period * command.twist.angular.x;
    result[4] = parameters_->rotational_scale * parameters_->publish_period * command.twist.angular.y;
    result[5] = parameters_->rotational_scale * parameters_->publish_period * command.twist.angular.z;
  }
  // Otherwise, commands are in m/s and rad/s
  else if (parameters_->command_in_type == "speed_units")
  {
    result[0] = command.twist.linear.x * parameters_->publish_period;
    result[1] = command.twist.linear.y * parameters_->publish_period;
    result[2] = command.twist.linear.z * parameters_->publish_period;
    result[3] = command.twist.angular.x * parameters_->publish_period;
    result[4] = command.twist.angular.y * parameters_->publish_period;
    result[5] = command.twist.angular.z * parameters_->publish_period;
  }
  else
  {
    rclcpp::Clock& clock = *node_->get_clock();
    RCLCPP_ERROR_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD, "Unexpected command_in_type");
  }

  return result;
}

Eigen::VectorXd ServoCalcs::scaleJointCommand(const control_msgs::msg::JointJog& command)
{
  Eigen::VectorXd result(num_joints_);
  result.setZero();

  std::size_t c;
  for (std::size_t m = 0; m < command.joint_names.size(); ++m)
  {
    try
    {
      c = joint_state_name_map_.at(command.joint_names[m]);
    }
    catch (const std::out_of_range& e)
    {
      rclcpp::Clock& clock = *node_->get_clock();
      RCLCPP_WARN_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD, "Ignoring joint " << command.joint_names[m]);
      continue;
    }
    // Apply user-defined scaling if inputs are unitless [-1:1]
    if (parameters_->command_in_type == "unitless")
      result[c] = command.velocities[m] * parameters_->joint_scale * parameters_->publish_period;
    // Otherwise, commands are in m/s and rad/s
    else if (parameters_->command_in_type == "speed_units")
      result[c] = command.velocities[m] * parameters_->publish_period;
    else
    {
      rclcpp::Clock& clock = *node_->get_clock();
      RCLCPP_ERROR_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD,
                                   "Unexpected command_in_type, check yaml file.");
    }
  }

  return result;
}

void ServoCalcs::removeDimension(Eigen::MatrixXd& jacobian, Eigen::VectorXd& delta_x, unsigned int row_to_remove) const
{
  unsigned int num_rows = jacobian.rows() - 1;
  unsigned int num_cols = jacobian.cols();

  if (row_to_remove < num_rows)
  {
    jacobian.block(row_to_remove, 0, num_rows - row_to_remove, num_cols) =
        jacobian.block(row_to_remove + 1, 0, num_rows - row_to_remove, num_cols);
    delta_x.segment(row_to_remove, num_rows - row_to_remove) =
        delta_x.segment(row_to_remove + 1, num_rows - row_to_remove);
  }
  jacobian.conservativeResize(num_rows, num_cols);
  delta_x.conservativeResize(num_rows);
}

void ServoCalcs::removeDriftDimensions(Eigen::MatrixXd& matrix, Eigen::VectorXd& delta_x)
{
  // May allow some dimensions to drift, based on drift_dimensions
  // i.e. take advantage of task redundancy.
  // Remove the Jacobian rows corresponding to True in the vector drift_dimensions
  // Work backwards through the 6-vector so indices don't get out of order
  for (auto dimension = matrix.rows() - 1; dimension >= 0; --dimension)
  {
    if (drift_dimensions_[dimension] && matrix.rows() > 1)
    {
      removeDimension(matrix, delta_x, dimension);
    }
  }
}

void ServoCalcs::enforceControlDimensions(geometry_msgs::msg::TwistStamped& command)
{
  // Can't loop through the message, so check them all
  if (!control_dimensions_[0])
    command.twist.linear.x = 0;
  if (!control_dimensions_[1])
    command.twist.linear.y = 0;
  if (!control_dimensions_[2])
    command.twist.linear.z = 0;
  if (!control_dimensions_[3])
    command.twist.angular.x = 0;
  if (!control_dimensions_[4])
    command.twist.angular.y = 0;
  if (!control_dimensions_[5])
    command.twist.angular.z = 0;
}

bool ServoCalcs::getCommandFrameTransform(Eigen::Isometry3d& transform)
{
  const std::lock_guard<std::mutex> lock(main_loop_mutex_);
  transform = tf_moveit_to_robot_cmd_frame_;

  // All zeros means the transform wasn't initialized, so return false
  return !transform.matrix().isZero(0);
}

bool ServoCalcs::getCommandFrameTransform(geometry_msgs::msg::TransformStamped& transform)
{
  const std::lock_guard<std::mutex> lock(main_loop_mutex_);
  // All zeros means the transform wasn't initialized, so return false
  if (tf_moveit_to_robot_cmd_frame_.matrix().isZero(0))
  {
    return false;
  }

  transform =
      convertIsometryToTransform(tf_moveit_to_robot_cmd_frame_, parameters_->planning_frame, robot_link_command_frame_);
  return true;
}

bool ServoCalcs::getEEFrameTransform(Eigen::Isometry3d& transform)
{
  const std::lock_guard<std::mutex> lock(main_loop_mutex_);
  transform = tf_moveit_to_ee_frame_;

  // All zeros means the transform wasn't initialized, so return false
  return !transform.matrix().isZero(0);
}

bool ServoCalcs::getEEFrameTransform(geometry_msgs::msg::TransformStamped& transform)
{
  const std::lock_guard<std::mutex> lock(main_loop_mutex_);
  // All zeros means the transform wasn't initialized, so return false
  if (tf_moveit_to_ee_frame_.matrix().isZero(0))
  {
    return false;
  }

  transform =
      convertIsometryToTransform(tf_moveit_to_ee_frame_, parameters_->planning_frame, parameters_->ee_frame_name);
  return true;
}

void ServoCalcs::twistStampedCB(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(main_loop_mutex_);
  latest_twist_stamped_ = msg;
  latest_nonzero_twist_stamped_ = isNonZero(*latest_twist_stamped_.get());

  if (msg.get()->header.stamp != rclcpp::Time(0.))
    latest_twist_command_stamp_ = msg.get()->header.stamp;

  // notify that we have a new input
  new_input_cmd_ = true;
  input_cv_.notify_all();
}

void ServoCalcs::jointCmdCB(const control_msgs::msg::JointJog::SharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(main_loop_mutex_);
  latest_joint_cmd_ = msg;
  latest_nonzero_joint_cmd_ = isNonZero(*latest_joint_cmd_.get());

  if (msg.get()->header.stamp != rclcpp::Time(0.))
    latest_joint_command_stamp_ = msg.get()->header.stamp;

  // notify that we have a new input
  new_input_cmd_ = true;
  input_cv_.notify_all();
}

void ServoCalcs::collisionVelocityScaleCB(const std_msgs::msg::Float64::SharedPtr msg)
{
  collision_velocity_scale_ = msg.get()->data;
}

void ServoCalcs::changeDriftDimensions(const std::shared_ptr<moveit_msgs::srv::ChangeDriftDimensions::Request> req,
                                       std::shared_ptr<moveit_msgs::srv::ChangeDriftDimensions::Response> res)
{
  drift_dimensions_[0] = req->drift_x_translation;
  drift_dimensions_[1] = req->drift_y_translation;
  drift_dimensions_[2] = req->drift_z_translation;
  drift_dimensions_[3] = req->drift_x_rotation;
  drift_dimensions_[4] = req->drift_y_rotation;
  drift_dimensions_[5] = req->drift_z_rotation;

  res->success = true;
}

void ServoCalcs::changeControlDimensions(const std::shared_ptr<moveit_msgs::srv::ChangeControlDimensions::Request> req,
                                         std::shared_ptr<moveit_msgs::srv::ChangeControlDimensions::Response> res)
{
  control_dimensions_[0] = req->control_x_translation;
  control_dimensions_[1] = req->control_y_translation;
  control_dimensions_[2] = req->control_z_translation;
  control_dimensions_[3] = req->control_x_rotation;
  control_dimensions_[4] = req->control_y_rotation;
  control_dimensions_[5] = req->control_z_rotation;

  res->success = true;
}

bool ServoCalcs::resetServoStatus(const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
                                  std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
{
  status_ = StatusCode::NO_WARNING;
  return true;
}

void ServoCalcs::setPaused(bool paused)
{
  paused_ = paused;
}

}  // namespace moveit_servo
