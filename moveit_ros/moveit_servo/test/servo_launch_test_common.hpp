/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
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

/*      Title     : servo_launch_test_common.hpp
 *      Project   : moveit_servo
 *      Created   : 08/03/2020
 *      Author    : Adam Pettinger
 */

// C++
#include <string>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit_msgs/srv/change_drift_dimensions.hpp>
#include <moveit_msgs/srv/change_control_dimensions.hpp>

// Testing
#include <gtest/gtest.h>

// Servo
#include <moveit_servo/servo_parameters.cpp>
#include <moveit_servo/status_codes.h>
#include "test_parameter_struct.hpp"

#pragma once

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.servo_launch_test_common.hpp");

namespace moveit_servo
{
class ServoFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { this->executor_->spin(); });
  }

  ServoFixture()
    : node_(std::make_shared<rclcpp::Node>("servo_testing"))
    , executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
    , parameters_(getTestParameters())
    , PUBLISH_HZ(2.0 / parameters_->incoming_command_timeout)
    , PUBLISH_PERIOD(1.0 / PUBLISH_HZ)
    , TIMEOUT_ITERATIONS(10 * PUBLISH_HZ)
    , publish_loop_rate_(PUBLISH_HZ)
    , servo_server_node_name_("/servo_server")
  {
    // Init ROS interfaces
    // Publishers
    pub_twist_cmd_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
        resolveServoTopicName(parameters_->cartesian_command_in_topic), 10);
    pub_joint_cmd_ = node_->create_publisher<control_msgs::msg::JointJog>(
        resolveServoTopicName(parameters_->joint_command_in_topic), 10);
  }

  void TearDown() override
  {
    // If the stop client isn't null, we set it up and likely started the Servo. Stop it.
    // Otherwise the Servo is still running when another test starts...
    if (!client_servo_stop_)
    {
      client_servo_stop_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    }
    executor_->cancel();
    if (executor_thread_.joinable())
      executor_thread_.join();
  }

  std::string resolveServoTopicName(std::string topic_name) const
  {
    if (topic_name.at(0) == '~')
      return topic_name.replace(0, 1, servo_server_node_name_);
    else
      return topic_name;
  }

  // Set up for callbacks (so they aren't run for EVERY test)
  bool setupStartClient()
  {
    // Start client
    client_servo_start_ = node_->create_client<std_srvs::srv::Trigger>(resolveServoTopicName("~/start_servo"));
    while (!client_servo_start_->service_is_ready())
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(LOGGER, "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO(LOGGER, "client_servo_start_ service not available, waiting again...");
      rclcpp::sleep_for(500ms);
    }

    // If we setup the start client, also setup the stop client...
    client_servo_stop_ = node_->create_client<std_srvs::srv::Trigger>(resolveServoTopicName("~/stop_servo"));
    while (!client_servo_stop_->service_is_ready())
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(LOGGER, "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO(LOGGER, "client_servo_stop_ service not available, waiting again...");
      rclcpp::sleep_for(500ms);
    }

    // Status sub (we need this to check that we've started / stopped)
    sub_servo_status_ = node_->create_subscription<std_msgs::msg::Int8>(
        resolveServoTopicName(parameters_->status_topic), 10,
        std::bind(&ServoFixture::statusCB, this, std::placeholders::_1));
    return true;
  }

  bool setupPauseClient()
  {
    client_servo_pause_ = node_->create_client<std_srvs::srv::Trigger>(resolveServoTopicName("~/pause_servo"));
    while (!client_servo_pause_->service_is_ready())
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(LOGGER, "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO(LOGGER, "client_servo_pause_ service not available, waiting again...");
      rclcpp::sleep_for(500ms);
    }
    return true;
  }

  bool setupUnpauseClient()
  {
    client_servo_unpause_ = node_->create_client<std_srvs::srv::Trigger>(resolveServoTopicName("~/unpause_servo"));
    while (!client_servo_unpause_->service_is_ready())
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(LOGGER, "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO(LOGGER, "client_servo_unpause_ service not available, waiting again...");
      rclcpp::sleep_for(500ms);
    }
    return true;
  }

  bool setupControlDimsClient()
  {
    client_change_control_dims_ = node_->create_client<moveit_msgs::srv::ChangeControlDimensions>(
        resolveServoTopicName("~/change_control_dimensions"));
    while (!client_change_control_dims_->service_is_ready())
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(LOGGER, "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO(LOGGER, "client_change_control_dims_ service not available, waiting again...");
      rclcpp::sleep_for(500ms);
    }
    return true;
  }

  bool setupDriftDimsClient()
  {
    client_change_drift_dims_ = node_->create_client<moveit_msgs::srv::ChangeDriftDimensions>(
        resolveServoTopicName("~/change_drift_dimensions"));
    while (!client_change_drift_dims_->service_is_ready())
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(LOGGER, "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO(LOGGER, "client_change_drift_dims_ service not available, waiting again...");
      rclcpp::sleep_for(500ms);
    }
    return true;
  }

  bool setupCollisionScaleSub()
  {
    sub_collision_scale_ = node_->create_subscription<std_msgs::msg::Float64>(
        resolveServoTopicName("~/collision_velocity_scale"), ROS_QUEUE_SIZE,
        std::bind(&ServoFixture::collisionScaleCB, this, std::placeholders::_1));
    return true;
  }

  bool setupCommandSub(std::string command_type)
  {
    if (command_type == "trajectory_msgs/JointTrajectory")
    {
      sub_trajectory_cmd_output_ = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
          resolveServoTopicName(parameters_->command_out_topic), 10,
          std::bind(&ServoFixture::trajectoryCommandCB, this, std::placeholders::_1));
      return true;
    }
    else if (command_type == "std_msgs/Float64MultiArray")
    {
      sub_array_cmd_output_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
          resolveServoTopicName(parameters_->command_out_topic), 10,
          std::bind(&ServoFixture::arrayCommandCB, this, std::placeholders::_1));
      return true;
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Invalid command type for Servo output command");
      return false;
    }
  }

  bool setupJointStateSub()
  {
    sub_joint_state_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        resolveServoTopicName(parameters_->joint_topic), 10,
        std::bind(&ServoFixture::jointStateCB, this, std::placeholders::_1));
    return true;
  }

  void statusCB(const std_msgs::msg::Int8::SharedPtr msg)
  {
    const std::lock_guard<std::mutex> lock(latest_state_mutex_);
    ++num_status_;
    latest_status_ = static_cast<StatusCode>(msg.get()->data);
    if (latest_status_ == status_tracking_code_)
      status_seen_ = true;
  }

  void collisionScaleCB(const std_msgs::msg::Float64::SharedPtr msg)
  {
    const std::lock_guard<std::mutex> lock(latest_state_mutex_);
    ++num_collision_scale_;
    latest_collision_scale_ = msg.get()->data;
  }

  void jointStateCB(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    const std::lock_guard<std::mutex> lock(latest_state_mutex_);
    ++num_joint_state_;
    latest_joint_state_ = msg;
  }

  void trajectoryCommandCB(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    const std::lock_guard<std::mutex> lock(latest_state_mutex_);
    ++num_commands_;
    latest_traj_cmd_ = msg;
  }

  void arrayCommandCB(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    const std::lock_guard<std::mutex> lock(latest_state_mutex_);
    ++num_commands_;
    latest_array_cmd_ = msg;
  }

  StatusCode getLatestStatus()
  {
    const std::lock_guard<std::mutex> lock(latest_state_mutex_);
    return latest_status_;
  }

  size_t getNumStatus()
  {
    const std::lock_guard<std::mutex> lock(latest_state_mutex_);
    return num_status_;
  }

  void resetNumStatus()
  {
    const std::lock_guard<std::mutex> lock(latest_state_mutex_);
    num_status_ = 0;
  }

  double getLatestCollisionScale()
  {
    const std::lock_guard<std::mutex> lock(latest_state_mutex_);
    return latest_collision_scale_;
  }

  size_t getNumCollisionScale()
  {
    const std::lock_guard<std::mutex> lock(latest_state_mutex_);
    return num_collision_scale_;
  }

  void resetNumCollisionScale()
  {
    const std::lock_guard<std::mutex> lock(latest_state_mutex_);
    num_collision_scale_ = 0;
  }

  sensor_msgs::msg::JointState getLatestJointState()
  {
    const std::lock_guard<std::mutex> lock(latest_state_mutex_);
    return *latest_joint_state_;
  }

  size_t getNumJointState()
  {
    const std::lock_guard<std::mutex> lock(latest_state_mutex_);
    return num_joint_state_;
  }

  void resetNumJointState()
  {
    const std::lock_guard<std::mutex> lock(latest_state_mutex_);
    num_joint_state_ = 0;
  }

  trajectory_msgs::msg::JointTrajectory getLatestTrajCommand()
  {
    const std::lock_guard<std::mutex> lock(latest_state_mutex_);
    return *latest_traj_cmd_;
  }

  std_msgs::msg::Float64MultiArray getLatestArrayCommand()
  {
    const std::lock_guard<std::mutex> lock(latest_state_mutex_);
    return *latest_array_cmd_;
  }

  size_t getNumCommands()
  {
    const std::lock_guard<std::mutex> lock(latest_state_mutex_);
    return num_commands_;
  }

  void resetNumCommands()
  {
    const std::lock_guard<std::mutex> lock(latest_state_mutex_);
    num_commands_ = 0;
  }

  void watchForStatus(StatusCode code_to_watch_for)
  {
    const std::lock_guard<std::mutex> lock(latest_state_mutex_);
    status_seen_ = false;
    status_tracking_code_ = code_to_watch_for;
  }

  bool sawTrackedStatus()
  {
    const std::lock_guard<std::mutex> lock(latest_state_mutex_);
    return status_seen_;
  }

  bool start()
  {
    auto time_start = node_->now();
    auto start_result = client_servo_start_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (!start_result.get()->success)
    {
      RCLCPP_ERROR(LOGGER, "Error returned form service call to start servo");
      return false;
    }
    RCLCPP_INFO_STREAM(LOGGER, "Wait for start servo: " << (node_->now() - time_start).seconds());

    // Test that status messages start
    time_start = node_->now();
    auto num_statuses_start = getNumStatus();
    size_t iterations = 0;
    while (getNumStatus() == num_statuses_start && iterations++ < TIMEOUT_ITERATIONS)
    {
      publish_loop_rate_.sleep();
    }

    RCLCPP_INFO_STREAM(LOGGER, "Wait for status num increasing: " << (node_->now() - time_start).seconds());

    if (iterations >= TIMEOUT_ITERATIONS)
    {
      RCLCPP_ERROR(LOGGER, "Timeout waiting for status num increasing");
      return false;
    }

    return getNumStatus() > num_statuses_start;
  }

  bool stop()
  {
    auto time_start = node_->now();
    auto stop_result = client_servo_stop_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (!stop_result.get()->success)
    {
      RCLCPP_ERROR(LOGGER, "Error returned form service call to stop servo");
      return false;
    }
    RCLCPP_INFO_STREAM(LOGGER, "Wait for stop servo service: " << (node_->now() - time_start).seconds());

    // Test that status messages stop
    time_start = node_->now();
    size_t num_statuses_start = 0;
    size_t iterations = 0;
    do
    {
      num_statuses_start = getNumStatus();
      // Wait 4x the loop rate
      for (size_t i = 0; i < 4; ++i)
        publish_loop_rate_.sleep();
    } while (getNumStatus() != num_statuses_start && ++iterations < TIMEOUT_ITERATIONS);
    RCLCPP_INFO_STREAM(LOGGER, "Wait for status to stop: " << (node_->now() - time_start).seconds());

    if (iterations >= TIMEOUT_ITERATIONS)
    {
      RCLCPP_ERROR(LOGGER, "Timeout waiting for status num increasing");
      return false;
    }

    return true;
  }

protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Executor::SharedPtr executor_;
  moveit_servo::ServoParametersPtr parameters_;
  std::thread executor_thread_;

  // ROS interfaces
  // Service Clients
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_servo_start_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_servo_stop_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_servo_pause_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_servo_unpause_;
  rclcpp::Client<moveit_msgs::srv::ChangeControlDimensions>::SharedPtr client_change_control_dims_;
  rclcpp::Client<moveit_msgs::srv::ChangeDriftDimensions>::SharedPtr client_change_drift_dims_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_cmd_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr pub_joint_cmd_;

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_servo_status_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_collision_scale_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_trajectory_cmd_output_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_array_cmd_output_;

  // Subscription counting and data
  size_t num_status_;
  StatusCode latest_status_ = StatusCode::NO_WARNING;

  size_t num_collision_scale_;
  double latest_collision_scale_;

  size_t num_joint_state_;
  sensor_msgs::msg::JointState::ConstSharedPtr latest_joint_state_;

  size_t num_commands_;
  trajectory_msgs::msg::JointTrajectory::SharedPtr latest_traj_cmd_;
  std_msgs::msg::Float64MultiArray::ConstSharedPtr latest_array_cmd_;

  bool status_seen_;
  StatusCode status_tracking_code_ = StatusCode::NO_WARNING;

  const double PUBLISH_HZ;
  const double PUBLISH_PERIOD;
  const double TIMEOUT_ITERATIONS;
  rclcpp::Rate publish_loop_rate_;
  const std::string servo_server_node_name_;

  mutable std::mutex latest_state_mutex_;
};  // class ServoFixture
}  // namespace moveit_servo
