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

/* Author: Dave Coleman and Tyler Weaver
   Desc:   Test for the C++ interface to moveit_servo
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

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.servo_cpp_interface_test.cpp");

namespace moveit_servo
{
class ServoFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    // node_->set_parameter({"use_sim_time", true});
    executor_->add_node(node_);
    executor_task_fut_ = std::async(std::launch::async, [this]() {this->executor_->spin();});
  }

  ServoFixture()
  : node_(std::make_shared<rclcpp::Node>("diffbot_controller_test"))
  , executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
  {
    // Read the parameters used for testing
    parameters_ = getTestParameters();

    // Init ROS interfaces
    // Publishers
    pub_twist_cmd_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(parameters_->cartesian_command_in_topic, 10);
    pub_joint_cmd_ = node_->create_publisher<control_msgs::msg::JointJog>(parameters_->joint_command_in_topic, 10);
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
  }

  bool waitForFirstStatus()
  {
    rclcpp::WaitSet wait_set(std::vector<rclcpp::WaitSet::SubscriptionEntry>{ { sub_servo_status_ } });
    // auto wait_result = wait_set.wait();
    auto wait_result = wait_set.wait(std::chrono::seconds(15));

    std_msgs::msg::Int8 recieved_msg;
    rclcpp::MessageInfo msg_info;
    sub_servo_status_->take(recieved_msg, msg_info);  // TODO(adamp): this returns a bool, need to decide
                                                                // what happens if this returns false

    statusCB(std::make_shared<std_msgs::msg::Int8>(recieved_msg));
    RCLCPP_WARN_STREAM(LOGGER, "Wait kind is: " << wait_result.kind() << ". Status code is: " << latest_status_);
    return wait_result.kind() == rclcpp::WaitResultKind::Ready;
  }

  // Set up for callbacks (so they aren't run for EVERY test)
  bool setupStartClient()
  {
    // Start client
    client_servo_start_ = node_->create_client<std_srvs::srv::Trigger>("/start_servo");
    while (!client_servo_start_->wait_for_service(1s))
    {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(LOGGER, "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(LOGGER, "client_servo_start_ service not available, waiting again...");
    }

    // If we setup the start client, also setup the stop client...
    client_servo_stop_ = node_->create_client<std_srvs::srv::Trigger>("/stop_servo");
    while (!client_servo_stop_->wait_for_service(1s))
    {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(LOGGER, "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(LOGGER, "client_servo_stop_ service not available, waiting again...");
    }
    return true;
  }

  bool setupPauseClient()
  {
    client_servo_pause_ = node_->create_client<std_srvs::srv::Trigger>("/pause_servo");
    while (!client_servo_pause_->wait_for_service(1s))
    {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(LOGGER, "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(LOGGER, "client_servo_pause_ service not available, waiting again...");
    }
    return true;
  }

  bool setupUnpauseClient()
  {
    client_servo_unpause_ = node_->create_client<std_srvs::srv::Trigger>("/unpause_servo");
    while (!client_servo_unpause_->wait_for_service(1s))
    {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(LOGGER, "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(LOGGER, "client_servo_unpause_ service not available, waiting again...");
    }
    return true;
  }

  bool setupControlDimsClient()
  {
    client_change_control_dims_ = node_->create_client<moveit_msgs::srv::ChangeControlDimensions>("/servo_server/change_control_dimensions");
    while (!client_change_control_dims_->wait_for_service(1s))
    {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(LOGGER, "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(LOGGER, "client_change_control_dims_ service not available, waiting again...");
    }
    return true;
  }

  bool setupDriftDimsClient()
  {
    client_change_drift_dims_ = node_->create_client<moveit_msgs::srv::ChangeDriftDimensions>("/servo_server/change_drift_dimensions");
    while (!client_change_drift_dims_->wait_for_service(1s))
    {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(LOGGER, "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(LOGGER, "client_change_drift_dims_ service not available, waiting again...");
    }
    return true;
  }

  bool setupStatusSub()
  {
    sub_servo_status_ = node_->create_subscription<std_msgs::msg::Int8>("/" +
      parameters_->status_topic, 10, std::bind(&ServoFixture::statusCB, this, std::placeholders::_1));
    return true;
  }

  bool setupCollisionScaleSub()
  {
    sub_collision_scale_ = node_->create_subscription<std_msgs::msg::Float64>(
      "collision_velocity_scale", ROS_QUEUE_SIZE, std::bind(&ServoFixture::collisionScaleCB, this, std::placeholders::_1));
    return true;
  }

  bool setupCommandSub(std::string command_type)
  {
    if (command_type == "trajectory_msgs/JointTrajectory")
    {
      sub_trajectory_cmd_output_ = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        parameters_->command_out_topic, 10, std::bind(&ServoFixture::trajectoryCommandCB, this, std::placeholders::_1));
      return true;
    }
    else if (command_type == "std_msgs/Float64MultiArray")
    {
      sub_array_cmd_output_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        parameters_->command_out_topic, 10, std::bind(&ServoFixture::arrayCommandCB, this, std::placeholders::_1));
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
      parameters_->joint_topic, 10, std::bind(&ServoFixture::jointStateCB, this, std::placeholders::_1));
    return true;
  }

  void statusCB(const std_msgs::msg::Int8::SharedPtr msg)
  {
    const std::lock_guard<std::mutex> lock(latest_state_mutex_);
    ++num_status_;
    latest_status_ = static_cast<StatusCode>(msg.get()->data);
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

protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Executor::SharedPtr executor_;
  moveit_servo::ServoParametersPtr parameters_;
  std::future<void> executor_task_fut_;

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

  mutable std::mutex latest_state_mutex_;
};  // class ServoFixture

TEST_F(ServoFixture, StartStopTest)
{
  // Setup the start/stop clients, and the command callback
  ASSERT_TRUE(setupStartClient());
  ASSERT_TRUE(setupStatusSub());
  ASSERT_TRUE(setupJointStateSub());

  // Wait for a joint state message to ensure fake_joint_driver is up
  bool got_msg = false;
  while (!got_msg)
  {
    if (getNumJointState() > 0)
      got_msg = true;

    rclcpp::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_TRUE(got_msg);

  // Try to start Servo
  auto start_result = client_servo_start_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  EXPECT_TRUE(start_result.get()->success);

  // With servo running we should get a status that should be NO_WARNING
  rclcpp::sleep_for(std::chrono::milliseconds(20));
  EXPECT_TRUE(getNumStatus() > 0);
  EXPECT_TRUE(latest_status_ == moveit_servo::StatusCode::NO_WARNING);

  // Now stop servo and wait
  auto stop_result = client_servo_stop_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  EXPECT_TRUE(stop_result.get()->success);
  rclcpp::sleep_for(std::chrono::seconds(1));
  resetNumStatus();

  // Restart and recheck Servo
  start_result = client_servo_start_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  EXPECT_TRUE(start_result.get()->success);
  rclcpp::sleep_for(std::chrono::milliseconds(20));
  EXPECT_TRUE(getNumStatus() > 0);
  EXPECT_TRUE(latest_status_ == moveit_servo::StatusCode::NO_WARNING);
}

TEST_F(ServoFixture, SendTwistStampedTest)
{
  ASSERT_TRUE(setupStartClient());
  ASSERT_TRUE(setupCommandSub(parameters_->command_out_type));

  // Start Servo
  auto start_result = client_servo_start_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_TRUE(start_result.get()->success);

  // We want to count the number of commands Servo publishes, we need timing
  auto time_start = node_->now();

  // Publish N messages with some time between, ensure it's less than the timeout for Servo
  size_t num_commands = 30;
  rclcpp::Rate loop_rate(2/parameters_->incoming_command_timeout);
  resetNumCommands();
  for (size_t i = 0; i < num_commands && rclcpp::ok(); ++i)
  {
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = node_->now();
    msg->twist.linear.x = 0.1;
    msg->twist.angular.z = 0.5;
    pub_twist_cmd_->publish(std::move(msg));
    loop_rate.sleep();
  }

  // Capture the time and number of recieved messages
  auto time_end = node_->now();
  auto num_recieved = getNumCommands();

  // Compare actual number recieved to expected number
  auto num_expected = (time_end - time_start).seconds() / parameters_->publish_period;
  
  EXPECT_GT(num_recieved, 0.7*num_expected);
  EXPECT_LT(num_recieved, 1.1*num_expected);
}

TEST_F(ServoFixture, SendJointServoTest)
{
  ASSERT_TRUE(setupStartClient());
  ASSERT_TRUE(setupCommandSub(parameters_->command_out_type));

  // Start Servo
  auto start_result = client_servo_start_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_TRUE(start_result.get()->success);

  // We want to count the number of commands Servo publishes, we need timing
  auto time_start = node_->now();

  // Publish N messages with some time between, ensure it's less than the timeout for Servo
  size_t num_commands = 30;
  rclcpp::Rate loop_rate(2/parameters_->incoming_command_timeout);
  resetNumCommands();
  for (size_t i = 0; i < num_commands && rclcpp::ok(); ++i)
  {
    auto msg = std::make_unique<control_msgs::msg::JointJog>();
    msg->header.stamp = node_->now();
    msg->header.frame_id = "panda_link3";
    msg->joint_names.push_back("panda_joint1");
    msg->velocities.push_back(0.1);
    pub_joint_cmd_->publish(std::move(msg));
    loop_rate.sleep();
  }

  // Capture the time and number of recieved messages
  auto time_end = node_->now();
  auto num_recieved = getNumCommands();

  // Compare actual number recieved to expected number
  auto num_expected = (time_end - time_start).seconds() / parameters_->publish_period;
  
  EXPECT_GT(num_recieved, 0.8*num_expected);
  EXPECT_LT(num_recieved, 1.1*num_expected);
}

TEST_F(ServoFixture, StaleCommandStop)
{
  ASSERT_TRUE(setupStartClient());
  ASSERT_TRUE(setupCommandSub(parameters_->command_out_type));

  // Start Servo
  auto start_result = client_servo_start_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_TRUE(start_result.get()->success);

  // Setup the message to publish (only once)
  auto msg = std::make_unique<control_msgs::msg::JointJog>();
  msg->header.stamp = node_->now();
  msg->joint_names.push_back("panda_joint1");
  msg->header.frame_id = "panda_link3";
  msg->velocities.push_back(0.1);

  // Get current position
  double start_position = getLatestTrajCommand().points[0].positions[0];
  
  // Publish once
  pub_joint_cmd_->publish(std::move(msg));

  // Wait the stale limit
  const int sleep_time = 1000* parameters_->incoming_command_timeout;
  rclcpp::sleep_for(std::chrono::milliseconds(sleep_time));
  
  // Get the new current position (should be different than first)
  double middle_position = getLatestTrajCommand().points[0].positions[0];
  EXPECT_NE(start_position, middle_position);

  // Wait the stale limit
  rclcpp::sleep_for(std::chrono::milliseconds(sleep_time));

  // Get the current position (should be no change)
  double end_position = getLatestTrajCommand().points[0].positions[0];
  EXPECT_EQ(middle_position, end_position);
}

}  // namespace moveit_servo

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
