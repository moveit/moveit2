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

/* Author: Andy Zelenak
   Desc:   Test of tracking toward a pose
*/

// C++
#include <string>
#include <thread>

// ROS
#include <rclcpp/rclcpp.hpp>

// Testing
#include <gtest/gtest.h>

// Servo
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <moveit_servo/servo_parameters.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.pose_tracking_test");

static constexpr double TRANSLATION_TOLERANCE = 0.01;  // meters
static constexpr double ROTATION_TOLERANCE = 0.1;      // quaternion

namespace moveit_servo
{
class PoseTrackingFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { this->executor_->spin(); });

    servo_parameters_ = moveit_servo::ServoParameters::makeServoParameters(node_);
    ;
    if (servo_parameters_ == nullptr)
    {
      RCLCPP_FATAL(LOGGER, "Could not get servo parameters!");
      exit(EXIT_FAILURE);
    }

    // store test constants as shared pointer to constant struct
    {
      auto test_parameters = std::make_shared<struct TestParameters>();
      test_parameters->publish_hz = 2.0 / servo_parameters_->incoming_command_timeout;
      test_parameters->publish_period = 1.0 / test_parameters->publish_hz;
      test_parameters->timeout_iterations = 10 * test_parameters->publish_hz;
      test_parameters_ = test_parameters;
    }

    // Load the planning scene monitor
    if (!planning_scene_monitor_->getPlanningScene())
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Error in setting up the PlanningSceneMonitor.");
      exit(EXIT_FAILURE);
    }

    planning_scene_monitor_->providePlanningSceneService();
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor(
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
        false /* skip octomap monitor */);
    planning_scene_monitor_->startStateMonitor(servo_parameters_->joint_topic);
    planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

    // Wait for Planning Scene Monitor to setup
    if (!planning_scene_monitor_->waitForCurrentRobotState(node_->now(), 5.0 /* seconds */))
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Error waiting for current robot state in PlanningSceneMonitor.");
      exit(EXIT_FAILURE);
    }

    tracker_ = std::make_shared<moveit_servo::PoseTracking>(node_, servo_parameters_, planning_scene_monitor_);

    // Tolerance for pose seeking
    translation_tolerance_ << TRANSLATION_TOLERANCE, TRANSLATION_TOLERANCE, TRANSLATION_TOLERANCE;
  }

  PoseTrackingFixture()
    : node_(std::make_shared<rclcpp::Node>("pose_tracking_testing"))
    , executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
    , planning_scene_monitor_(
          std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, "robot_description"))
  {
    // Init ROS interfaces
    // Publishers
    target_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 1);
  }

  void TearDown() override
  {
    executor_->cancel();
    if (executor_thread_.joinable())
      executor_thread_.join();
  }

protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Executor::SharedPtr executor_;
  std::thread executor_thread_;

  moveit_servo::ServoParameters::SharedConstPtr servo_parameters_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  struct TestParameters
  {
    double publish_hz;
    double publish_period;
    double timeout_iterations;
  };
  std::shared_ptr<const struct TestParameters> test_parameters_;

  moveit_servo::PoseTrackingPtr tracker_;

  Eigen::Vector3d translation_tolerance_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
};  // class PoseTrackingFixture

// Check for commands going out to ros_control
TEST_F(PoseTrackingFixture, OutgoingMsgTest)
{
  // halt Servoing when first msg to ros_control is received
  // and test some conditions
  trajectory_msgs::msg::JointTrajectory last_received_msg;
  std::function<void(const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr)> traj_callback =
      [&/* this */](const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr msg) {
        EXPECT_EQ(msg->header.frame_id, "panda_link0");

        // Exact joint positions may vary based on IK solver
        // so check that the EE transform is as expected
        moveit::core::RobotStatePtr temp_state(planning_scene_monitor_->getStateMonitor()->getCurrentState());
        const std::string group_name = "panda_arm";
        // copyJointGroupPositions can't take a const vector, make a copy
        std::vector<double> positions(msg->points[0].positions);
        temp_state->copyJointGroupPositions(group_name, positions);
        Eigen::Isometry3d hand_tf = temp_state->getFrameTransform("panda_hand");

        moveit::core::RobotStatePtr test_state(planning_scene_monitor_->getStateMonitor()->getCurrentState());
        std::vector<double> test_positions = { 0, -0.785, 0, -2.360, 0, 1.571, 0.758 };
        test_state->copyJointGroupPositions(group_name, test_positions);
        Eigen::Isometry3d test_hand_tf = test_state->getFrameTransform("panda_hand");

        double precision = 0.02;  // Hilbert-Schmidt norm
        EXPECT_TRUE(test_hand_tf.isApprox(hand_tf, precision));

        this->tracker_->stopMotion();
        return;
      };
  auto traj_sub = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/panda_arm_controller/joint_trajectory", 1, traj_callback);

  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = "panda_link4";
  target_pose.header.stamp = node_->now();
  target_pose.pose.position.x = 0.2;
  target_pose.pose.position.y = 0.2;
  target_pose.pose.position.z = 0.2;
  target_pose.pose.orientation.w = 1;

  // Republish the target pose in a new thread, as if the target is moving
  std::thread target_pub_thread([&] {
    size_t msg_count = 0;
    rclcpp::Rate loop_rate(50);
    while (++msg_count < 100)
    {
      target_pose_pub_->publish(target_pose);
      loop_rate.sleep();
    }
  });

  // resetTargetPose() can be used to clear the target pose and wait for a new one, e.g. when moving between multiple waypoints
  tracker_->resetTargetPose();

  tracker_->moveToPose(translation_tolerance_, ROTATION_TOLERANCE, 1 /* target pose timeout */);

  target_pub_thread.join();
}

}  // namespace moveit_servo

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
