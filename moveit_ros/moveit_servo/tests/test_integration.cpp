/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, PickNik Inc.
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

/* Author    : V Mohammed Ibrahim
   Desc      : Integration tests for the servo c++ API
   Title     : test_integration.cpp
   Project   : moveit_servo
   Created   : 07/07/2023
*/

#include "servo_cpp_fixture.hpp"

namespace
{

TEST_F(ServoCppFixture, JointJogTest)
{
  planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor_);
  auto robot_state = std::make_shared<moveit::core::RobotState>(locked_scene->getCurrentState());

  moveit_servo::StatusCode status_curr, status_next, status_initial;
  moveit_servo::JointJogCommand joint_jog_z{ { "panda_joint7" }, { 1.0 } };
  moveit_servo::JointJogCommand zero_joint_jog;

  // Compute next state.
  servo_test_instance_->setCommandType(moveit_servo::CommandType::JOINT_JOG);
  status_initial = servo_test_instance_->getStatus();
  ASSERT_EQ(status_initial, moveit_servo::StatusCode::NO_WARNING);

  moveit_servo::KinematicState curr_state = servo_test_instance_->getNextJointState(robot_state, zero_joint_jog);
  status_curr = servo_test_instance_->getStatus();
  ASSERT_EQ(status_curr, moveit_servo::StatusCode::NO_WARNING);

  moveit_servo::KinematicState next_state = servo_test_instance_->getNextJointState(robot_state, joint_jog_z);
  status_next = servo_test_instance_->getStatus();
  ASSERT_EQ(status_next, moveit_servo::StatusCode::NO_WARNING);

  // Check against manually verified value
  double delta = next_state.positions[6] - curr_state.positions[6];
  constexpr double tol = 1.0e-5;
  ASSERT_NEAR(delta, 0.01, tol);
}

TEST_F(ServoCppFixture, TwistTest)
{
  planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor_);
  auto robot_state = std::make_shared<moveit::core::RobotState>(locked_scene->getCurrentState());

  moveit_servo::StatusCode status_curr, status_next, status_initial;
  moveit_servo::TwistCommand twist_non_zero{ "panda_link0", { 0.0, 0.0, 0.0, 0.0, 0.0, 0.1 } };
  moveit_servo::TwistCommand twist_zero{ "panda_link0", { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };

  servo_test_instance_->setCommandType(moveit_servo::CommandType::TWIST);
  status_initial = servo_test_instance_->getStatus();
  ASSERT_EQ(status_initial, moveit_servo::StatusCode::NO_WARNING);
  moveit_servo::KinematicState curr_state = servo_test_instance_->getNextJointState(robot_state, twist_zero);
  status_curr = servo_test_instance_->getStatus();
  ASSERT_EQ(status_curr, moveit_servo::StatusCode::NO_WARNING);

  moveit_servo::KinematicState next_state = servo_test_instance_->getNextJointState(robot_state, twist_non_zero);
  status_next = servo_test_instance_->getStatus();
  ASSERT_EQ(status_next, moveit_servo::StatusCode::NO_WARNING);

  // Check against manually verified value
  constexpr double expected_delta = -0.000338;
  double delta = next_state.positions[6] - curr_state.positions[6];
  constexpr double tol = 1.0e-5;
  ASSERT_NEAR(delta, expected_delta, tol);
}

TEST_F(ServoCppFixture, NonPlanningFrameTwistTest)
{
  planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor_);
  auto robot_state = std::make_shared<moveit::core::RobotState>(locked_scene->getCurrentState());

  moveit_servo::StatusCode status_curr, status_next, status_initial;
  moveit_servo::TwistCommand twist_non_zero{ "panda_link8", { 0.0, 0.0, 0.0, 0.0, 0.0, 0.1 } };
  moveit_servo::TwistCommand twist_zero{ "panda_link8", { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };

  servo_test_instance_->setCommandType(moveit_servo::CommandType::TWIST);
  status_initial = servo_test_instance_->getStatus();
  ASSERT_EQ(status_initial, moveit_servo::StatusCode::NO_WARNING);
  moveit_servo::KinematicState curr_state = servo_test_instance_->getNextJointState(robot_state, twist_zero);
  status_curr = servo_test_instance_->getStatus();
  ASSERT_EQ(status_curr, moveit_servo::StatusCode::NO_WARNING);

  moveit_servo::KinematicState next_state = servo_test_instance_->getNextJointState(robot_state, twist_non_zero);
  status_next = servo_test_instance_->getStatus();
  ASSERT_EQ(status_next, moveit_servo::StatusCode::NO_WARNING);

  // Check against manually verified value
  constexpr double expected_delta = 0.000338;
  double delta = next_state.positions[6] - curr_state.positions[6];
  constexpr double tol = 1.0e-5;
  ASSERT_NEAR(delta, expected_delta, tol);
}

TEST_F(ServoCppFixture, PoseTest)
{
  planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor_);
  auto robot_state = std::make_shared<moveit::core::RobotState>(locked_scene->getCurrentState());

  moveit_servo::StatusCode status_curr, status_next, status_initial;
  moveit_servo::PoseCommand zero_pose, non_zero_pose;
  zero_pose.frame_id = "panda_link0";
  zero_pose.pose = getCurrentPose("panda_link8");

  non_zero_pose.frame_id = "panda_link0";
  non_zero_pose.pose = getCurrentPose("panda_link8");
  non_zero_pose.pose.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));

  servo_test_instance_->setCommandType(moveit_servo::CommandType::POSE);
  status_initial = servo_test_instance_->getStatus();
  ASSERT_EQ(status_initial, moveit_servo::StatusCode::NO_WARNING);

  moveit_servo::KinematicState curr_state = servo_test_instance_->getNextJointState(robot_state, zero_pose);
  status_curr = servo_test_instance_->getStatus();
  ASSERT_EQ(status_curr, moveit_servo::StatusCode::NO_WARNING);

  moveit_servo::KinematicState next_state = servo_test_instance_->getNextJointState(robot_state, non_zero_pose);
  status_next = servo_test_instance_->getStatus();
  ASSERT_EQ(status_next, moveit_servo::StatusCode::NO_WARNING);

  // Check against manually verified value
  constexpr double expected_delta = 0.003364;
  double delta = next_state.positions[6] - curr_state.positions[6];
  constexpr double tol = 1.0e-5;
  ASSERT_NEAR(delta, expected_delta, tol);
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
