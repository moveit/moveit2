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

/* Author    : V Mohammed Ibrahim
   Desc      : Tests for utilities that depend on the robot/ robot state.
   Title     : test_utils.cpp
   Project   : moveit_servo
   Created   : 06/20/2023
*/

#include <gtest/gtest.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "servo_cpp_fixture.hpp"
#include <moveit_servo/utils.hpp>

namespace
{

TEST(ServoUtilsUnitTests, validVector)
{
  Eigen::VectorXd valid_vector(7);
  valid_vector << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  EXPECT_TRUE(moveit_servo::isValidCommand(valid_vector));
}

TEST(ServoUtilsUnitTests, invalidVector)
{
  Eigen::VectorXd invalid_vector(6);
  invalid_vector << 0.0, 0.0, 0.0, 0.0, std::nan("");
  EXPECT_FALSE(moveit_servo::isValidCommand(invalid_vector));
}

TEST(ServoUtilsUnitTests, validTwist)
{
  Eigen::VectorXd valid_vector(6);
  valid_vector << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  moveit_servo::Twist valid_twist{ "panda_link0", valid_vector };
  EXPECT_TRUE(moveit_servo::isValidCommand(valid_twist));
}

TEST(ServoUtilsUnitTests, emptyTwistFrame)
{
  Eigen::VectorXd invalid_vector(6);
  invalid_vector << 0.0, 0.0, 0.0, 0.0, std::nan("");
  moveit_servo::Twist invalid_twist;
  invalid_twist.velocities = invalid_vector;
  EXPECT_FALSE(moveit_servo::isValidCommand(invalid_twist));
}

TEST(ServoUtilsUnitTests, invalidTwistVelocities)
{
  Eigen::VectorXd invalid_vector(6);
  invalid_vector << 0.0, 0.0, 0.0, 0.0, 0.0, std::nan("");
  moveit_servo::Twist invalid_twist{ "panda_link0", invalid_vector };
  EXPECT_FALSE(moveit_servo::isValidCommand(invalid_twist));
}

TEST(ServoUtilsUnitTests, validIsometry)
{
  Eigen::Isometry3d valid_isometry;
  valid_isometry.setIdentity();
  EXPECT_TRUE(moveit_servo::isValidCommand(valid_isometry));
}

TEST(ServoUtilsUnitTests, invalidIsometry)
{
  Eigen::Isometry3d invalid_isometry;
  invalid_isometry.setIdentity();
  invalid_isometry.translation().z() = std::nan("");
  EXPECT_FALSE(moveit_servo::isValidCommand(invalid_isometry));
}

TEST(ServoUtilsUnitTests, validPose)
{
  Eigen::Isometry3d valid_isometry;
  valid_isometry.setIdentity();
  moveit_servo::Pose valid_pose{ "panda_link0", valid_isometry };
  EXPECT_TRUE(moveit_servo::isValidCommand(valid_pose));
}

TEST_F(ServoCppFixture, testVeclocityScaling)
{
  moveit::core::JointBoundsVector joint_bounds = joint_model_group_->getActiveJointModelsBounds();
  // Get the upper bound for the velocities of each joint.
  Eigen::VectorXd incoming_velocities(joint_bounds.size());
  for (size_t i = 0; i < joint_bounds.size(); i++)
  {
    const auto joint_bound = (*joint_bounds[i])[0];
    if (joint_bound.velocity_bounded_)
    {
      incoming_velocities(i) = joint_bound.max_velocity_;
    }
  }

  // Create incoming velocities with only joint 1 and joint 2 over limit by a factor of 0.1 and 0.05
  // Scale down all other joint velocities by 0.3 to keep it within limits.
  incoming_velocities(0) *= 1.1;
  incoming_velocities(1) *= 1.05;
  incoming_velocities.tail<5>() *= 0.7;

  // The resulting scaling factor selected should be approximately 0.90909
  double user_velocity_override = 0.0;
  double scaling_factor =
      moveit_servo::velocityScalingFactor(incoming_velocities, joint_bounds, user_velocity_override);
  constexpr double tol = 0.001;
  ASSERT_NEAR(scaling_factor, 0.90909, tol);
}

TEST_F(ServoCppFixture, testLeavingSingularity)
{
  Eigen::Vector<double, 6> cartesian_delta{ 0.005, 0.0, 0.0, 0.0, 0.0, 0.0 };

  // Home state
  Eigen::Vector<double, 7> home_state{ 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785 };
  robot_state_->setJointGroupActivePositions(joint_model_group_, home_state);
  auto scaling_result = moveit_servo::velocityScalingFactorForSingularity(joint_model_group_, robot_state_,
                                                                          cartesian_delta, servo_params_);
  ASSERT_EQ(scaling_result.second, moveit_servo::StatusCode::NO_WARNING);

  // Approach singularity
  Eigen::Vector<double, 7> state_approaching_singularity{ 0.0, 0.334, 0.0, -1.177, 0.0, 1.510, 0.785 };
  robot_state_->setJointGroupActivePositions(joint_model_group_, state_approaching_singularity);
  scaling_result = moveit_servo::velocityScalingFactorForSingularity(joint_model_group_, robot_state_, cartesian_delta,
                                                                     servo_params_);
  ASSERT_EQ(scaling_result.second, moveit_servo::StatusCode::DECELERATE_FOR_APPROACHING_SINGULARITY);

  // Move away from singularity
  cartesian_delta(0) *= -1;
  Eigen::Vector<double, 7> state_leaving_singularity{ 0.0, 0.3458, 0.0, -1.1424, 0.0, 1.4865, 0.785 };
  robot_state_->setJointGroupActivePositions(joint_model_group_, state_leaving_singularity);
  scaling_result = moveit_servo::velocityScalingFactorForSingularity(joint_model_group_, robot_state_, cartesian_delta,
                                                                     servo_params_);
  ASSERT_EQ(scaling_result.second, moveit_servo::StatusCode::DECELERATE_FOR_LEAVING_SINGULARITY);
}

TEST_F(ServoCppFixture, testApproachingSingularity)
{
  Eigen::Vector<double, 6> cartesian_delta{ 0.005, 0.0, 0.0, 0.0, 0.0, 0.0 };

  // Home state
  robot_state_->setToDefaultValues(joint_model_group_, "ready");
  auto scaling_result = moveit_servo::velocityScalingFactorForSingularity(joint_model_group_, robot_state_,
                                                                          cartesian_delta, servo_params_);
  ASSERT_EQ(scaling_result.second, moveit_servo::StatusCode::NO_WARNING);

  // Approach singularity
  Eigen::Vector<double, 7> state_approaching_singularity{ 0.0, 0.334, 0.0, -1.177, 0.0, 1.510, 0.785 };
  robot_state_->setJointGroupActivePositions(joint_model_group_, state_approaching_singularity);
  scaling_result = moveit_servo::velocityScalingFactorForSingularity(joint_model_group_, robot_state_, cartesian_delta,
                                                                     servo_params_);
  ASSERT_EQ(scaling_result.second, moveit_servo::StatusCode::DECELERATE_FOR_APPROACHING_SINGULARITY);
}

TEST_F(ServoCppFixture, testHaltForSingularity)
{
  Eigen::Vector<double, 6> cartesian_delta{ 0.005, 0.0, 0.0, 0.0, 0.0, 0.0 };

  // Home state
  robot_state_->setToDefaultValues(joint_model_group_, "ready");
  auto scaling_result = moveit_servo::velocityScalingFactorForSingularity(joint_model_group_, robot_state_,
                                                                          cartesian_delta, servo_params_);
  ASSERT_EQ(scaling_result.second, moveit_servo::StatusCode::NO_WARNING);

  // Move to singular state.
  Eigen::Vector<double, 7> singular_state{ -0.0001, 0.5690, 0.0005, -0.7782, 0.0, 1.3453, 0.7845 };
  robot_state_->setJointGroupActivePositions(joint_model_group_, singular_state);
  scaling_result = moveit_servo::velocityScalingFactorForSingularity(joint_model_group_, robot_state_, cartesian_delta,
                                                                     servo_params_);
  ASSERT_EQ(scaling_result.second, moveit_servo::StatusCode::HALT_FOR_SINGULARITY);
}

TEST_F(ServoCppFixture, testGetEndEffectorFrame)
{
  Eigen::Isometry3d ee_pose = servo_test_instance_->getEndEffectorPose();
  ASSERT_TRUE(moveit_servo::isValidCommand(ee_pose));
}

TEST_F(ServoCppFixture, testPoseFromCartesianDelta)
{
  kinematics::KinematicsBaseConstPtr ik_solver = joint_model_group_->getSolverInstance();
  const Eigen::Isometry3d base_to_tip_frame_transform =
      robot_state_->getGlobalLinkTransform(ik_solver->getBaseFrame()).inverse() *
      robot_state_->getGlobalLinkTransform(ik_solver->getTipFrame());

  // Pose message for a carteisan delta with only a +45 degree rotation about z.
  Eigen::Vector<double, 6> cartesian_delta{ 0.0, 0.0, 0.0, 0.0, 0.0, M_PI / 4 };

  geometry_msgs::msg::Pose received_pose =
      moveit_servo::poseFromCartesianDelta(cartesian_delta, base_to_tip_frame_transform);

  // End effector pose rotated by 45 degree
  Eigen::Isometry3d ee_pose = servo_test_instance_->getEndEffectorPose();
  ee_pose.rotate(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
  Eigen::Quaterniond ee_rotation(ee_pose.rotation());
  double ee_pose_z = ee_rotation.z();
  constexpr double tol = 0.001;
  ASSERT_NEAR(received_pose.orientation.z, ee_pose_z, tol);
}

TEST_F(ServoCppFixture, testNonPlanningFrameTwist)
{
  servo_test_instance_->expectedCommandType(moveit_servo::CommandType::TWIST);
  // Servo automatically converts the frame to planning frame
  // We test it by setting the command frame to another frame
  moveit_servo::Twist twist_wrong_frame{ "panda_link3", { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };

  moveit_servo::StatusCode status = servo_test_instance_->getStatus();
  ASSERT_EQ(status, moveit_servo::StatusCode::NO_WARNING);

  auto result = servo_test_instance_->getNextJointState(twist_wrong_frame);
  status = servo_test_instance_->getStatus();
  ASSERT_EQ(status, moveit_servo::StatusCode::NO_WARNING);
}

TEST_F(ServoCppFixture, testNonPlanningFramePose)
{
  servo_test_instance_->expectedCommandType(moveit_servo::CommandType::POSE);
  // Servo automatically converts the frame to planning frame
  // We test it by setting the command frame to another frame
  moveit_servo::Pose pose_wrong_frame;
  pose_wrong_frame.frame_id = "panda_link3";
  pose_wrong_frame.pose.setIdentity();

  moveit_servo::StatusCode status = servo_test_instance_->getStatus();
  ASSERT_EQ(status, moveit_servo::StatusCode::NO_WARNING);

  auto result = servo_test_instance_->getNextJointState(pose_wrong_frame);
  status = servo_test_instance_->getStatus();
  ASSERT_EQ(status, moveit_servo::StatusCode::NO_WARNING);
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
