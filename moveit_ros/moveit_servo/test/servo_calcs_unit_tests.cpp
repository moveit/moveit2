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

/* Author: Tyler Weaver, Andy Zelenak
   Desc:   Unit tests
*/

#include <gtest/gtest.h>

#include <moveit/utils/robot_model_test_utils.h>

#include <moveit_servo/enforce_limits.hpp>
#include <moveit_servo/servo_calcs.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/utilities.h>

namespace
{
constexpr double PUBLISH_PERIOD = 0.01;

void checkVelocityLimits(const moveit::core::JointModelGroup* joint_model_group, const Eigen::ArrayXd& velocity)
{
  std::size_t joint_index{ 0 };
  for (const moveit::core::JointModel* joint : joint_model_group->getActiveJointModels())
  {
    const auto& bounds = joint->getVariableBounds(joint->getName());

    if (bounds.velocity_bounded_)
    {
      EXPECT_GE(velocity(joint_index), bounds.min_velocity_) << "Joint " << joint_index << " violates velocity limit";
      EXPECT_LE(velocity(joint_index), bounds.max_velocity_) << "Joint " << joint_index << " violates velocity limit";
    }
    ++joint_index;
  }
}

class ServoCalcsUnitTests : public testing::Test
{
protected:
  void SetUp() override
  {
    robot_model_ = moveit::core::loadTestingRobotModel("panda");
    joint_model_group_ = robot_model_->getJointModelGroup("panda_arm");
  }

  moveit::core::RobotModelPtr robot_model_;
  const moveit::core::JointModelGroup* joint_model_group_;
};

}  // namespace

TEST_F(ServoCalcsUnitTests, VelocitiesTooFast)
{
  // Request velocities that are too fast
  std::vector<double> joint_position{ 0, 0, 0, 0, 0, 0, 0 };
  std::vector<double> joint_velocity{ 0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 };
  sensor_msgs::msg::JointState joint_state;
  joint_state.position = joint_position;
  joint_state.velocity = joint_velocity;

  moveit_servo::enforceVelocityLimits(joint_model_group_, PUBLISH_PERIOD, joint_state);

  Eigen::ArrayXd eigen_velocity =
      Eigen::Map<Eigen::ArrayXd, Eigen::Unaligned>(joint_state.velocity.data(), joint_state.velocity.size());
  checkVelocityLimits(joint_model_group_, eigen_velocity);
}

TEST_F(ServoCalcsUnitTests, NegativeVelocitiesTooFast)
{
  // Negative velocities exceeding the limit
  std::vector<double> joint_position{ 0, 0, 0, 0, 0, 0, 0 };
  std::vector<double> joint_velocity{ 0, -1.0, -2.0, -3.0, -4.0, -5.0, -6.0 };
  sensor_msgs::msg::JointState joint_state;
  joint_state.position = joint_position;
  joint_state.velocity = joint_velocity;

  moveit_servo::enforceVelocityLimits(joint_model_group_, PUBLISH_PERIOD, joint_state);

  Eigen::ArrayXd eigen_velocity =
      Eigen::Map<Eigen::ArrayXd, Eigen::Unaligned>(joint_state.velocity.data(), joint_state.velocity.size());
  checkVelocityLimits(joint_model_group_, eigen_velocity);
}

TEST_F(ServoCalcsUnitTests, AcceptableJointVelocities)
{
  // Final test with joint velocities that are acceptable
  std::vector<double> joint_position{ 0, 0, 0, 0, 0, 0, 0 };
  std::vector<double> joint_velocity{ 0, 0.001, 0.001, -0.001, 0.001, 0.001, 0.001 };
  sensor_msgs::msg::JointState joint_state;
  joint_state.position = joint_position;
  joint_state.velocity = joint_velocity;

  moveit_servo::enforceVelocityLimits(joint_model_group_, PUBLISH_PERIOD, joint_state);

  Eigen::ArrayXd eigen_velocity =
      Eigen::Map<Eigen::ArrayXd, Eigen::Unaligned>(joint_state.velocity.data(), joint_state.velocity.size());
  checkVelocityLimits(joint_model_group_, eigen_velocity);
}

TEST_F(ServoCalcsUnitTests, SingularityScaling)
{
  // If we are at a singularity, we should halt
  Eigen::VectorXd commanded_twist(6);
  commanded_twist << 1, 0, 0, 0, 0, 0;

  // Start near a singularity
  std::shared_ptr<moveit::core::RobotState> robot_state = std::make_shared<moveit::core::RobotState>(robot_model_);
  robot_state->setToDefaultValues();
  robot_state->setVariablePosition("panda_joint1", 0.221);
  robot_state->setVariablePosition("panda_joint2", 0.530);
  robot_state->setVariablePosition("panda_joint3", -0.231);
  robot_state->setVariablePosition("panda_joint4", -0.920);
  robot_state->setVariablePosition("panda_joint5", 0.117);
  robot_state->setVariablePosition("panda_joint6", 1.439);
  robot_state->setVariablePosition("panda_joint7", -1.286);

  Eigen::MatrixXd jacobian = robot_state->getJacobian(joint_model_group_);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd =
      Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd matrix_s = svd.singularValues().asDiagonal();
  Eigen::MatrixXd pseudo_inverse = svd.matrixV() * matrix_s.inverse() * svd.matrixU().transpose();

  // Use very low thresholds to ensure they are triggered
  double hard_stop_singularity_threshold = 2;
  double lower_singularity_threshold = 1;
  double leaving_singularity_threshold_multiplier = 2;

  rclcpp::Clock clock;
  moveit_servo::StatusCode status;

  double scaling_factor = moveit_servo::velocityScalingFactorForSingularity(
      joint_model_group_, commanded_twist, svd, pseudo_inverse, hard_stop_singularity_threshold,
      lower_singularity_threshold, leaving_singularity_threshold_multiplier, clock, robot_state, status);

  EXPECT_EQ(scaling_factor, 0);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
