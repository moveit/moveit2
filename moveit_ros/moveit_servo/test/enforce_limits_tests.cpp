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
   Desc:   Enforce limits unit tests
*/

#include <moveit/utils/robot_model_test_utils.h>
#include <gtest/gtest.h>
#include <moveit_servo/enforce_limits.hpp>

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

class EnforceLimitsTests : public testing::Test
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

TEST_F(EnforceLimitsTests, VelocityScalingTest)
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

TEST_F(EnforceLimitsTests, NegativeJointAngleDeltasTest)
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

TEST_F(EnforceLimitsTests, LowJointVelocityDeltaTest)
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

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
