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

void checkVelocityLimits(const moveit::core::JointModelGroup* joint_model_group, const Eigen::ArrayXd& delta_theta)
{
  std::size_t joint_delta_index{ 0 };
  for (const moveit::core::JointModel* joint : joint_model_group->getActiveJointModels())
  {
    const auto& bounds = joint->getVariableBounds(joint->getName());

    if (bounds.velocity_bounded_)
    {
      auto velocity = delta_theta(joint_delta_index) / PUBLISH_PERIOD;
      EXPECT_GE(velocity, bounds.min_velocity_) << "Joint " << joint_delta_index << " violates velocity limit";
      EXPECT_LE(velocity, bounds.max_velocity_) << "Joint " << joint_delta_index << " violates velocity limit";
    }
    ++joint_delta_index;
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
  // Request joint angle changes that are too fast given the control period.
  Eigen::ArrayXd delta_theta(7);
  delta_theta[0] = 0;  // rad
  delta_theta[1] = 0.01;
  delta_theta[2] = 0.02;
  delta_theta[3] = 0.03;
  delta_theta[4] = 0.04;
  delta_theta[5] = 0.05;
  delta_theta[6] = 0.06;

  // Store the original joint commands for comparison before applying velocity scaling.
  auto result_delta_theta = moveit_servo::enforceVelocityLimits(joint_model_group_, PUBLISH_PERIOD, delta_theta);

  // Test that we don't violate velocity limits
  checkVelocityLimits(joint_model_group_, result_delta_theta);
}

TEST_F(EnforceLimitsTests, NegativeJointAngleDeltasTest)
{
  // Now, negative joint angle deltas. Some will result to velocities
  // greater than the arm joint velocity limits.
  Eigen::ArrayXd delta_theta(7);
  delta_theta[0] = 0.01;  // rad
  delta_theta[1] = -0.01;
  delta_theta[2] = -0.02;
  delta_theta[3] = -0.03;
  delta_theta[4] = -0.04;
  delta_theta[5] = -0.05;
  delta_theta[6] = -0.06;

  // Store the original joint commands for comparison before applying velocity scaling.
  auto result_delta_theta = moveit_servo::enforceVelocityLimits(joint_model_group_, PUBLISH_PERIOD, delta_theta);

  // Test that we don't violate velocity limits
  checkVelocityLimits(joint_model_group_, result_delta_theta);
}

TEST_F(EnforceLimitsTests, LowJointVelocityDeltaTest)
{
  // Final test with joint angle deltas that will result in velocities
  // below the lowest Panda arm joint velocity limit.
  Eigen::ArrayXd delta_theta(7);
  delta_theta[0] = 0;  // rad
  delta_theta[1] = -0.013;
  delta_theta[2] = 0.023;
  delta_theta[3] = -0.004;
  delta_theta[4] = 0.021;
  delta_theta[5] = 0.012;
  delta_theta[6] = 0.0075;

  // Store the original joint commands for comparison before applying velocity scaling.
  auto result_delta_theta = moveit_servo::enforceVelocityLimits(joint_model_group_, PUBLISH_PERIOD, delta_theta);

  // Test that we don't violate velocity limits
  checkVelocityLimits(joint_model_group_, result_delta_theta);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
