/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, PickNik Inc.
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

#include <gtest/gtest.h>
#include <moveit/online_signal_smoothing/acceleration_filter.h>
#include <moveit/utils/robot_model_test_utils.h>

class AccelerationFilterTest : public testing::Test
{
protected:
  void SetUp() override
  {
    robot_model_ = moveit::core::loadTestingRobotModel("panda");
  };

  void set_limit(std::optional<Eigen::VectorXd> acceleration_limit)
  {
    const std::vector<moveit::core::JointModel*> joint_models = robot_model_->getJointModels();
    auto joint_model_group = robot_model_->getJointModelGroup(move_group_name_);
    size_t ind = 0;
    for (auto& joint_model : joint_models)
    {
      if (!joint_model_group->hasJointModel(joint_model->getName()))
      {
        continue;
      }
      std::vector<moveit_msgs::msg::JointLimits> joint_bounds_msg(joint_model->getVariableBoundsMsg());
      for (auto& joint_bound : joint_bounds_msg)
      {
        joint_bound.has_acceleration_limits = acceleration_limit.has_value();
        if (joint_bound.has_acceleration_limits)
        {
          joint_bound.max_acceleration = acceleration_limit.value()[ind];
        }
      }
      joint_model->setVariableBounds(joint_bounds_msg);
      ind++;
    }
  }

protected:
  moveit::core::RobotModelPtr robot_model_;
  std::string move_group_name_ = "panda_arm";
};

TEST_F(AccelerationFilterTest, FilterInitialize)
{
  online_signal_smoothing::AccelerationLimitedPlugin filter;

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("AccelerationFilterTest");
  node->declare_parameter<std::string>("move_group_name", move_group_name_);

  // fail because the number of joint is wrong
  EXPECT_FALSE(filter.initialize(node, robot_model_, 3));

  // fail because the update_rate parameter is not set
  EXPECT_FALSE(filter.initialize(node, robot_model_, 7));
  node->declare_parameter<double>("update_rate", 0.01);

  // fail because the robot does not have acceleration limits
  set_limit({});
  EXPECT_FALSE(filter.initialize(node, robot_model_, 7));

  // succeed with acceleration limits
  Eigen::VectorXd acceleration_limit = 1.2 * Eigen::VectorXd::Ones(7);
  set_limit(acceleration_limit);
  EXPECT_TRUE(filter.initialize(node, robot_model_, 7));
}

TEST_F(AccelerationFilterTest, FilterDoSmooth)
{
  online_signal_smoothing::AccelerationLimitedPlugin filter;

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("AccelerationFilterTest");
  node->declare_parameter<std::string>("move_group_name", move_group_name_);
  double update_rate = 1.0;
  node->declare_parameter<double>("update_rate", update_rate);
  Eigen::VectorXd acceleration_limit = 1.2 * Eigen::VectorXd::Ones(7);
  set_limit(acceleration_limit);

  EXPECT_TRUE(filter.initialize(node, robot_model_, 7));

  // fail when called with the wrong number of joints
  Eigen::VectorXd position = Eigen::VectorXd::Zero(5);
  Eigen::VectorXd velocity = Eigen::VectorXd::Zero(5);
  Eigen::VectorXd acceleration = Eigen::VectorXd::Zero(5);
  EXPECT_FALSE(filter.doSmoothing(position, velocity, acceleration));

  // fail because reset was not called
  position = Eigen::VectorXd::Zero(7);
  velocity = Eigen::VectorXd::Zero(7);
  acceleration = Eigen::VectorXd::Zero(7);
  EXPECT_FALSE(filter.doSmoothing(position, velocity, acceleration));

  // succeed
  filter.reset(Eigen::VectorXd::Zero(7), Eigen::VectorXd::Zero(7), Eigen::VectorXd::Zero(7));
  EXPECT_TRUE(filter.doSmoothing(position, velocity, acceleration));

  // succeed: apply acceleration limits
  filter.reset(Eigen::VectorXd::Zero(7), Eigen::VectorXd::Zero(7), Eigen::VectorXd::Zero(7));
  position.array() = 3.0;
  EXPECT_TRUE(filter.doSmoothing(position, velocity, acceleration));
  EXPECT_TRUE((position.array() - update_rate * update_rate * acceleration_limit.array()).matrix().norm() < 1E-3);

  // succeed: apply acceleration limits
  position.array() = 0.9;
  filter.reset(position * 0, velocity * 0, acceleration * 0);
  EXPECT_TRUE(filter.doSmoothing(position, velocity, acceleration));
  EXPECT_TRUE((position.array() - 0.9).matrix().norm() < 1E-3);

  // succeed: slow down
  velocity = 10 * Eigen::VectorXd::Ones(7);
  filter.reset(Eigen::VectorXd::Zero(7), velocity, Eigen::VectorXd::Zero(7));
  position.array() = 0.01;
  Eigen::VectorXd expected_offset =
      velocity.array() * update_rate - update_rate * update_rate * acceleration_limit.array();
  EXPECT_TRUE(filter.doSmoothing(position, velocity, acceleration));
  EXPECT_TRUE((velocity * update_rate - expected_offset).norm() < 1E-3);
}

TEST_F(AccelerationFilterTest, FilterBadAccelerationConfig)
{
  online_signal_smoothing::AccelerationLimitedPlugin filter;

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("AccelerationFilterTest");
  node->declare_parameter<std::string>("move_group_name", move_group_name_);
  double update_rate = 0.1;
  node->declare_parameter<double>("update_rate", update_rate);
  Eigen::VectorXd acceleration_limit = -1.0 * Eigen::VectorXd::Ones(7);
  set_limit(acceleration_limit);
  EXPECT_TRUE(filter.initialize(node, robot_model_, 7));
  Eigen::VectorXd position = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd velocity = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd acceleration = Eigen::VectorXd::Zero(7);
  EXPECT_TRUE(filter.reset(position, velocity, acceleration));
  EXPECT_FALSE(filter.doSmoothing(position, velocity, acceleration));
}

TEST_F(AccelerationFilterTest, FilterDoSmoothRandomized)
{
  double update_rate = 0.1;
  for (size_t iter = 0; iter < 64; ++iter)
  {
    online_signal_smoothing::AccelerationLimitedPlugin filter;

    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("AccelerationFilterTest");
    node->declare_parameter<std::string>("move_group_name", move_group_name_);
    node->declare_parameter<double>("update_rate", update_rate);
    Eigen::VectorXd acceleration_limit = 1.2 * (1.0 + Eigen::VectorXd::Random(7).array());
    set_limit(acceleration_limit);
    EXPECT_TRUE(filter.initialize(node, robot_model_, 7));
    filter.reset(Eigen::VectorXd::Zero(7), Eigen::VectorXd::Zero(7), Eigen::VectorXd::Zero(7));
    Eigen::VectorXd position = Eigen::VectorXd::Random(7);
    Eigen::VectorXd velocity = Eigen::VectorXd::Random(7);
    Eigen::VectorXd acceleration = Eigen::VectorXd::Random(7);
    EXPECT_TRUE(filter.doSmoothing(position, velocity, acceleration));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
