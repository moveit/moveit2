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

constexpr std::string_view PLANNING_GROUP_NAME = "panda_arm";
constexpr size_t PANDA_NUM_JOINTS = 7u;
constexpr std::string_view ROBOT_MODEL = "panda";

class AccelerationFilterTest : public testing::Test
{
protected:
  void SetUp() override
  {
    robot_model_ = moveit::core::loadTestingRobotModel(ROBOT_MODEL.data());
  };

  void setLimits(std::optional<Eigen::VectorXd> acceleration_limits)
  {
    const std::vector<moveit::core::JointModel*> joint_models = robot_model_->getJointModels();
    auto joint_model_group = robot_model_->getJointModelGroup(PLANNING_GROUP_NAME.data());
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
        joint_bound.has_acceleration_limits = acceleration_limits.has_value();
        if (joint_bound.has_acceleration_limits)
        {
          joint_bound.max_acceleration = acceleration_limits.value()[ind];
        }
      }
      joint_model->setVariableBounds(joint_bounds_msg);
      ind++;
    }
  }

protected:
  moveit::core::RobotModelPtr robot_model_;
};

TEST_F(AccelerationFilterTest, FilterInitialize)
{
  online_signal_smoothing::AccelerationLimitedPlugin filter;
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("AccelerationFilterTest");

  // fail because the update_period parameter is not set
  EXPECT_THROW(filter.initialize(node, robot_model_, PANDA_NUM_JOINTS),
               rclcpp::exceptions::ParameterUninitializedException);

  node = std::make_shared<rclcpp::Node>("AccelerationFilterTest");
  node->declare_parameter<std::string>("planning_group_name", PLANNING_GROUP_NAME.data());
  node->declare_parameter("update_period", 0.01);

  // fail because the number of joints is wrong
  EXPECT_FALSE(filter.initialize(node, robot_model_, 3u));

  // fail because the robot does not have acceleration limits
  setLimits({});
  EXPECT_FALSE(filter.initialize(node, robot_model_, PANDA_NUM_JOINTS));

  // succeed with acceleration limits
  Eigen::VectorXd acceleration_limits = 1.2 * Eigen::VectorXd::Ones(PANDA_NUM_JOINTS);
  setLimits(acceleration_limits);
  EXPECT_TRUE(filter.initialize(node, robot_model_, PANDA_NUM_JOINTS));
}

TEST_F(AccelerationFilterTest, FilterDoSmooth)
{
  online_signal_smoothing::AccelerationLimitedPlugin filter;

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("AccelerationFilterTest");
  node->declare_parameter<std::string>("planning_group_name", PLANNING_GROUP_NAME.data());
  const double update_period = 1.0;
  node->declare_parameter<double>("update_period", update_period);
  Eigen::VectorXd acceleration_limits = 1.2 * Eigen::VectorXd::Ones(PANDA_NUM_JOINTS);
  setLimits(acceleration_limits);

  EXPECT_TRUE(filter.initialize(node, robot_model_, PANDA_NUM_JOINTS));

  // fail when called with the wrong number of joints
  Eigen::VectorXd position = Eigen::VectorXd::Zero(5);
  Eigen::VectorXd velocity = Eigen::VectorXd::Zero(5);
  Eigen::VectorXd acceleration = Eigen::VectorXd::Zero(5);
  EXPECT_FALSE(filter.doSmoothing(position, velocity, acceleration));

  // fail because reset was not called
  position = Eigen::VectorXd::Zero(PANDA_NUM_JOINTS);
  velocity = Eigen::VectorXd::Zero(PANDA_NUM_JOINTS);
  acceleration = Eigen::VectorXd::Zero(PANDA_NUM_JOINTS);
  EXPECT_FALSE(filter.doSmoothing(position, velocity, acceleration));

  // succeed
  filter.reset(Eigen::VectorXd::Zero(PANDA_NUM_JOINTS), Eigen::VectorXd::Zero(PANDA_NUM_JOINTS),
               Eigen::VectorXd::Zero(PANDA_NUM_JOINTS));
  EXPECT_TRUE(filter.doSmoothing(position, velocity, acceleration));

  // succeed: apply acceleration limits
  filter.reset(Eigen::VectorXd::Zero(PANDA_NUM_JOINTS), Eigen::VectorXd::Zero(PANDA_NUM_JOINTS),
               Eigen::VectorXd::Zero(PANDA_NUM_JOINTS));
  position.array() = 3.0;
  EXPECT_TRUE(filter.doSmoothing(position, velocity, acceleration));
  EXPECT_TRUE((position.array() - update_period * update_period * acceleration_limits.array()).matrix().norm() < 1E-3);

  // succeed: apply acceleration limits
  position.array() = 0.9;
  filter.reset(position * 0, velocity * 0, acceleration * 0);
  EXPECT_TRUE(filter.doSmoothing(position, velocity, acceleration));
  EXPECT_TRUE((position.array() - 0.9).matrix().norm() < 1E-3);

  // succeed: slow down
  velocity = 10 * Eigen::VectorXd::Ones(PANDA_NUM_JOINTS);
  filter.reset(Eigen::VectorXd::Zero(PANDA_NUM_JOINTS), velocity, Eigen::VectorXd::Zero(PANDA_NUM_JOINTS));
  position.array() = 0.01;
  Eigen::VectorXd expected_offset =
      velocity.array() * update_period - update_period * update_period * acceleration_limits.array();
  EXPECT_TRUE(filter.doSmoothing(position, velocity, acceleration));
  EXPECT_TRUE((velocity * update_period - expected_offset).norm() < 1E-3);
}

TEST_F(AccelerationFilterTest, FilterBadAccelerationConfig)
{
  online_signal_smoothing::AccelerationLimitedPlugin filter;

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("AccelerationFilterTest");
  node->declare_parameter<std::string>("planning_group_name", PLANNING_GROUP_NAME.data());
  const double update_period = 0.1;
  node->declare_parameter<double>("update_period", update_period);
  Eigen::VectorXd acceleration_limits = -1.0 * Eigen::VectorXd::Ones(PANDA_NUM_JOINTS);
  setLimits(acceleration_limits);
  EXPECT_TRUE(filter.initialize(node, robot_model_, PANDA_NUM_JOINTS));
  Eigen::VectorXd position = Eigen::VectorXd::Zero(PANDA_NUM_JOINTS);
  Eigen::VectorXd velocity = Eigen::VectorXd::Zero(PANDA_NUM_JOINTS);
  Eigen::VectorXd acceleration = Eigen::VectorXd::Zero(PANDA_NUM_JOINTS);
  EXPECT_TRUE(filter.reset(position, velocity, acceleration));
  EXPECT_FALSE(filter.doSmoothing(position, velocity, acceleration));
}

TEST_F(AccelerationFilterTest, FilterDoSmoothRandomized)
{
  online_signal_smoothing::AccelerationLimitedPlugin filter;
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("AccelerationFilterTest");
  const double update_period = 0.1;
  node->declare_parameter<std::string>("planning_group_name", PLANNING_GROUP_NAME.data());
  node->declare_parameter<double>("update_period", update_period);
  Eigen::VectorXd acceleration_limits = 1.2 * (1.0 + Eigen::VectorXd::Random(PANDA_NUM_JOINTS).array());
  setLimits(acceleration_limits);
  EXPECT_TRUE(filter.initialize(node, robot_model_, PANDA_NUM_JOINTS));

  for (size_t iter = 0; iter < 64; ++iter)
  {
    acceleration_limits = 1.2 * (1.0 + Eigen::VectorXd::Random(PANDA_NUM_JOINTS).array());
    setLimits(acceleration_limits);
    EXPECT_TRUE(filter.initialize(node, robot_model_, PANDA_NUM_JOINTS));
    filter.reset(Eigen::VectorXd::Zero(PANDA_NUM_JOINTS), Eigen::VectorXd::Zero(PANDA_NUM_JOINTS),
                 Eigen::VectorXd::Zero(PANDA_NUM_JOINTS));
    Eigen::VectorXd position = Eigen::VectorXd::Random(PANDA_NUM_JOINTS);
    Eigen::VectorXd velocity = Eigen::VectorXd::Random(PANDA_NUM_JOINTS);
    Eigen::VectorXd acceleration = Eigen::VectorXd::Random(PANDA_NUM_JOINTS);
    EXPECT_TRUE(filter.doSmoothing(position, velocity, acceleration));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
