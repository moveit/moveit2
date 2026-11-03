/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Nanjing University
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
 *   * Neither the name of Nanjing University nor the names of its
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
#include <rclcpp/rclcpp.hpp>

#include <moveit/trajectory_execution_manager/trajectory_execution_manager.hpp>
#include <moveit/utils/robot_model_test_utils.hpp>

TEST(TrajectoryExecutionManager, PreservesMultiDofDerivativesByJointName)
{
  moveit::core::RobotModelBuilder builder("floating_robot", "world");
  builder.addChain("world->base", "floating");
  builder.addChain("base->tool", "floating");
  const auto robot_model = builder.build();

  const std::vector<rclcpp::Parameter> parameters{ { "moveit_controller_manager",
                                                     "trajectory_execution_manager_test/ControllerManager" } };
  const auto options =
      rclcpp::NodeOptions().parameter_overrides(parameters).automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("trajectory_distribution_test", options);
  trajectory_execution_manager::TrajectoryExecutionManager manager(node, robot_model, nullptr, false);

  moveit_msgs::msg::RobotTrajectory trajectory;
  trajectory.multi_dof_joint_trajectory.joint_names = { "world-base-joint", "base-tool-joint" };
  trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
  point.transforms.resize(2);
  point.transforms[0].translation.x = 10.0;
  point.transforms[1].translation.x = 20.0;
  point.transforms[0].rotation.w = 1.0;
  point.transforms[1].rotation.w = 1.0;
  point.velocities.resize(2);
  point.velocities[0].linear.x = 1.0;
  point.velocities[1].linear.x = 2.0;
  point.accelerations.resize(2);
  point.accelerations[0].linear.x = 3.0;
  point.accelerations[1].linear.x = 4.0;
  trajectory.multi_dof_joint_trajectory.points = { point };

  ASSERT_TRUE(manager.push(trajectory, std::vector<std::string>{ "test_controller" }));
  ASSERT_EQ(manager.getTrajectories().size(), 1u);
  ASSERT_EQ(manager.getTrajectories()[0]->trajectory_parts_.size(), 1u);
  const auto& output = manager.getTrajectories()[0]->trajectory_parts_[0].multi_dof_joint_trajectory;
  ASSERT_EQ(output.joint_names, (std::vector<std::string>{ "base-tool-joint", "world-base-joint" }));
  ASSERT_EQ(output.points.size(), 1u);
  ASSERT_EQ(output.points[0].transforms.size(), 2u);
  ASSERT_EQ(output.points[0].velocities.size(), 2u);
  ASSERT_EQ(output.points[0].accelerations.size(), 2u);
  EXPECT_DOUBLE_EQ(output.points[0].transforms[0].translation.x, 20.0);
  EXPECT_DOUBLE_EQ(output.points[0].transforms[1].translation.x, 10.0);
  EXPECT_DOUBLE_EQ(output.points[0].velocities[0].linear.x, 2.0);
  EXPECT_DOUBLE_EQ(output.points[0].velocities[1].linear.x, 1.0);
  EXPECT_DOUBLE_EQ(output.points[0].accelerations[0].linear.x, 4.0);
  EXPECT_DOUBLE_EQ(output.points[0].accelerations[1].linear.x, 3.0);

  auto malformed = trajectory;
  malformed.multi_dof_joint_trajectory.points[0].velocities.pop_back();
  EXPECT_FALSE(manager.push(malformed, std::vector<std::string>{ "test_controller" }));
  malformed = trajectory;
  malformed.multi_dof_joint_trajectory.points[0].accelerations.pop_back();
  EXPECT_FALSE(manager.push(malformed, std::vector<std::string>{ "test_controller" }));
  malformed = trajectory;
  malformed.multi_dof_joint_trajectory.points[0].transforms.pop_back();
  EXPECT_FALSE(manager.push(malformed, std::vector<std::string>{ "test_controller" }));
  EXPECT_EQ(manager.getTrajectories().size(), 1u);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
