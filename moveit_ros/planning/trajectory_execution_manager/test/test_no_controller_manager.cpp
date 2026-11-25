/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, VBTI
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
 *   * Neither the name of VBTI nor the names of its
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

/* Author: Matthijs van der Burgh
   Desc: Test the TrajectoryExecutionManager configured without a controller manager
*/

#include <gtest/gtest.h>

#include <memory>

#include <moveit/trajectory_execution_manager/trajectory_execution_manager.hpp>
#include <moveit/utils/robot_model_test_utils.hpp>
#include <rclcpp/rclcpp.hpp>

TEST(TrajectoryExecutionManager, EmptyControllerManagerParameter)
{
  // GIVEN a node with an empty moveit_controller_manager parameter
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.parameter_overrides({ rclcpp::Parameter("moveit_controller_manager", "") });
  const auto node = rclcpp::Node::make_shared("test_no_controller_manager", options);
  moveit::core::RobotModelBuilder builder("one_joint_robot", "base_link");
  builder.addChain("base_link->link_1", "revolute");
  const moveit::core::RobotModelConstPtr robot_model = builder.build();
  ASSERT_NE(robot_model, nullptr);

  // WHEN the trajectory execution manager is constructed
  std::unique_ptr<trajectory_execution_manager::TrajectoryExecutionManager> manager;
  ASSERT_NO_THROW(
      manager = std::make_unique<trajectory_execution_manager::TrajectoryExecutionManager>(node, robot_model, nullptr));

  // THEN no controller manager and no controller manager node exist
  EXPECT_EQ(manager->getControllerManager(), nullptr);
  EXPECT_EQ(manager->getControllerManagerNode(), nullptr);
  // THEN no controller is available for the joint
  EXPECT_FALSE(manager->ensureActiveControllersForJoints({ "base_link-link_1-joint" }));
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
