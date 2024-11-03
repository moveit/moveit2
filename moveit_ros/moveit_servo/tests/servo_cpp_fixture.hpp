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

/*      Title       : servo_cpp_fixture.hpp
 *      Project     : moveit_servo
 *      Created     : 07/13/2020
 *      Author      : Adam Pettinger, V Mohammed Ibrahim
 *
 *      Description : Resources used by servo c++ integration tests
 */

#include <gtest/gtest.h>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/command.hpp>
#include <moveit_servo/utils/common.hpp>
#include <moveit_servo/utils/datatypes.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

class ServoCppFixture : public testing::Test
{
protected:
  void SetUp() override
  {
    // Create a node to be given to Servo.
    servo_test_node_ = std::make_shared<rclcpp::Node>("moveit_servo_test");

    // Create a Servo object for testing.
    const std::string servo_param_namespace = "moveit_servo_test";
    servo_param_listener_ = std::make_shared<servo::ParamListener>(servo_test_node_, servo_param_namespace);
    servo_params_ = servo_param_listener_->get_params();

    planning_scene_monitor_ = moveit_servo::createPlanningSceneMonitor(servo_test_node_, servo_params_);

    // Wait until the joint configuration is nonzero before starting MoveIt Servo.
    int num_tries = 0;
    const int max_tries = 20;
    while (true)
    {
      const auto q = getCurrentJointPositions("panda_arm");
      if (q.norm() > 0.0)
      {
        break;
      }
      if (num_tries > max_tries)
      {
        FAIL() << "Robot joint configuration did not reach expected state after some time. Test is flaky.";
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      num_tries++;
    }

    servo_test_instance_ =
        std::make_shared<moveit_servo::Servo>(servo_test_node_, servo_param_listener_, planning_scene_monitor_);
  }

  /// Helper function to get the current pose of a specified frame.
  Eigen::Isometry3d getCurrentPose(const std::string& target_frame) const
  {
    return planning_scene_monitor_->getPlanningScene()->getCurrentState().getGlobalLinkTransform(target_frame);
  }

  /// Helper function to get the joint configuration of a group.
  Eigen::VectorXd getCurrentJointPositions(const std::string& group_name) const
  {
    std::vector<double> joint_positions;
    const auto robot_state = planning_scene_monitor_->getPlanningScene()->getCurrentState();
    robot_state.copyJointGroupPositions(group_name, joint_positions);
    return Eigen::Map<Eigen::VectorXd>(joint_positions.data(), joint_positions.size());
  }

  std::shared_ptr<rclcpp::Node> servo_test_node_;
  std::shared_ptr<const servo::ParamListener> servo_param_listener_;
  servo::Params servo_params_;
  std::shared_ptr<moveit_servo::Servo> servo_test_instance_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
};
