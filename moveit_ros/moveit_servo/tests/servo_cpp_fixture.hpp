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
    // Wait for complete state update before starting MoveIt Servo.
    if (!planning_scene_monitor_->getStateMonitor()->waitForCompleteState("panda_arm", 1.0))
    {
      FAIL() << "Could not retrieve complete robot state";
    }
    // Forward state update to planning scene
    planning_scene_monitor_->updateSceneWithCurrentState();

    servo_test_instance_ =
        std::make_shared<moveit_servo::Servo>(servo_test_node_, servo_param_listener_, planning_scene_monitor_);
  }

  /// Helper function to get the current pose of a specified frame.
  Eigen::Isometry3d getCurrentPose(const std::string& target_frame) const
  {
    planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor_);
    return locked_scene->getCurrentState().getGlobalLinkTransform(target_frame);
  }

  std::shared_ptr<rclcpp::Node> servo_test_node_;
  std::shared_ptr<const servo::ParamListener> servo_param_listener_;
  servo::Params servo_params_;
  std::shared_ptr<moveit_servo::Servo> servo_test_instance_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
};
