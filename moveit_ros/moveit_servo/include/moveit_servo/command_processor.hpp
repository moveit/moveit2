/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*      Title       : command_processor.hpp
 *      Project     : moveit_servo
 *      Created     : 06/04/2023
 *      Author      : Brian O'Neil, Andy Zelenak, Blake Anderson, V Mohammed Ibrahim
 *
 *      Description : The methods that compute the required change in joint angles for various input types.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_servo/datatypes.hpp>
#include <moveit_servo/status_codes.hpp>
#include <moveit_servo/utils.hpp>

namespace moveit_servo
{

class CommandProcessor
{
public:
  CommandProcessor(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                   const moveit::core::JointModelGroup* joint_model_group, moveit::core::RobotStatePtr& current_state,
                   servo::Params& servo_params, StatusCode& servo_status);

  /**
   * \brief Compute the change in joint position for the given joint jog command.
   * @param command The joint jog command.
   * @return The joint position change required (delta).
   */
  Eigen::VectorXd jointDeltaFromCommand(const JointJog& command);

  /**
   * \brief Compute the change in joint position for the given twist command.
   * @param command The twist command.
   * @return The joint position change required (delta).
   */
  Eigen::VectorXd jointDeltaFromCommand(const Twist& command);

  /**
   * \brief Compute the change in joint position for the given pose command.
   * @param command The pose command.
   * @return The joint position change required (delta).
   */
  Eigen::VectorXd jointDeltaFromCommand(const Pose& command);

  /**
   * \brief Returns the end effector pose in planning frame
   */
  const Eigen::Isometry3d getEndEffectorPose();

private:
  /**
   * \brief Set the IK solver that servo will use. If the robot does not have one, inverse Jacobian will be used instead.
   */
  void setIKSolver();

  /**
   * \brief Get the current status of servo.
   * @return The current status.
   */
  StatusCode getStatus();

  /**
   * \brief Get the message associated with the current servo status.
   * @return The status message.
   */
  const std::string getStatusMessage();

  /**
   * \brief Computes the required change in joint angles for given cartesian change, using the robots IK solver.
   * @param carteisan_position_delta The change in cartesian position.
   * @return The required joint angle deltas.
   */
  Eigen::VectorXd jointDeltaFromIK(const Eigen::VectorXd& cartesian_position_delta);

  // Variables
  size_t num_joints_;
  const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor_;
  const moveit::core::JointModelGroup* joint_model_group_;
  moveit::core::RobotStatePtr& robot_state_;
  servo::Params& servo_params_;
  StatusCode& servo_status_;

  kinematics::KinematicsBaseConstPtr ik_solver_ = nullptr;
};

}  // namespace moveit_servo
