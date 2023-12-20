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

/*      Title       : datatypes.hpp
 *      Project     : moveit_servo
 *      Created     : 06/05/2023
 *      Author      : Andy Zelenak, V Mohammed Ibrahim
 *
 *      Description : The custom datatypes used by Moveit Servo.
 */

#pragma once

#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <unordered_map>

namespace moveit_servo
{

enum class StatusCode : int8_t
{
  INVALID = -1,
  NO_WARNING = 0,
  DECELERATE_FOR_APPROACHING_SINGULARITY = 1,
  HALT_FOR_SINGULARITY = 2,
  DECELERATE_FOR_LEAVING_SINGULARITY = 3,
  DECELERATE_FOR_COLLISION = 4,
  HALT_FOR_COLLISION = 5,
  JOINT_BOUND = 6
};

const std::unordered_map<StatusCode, std::string> SERVO_STATUS_CODE_MAP(
    { { StatusCode::INVALID, "Invalid" },
      { StatusCode::NO_WARNING, "No warnings" },
      { StatusCode::DECELERATE_FOR_APPROACHING_SINGULARITY, "Moving closer to a singularity, decelerating" },
      { StatusCode::HALT_FOR_SINGULARITY, "Very close to a singularity, emergency stop" },
      { StatusCode::DECELERATE_FOR_LEAVING_SINGULARITY, "Moving away from a singularity, decelerating" },
      { StatusCode::DECELERATE_FOR_COLLISION, "Close to a collision, decelerating" },
      { StatusCode::HALT_FOR_COLLISION, "Collision detected, emergency stop" },
      { StatusCode::JOINT_BOUND, "Close to a joint bound (position or velocity), halting" } });

// The datatype that specifies the type of command that servo should expect.
enum class CommandType : int8_t
{
  JOINT_JOG = 0,
  TWIST = 1,
  POSE = 2,

  // Range of allowed values used for validation.
  MIN = JOINT_JOG,
  MAX = POSE
};

typedef std::pair<StatusCode, Eigen::VectorXd> JointDeltaResult;

// The joint jog command, this will be vector of length equal to the number of joints of the robot.
struct JointJogCommand
{
  std::vector<std::string> names;
  std::vector<double> velocities;
};

// The twist command,  frame_id is the name of the frame in which the command is specified in.
// frame_id must always be specified.
struct TwistCommand
{
  std::string frame_id;
  Eigen::Vector<double, 6> velocities;
};

// The Pose command,  frame_id is the name of the frame in which the command is specified in.
// frame_id must always be specified.
struct PoseCommand
{
  std::string frame_id;
  Eigen::Isometry3d pose;
};

// The generic input type for servo that can be JointJog, Twist or Pose.
typedef std::variant<JointJogCommand, TwistCommand, PoseCommand> ServoInput;

// The output datatype of servo, this structure contains the names of the joints along with their positions, velocities and accelerations.
struct KinematicState
{
  std::vector<std::string> joint_names;
  Eigen::VectorXd positions, velocities, accelerations;
  rclcpp::Time time_stamp;

  KinematicState(const int num_joints)
  {
    joint_names.resize(num_joints);
    positions = Eigen::VectorXd::Zero(num_joints);
    velocities = Eigen::VectorXd::Zero(num_joints);
    accelerations = Eigen::VectorXd::Zero(num_joints);
  }

  KinematicState()
  {
  }
};

// Mapping joint names and their position in the move group vector
typedef std::unordered_map<std::string, std::size_t> JointNameToMoveGroupIndexMap;

}  // namespace moveit_servo
