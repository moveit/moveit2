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
 *      Created     : 05/06/2023
 *      Author      : V Mohammed Ibrahim
 *
 *      Description : The custom datatypes used by Moveit Servo.
 */

#pragma once

namespace moveit_servo
{
// Datatypes used by servo

typedef Eigen::VectorXd JointJog;

struct Pose
{
  std::string frame_id;
  Eigen::Isometry3d pose;
};

struct Twist
{
  std::string frame_id;
  Eigen::Vector<double, 6> velocities;
};

typedef std::variant<JointJog, Twist, Pose> ServoInput;

struct KinematicState
{
  std::vector<std::string> joint_names;
  std::vector<double> positions, velocities, accelerations;

  KinematicState(const int num_joints)
  {
    joint_names.resize(num_joints);
    positions.resize(num_joints);
    velocities.resize(num_joints);
    accelerations.resize(num_joints);
  }
};

enum class CommandType
{
  JOINT_JOG = 0,
  TWIST = 1,
  POSE = 2
};
}  // namespace moveit_servo
