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

/*      Title       : command.hpp
 *      Project     : moveit_servo
 *      Created     : 06/04/2023
 *      Author      : Brian O'Neil, Andy Zelenak, Blake Anderson, V Mohammed Ibrahim
 *
 *      Description : The methods that compute the required change in joint angles for various input types.
 */

#pragma once

#include <moveit_servo/utils/common.hpp>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace moveit_servo
{

/**
 * \brief Compute the change in joint position for the given joint jog command.
 * @param command The joint jog command.
 * @param robot_state_ The current robot state as obtained from PlanningSceneMonitor.
 * @param servo_params The servo parameters.
 * @param joint_name_group_index_map Mapping between joint subgroup name and move group joint vector position.
 * @return The status and joint position change required (delta).
 */
JointDeltaResult jointDeltaFromJointJog(const JointJogCommand& command, const moveit::core::RobotStatePtr& robot_state,
                                        const servo::Params& servo_params,
                                        const JointNameToMoveGroupIndexMap& joint_name_group_index_map);

/**
 * \brief Compute the change in joint position for the given twist command.
 * @param command The twist command.
 * @param robot_state_ The current robot state as obtained from PlanningSceneMonitor.
 * @param servo_params The servo parameters.
 * @param planning_frame The planning frame name.
 * @param joint_name_group_index_map Mapping between joint subgroup name and move group joint vector position.
 * @return The status and joint position change required (delta).
 */
JointDeltaResult jointDeltaFromTwist(const TwistCommand& command, const moveit::core::RobotStatePtr& robot_state,
                                     const servo::Params& servo_params, const std::string& planning_frame,
                                     const JointNameToMoveGroupIndexMap& joint_name_group_index_map);

/**
 * \brief Compute the change in joint position for the given pose command.
 * @param command The pose command.
 * @param robot_state_ The current robot state as obtained from PlanningSceneMonitor.
 * @param servo_params The servo parameters.
 * @param planning_frame The planning frame name.
 * @param ee_frame The end effector frame name.
 * @param joint_name_group_index_map Mapping between sub group joint name and move group joint vector position
 * @return The status and joint position change required (delta).
 */
JointDeltaResult jointDeltaFromPose(const PoseCommand& command, const moveit::core::RobotStatePtr& robot_state,
                                    const servo::Params& servo_params, const std::string& planning_frame,
                                    const std::string& ee_frame,
                                    const JointNameToMoveGroupIndexMap& joint_name_group_index_map);

/**
 * \brief Computes the required change in joint angles for given Cartesian change, using the robot's IK solver.
 * @param cartesian_position_delta The change in Cartesian position.
 * @param robot_state_ The current robot state as obtained from PlanningSceneMonitor.
 * @param servo_params The servo parameters.
 * @param joint_name_group_index_map Mapping between joint subgroup name and move group joint vector position.
 * @return The status and joint position change required (delta).
 */
JointDeltaResult jointDeltaFromIK(const Eigen::VectorXd& cartesian_position_delta,
                                  const moveit::core::RobotStatePtr& robot_state, const servo::Params& servo_params,
                                  const JointNameToMoveGroupIndexMap& joint_name_group_index_map);

}  // namespace moveit_servo
