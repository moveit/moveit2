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

/*      Title       : utils.hpp
 *      Project     : moveit_servo
 *      Created     : 05/17/2023
 *      Author      : Andy Zelenak, V Mohammed Ibrahim
 *
 *      Description : The utility functions used by MoveIt Servo.
 */

#pragma once

#include <moveit_servo_lib_parameters.hpp>
#include <moveit_servo/utils/datatypes.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace moveit_servo
{

/**
 * \brief Checks if a given VectorXd is a valid command.
 * @param command The command to be checked.
 * @return True if the command is valid, else False.
 */
bool isValidCommand(const Eigen::VectorXd& command);

/**
 * \brief Checks if a given Isometry3d (pose) is a valid command.
 * @param command The command to be checked.
 * @return True if the command is valid, else False.
 */
bool isValidCommand(const Eigen::Isometry3d& command);

/**
 * \brief Checks if a given Twist command is valid.
 * @param command The command to be checked.
 * @return True if the command is valid, else False.
 */
bool isValidCommand(const TwistCommand& command);

/**
 * \brief Checks if a given Pose command is valid.
 * @param command The command to be checked.
 * @return True if the command is valid, else False.
 */
bool isValidCommand(const PoseCommand& command);

/**
 * \brief Create a pose message for the provided change in Cartesian position.
 * @param delta_x The change in Cartesian position.
 * @param base_to_tip_frame_transform The transformation from robot base to ee frame.
 * @return The pose message.
 */
geometry_msgs::msg::Pose poseFromCartesianDelta(const Eigen::VectorXd& delta_x,
                                                const Eigen::Isometry3d& base_to_tip_frame_transform);

/**
 * \brief Create a trajectory message from given joint state.
 * @param servo_params The configuration used by servo, required for setting some field of the trajectory message.
 * @param joint_state The joint state to be added into the trajectory.
 * @return The trajectory message.
 */
trajectory_msgs::msg::JointTrajectory composeTrajectoryMessage(const servo::Params& servo_params,
                                                               const KinematicState& joint_state);

/**
 * \brief Create a Float64MultiArray message from given joint state
 * @param servo_params The configuration used by servo, required for selecting position vs velocity.
 * @param joint_state The joint state to be added into the Float64MultiArray.
 * @return The Float64MultiArray message.
 */
std_msgs::msg::Float64MultiArray composeMultiArrayMessage(const servo::Params& servo_params,
                                                          const KinematicState& joint_state);

/**
 * \brief Computes scaling factor for velocity when the robot is near a singularity.
 * @param joint_model_group The joint model group of the robot, used for fetching the Jacobian.
 * @param current_state The current state of the robot, used for singularity look ahead.
 * @param target_delta_x The vector containing the required change in Cartesian position.
 * @param servo_params The servo parameters, contains the singularity thresholds.
 * @return The velocity scaling factor and the reason for scaling.
 */
std::pair<double, StatusCode> velocityScalingFactorForSingularity(const moveit::core::RobotStatePtr& robot_state,
                                                                  const Eigen::VectorXd& target_delta_x,
                                                                  const servo::Params& servo_params);

/**
 * \brief Apply velocity scaling based on joint limits.
 * @param velocities The commanded velocities.
 * @param joint_bounds The bounding information for the robot joints.
 * @param scaling_override The user defined velocity scaling override.
 * @return The velocity scaling factor.
 */
double jointLimitVelocityScalingFactor(const Eigen::VectorXd& velocities,
                                       const moveit::core::JointBoundsVector& joint_bounds, double scaling_override);

/**
 * \brief Finds the joints that are exceeding allowable position limits.
 * @param positions The joint positions.
 * @param velocities The current commanded velocities.
 * @param joint_bounds The allowable limits for the robot joints.
 * @param margin Additional buffer on the actual joint limits.
 * @return The joints that are violating the specified position limits.
 */
std::vector<int> jointsToHalt(const Eigen::VectorXd& positions, const Eigen::VectorXd& velocities,
                              const moveit::core::JointBoundsVector& joint_bounds, double margin);

/**
 * \brief Helper function for converting Eigen::Isometry3d to geometry_msgs/TransformStamped.
 * @param eigen_tf The isometry to be converted to TransformStamped.
 * @param parent_frame The target frame.
 * @param child_frame The current frame.
 * @return The isometry as a TransformStamped message.
 */
geometry_msgs::msg::TransformStamped convertIsometryToTransform(const Eigen::Isometry3d& eigen_tf,
                                                                const std::string& parent_frame,
                                                                const std::string& child_frame);

/**
 * \brief Convert a PoseStamped message to a Servo Pose
 * @param msg The PoseStamped message.
 * @return The equivalent Servo Pose type.
 */
PoseCommand poseFromPoseStamped(const geometry_msgs::msg::PoseStamped& msg);

/**
 * \brief Creates the planning scene monitor used by servo
 */
planning_scene_monitor::PlanningSceneMonitorPtr createPlanningSceneMonitor(const rclcpp::Node::SharedPtr& node,
                                                                           const servo::Params& servo_params);
}  // namespace moveit_servo
