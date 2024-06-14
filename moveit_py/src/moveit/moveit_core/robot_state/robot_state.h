/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Peter David Fagan
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

/* Author: Peter David Fagan */

#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#pragma GCC diagnostic push
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif
#include <pybind11/eigen.h>
#pragma GCC diagnostic pop
#include <moveit_py/moveit_py_utils/copy_ros_msg.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit/robot_state/robot_state.h>

namespace py = pybind11;

namespace moveit_py
{
namespace bind_robot_state
{
void update(moveit::core::RobotState* self, bool force, std::string& category);

Eigen::MatrixXd getFrameTransform(const moveit::core::RobotState* self, std::string& frame_id);

Eigen::MatrixXd getGlobalLinkTransform(const moveit::core::RobotState* self, std::string& link_name);

geometry_msgs::msg::Pose getPose(const moveit::core::RobotState* self, const std::string& link_name);

Eigen::VectorXd copyJointGroupPositions(const moveit::core::RobotState* self, const std::string& joint_model_group_name);
Eigen::VectorXd copyJointGroupVelocities(const moveit::core::RobotState* self,
                                         const std::string& joint_model_group_name);
Eigen::VectorXd copyJointGroupAccelerations(const moveit::core::RobotState* self,
                                            const std::string& joint_model_group_name);

Eigen::MatrixXd getJacobian(const moveit::core::RobotState* self, const std::string& joint_model_group_name,
                            const std::string& link_model_name, const Eigen::Vector3d& reference_point_position,
                            bool use_quaternion_representation);

Eigen::MatrixXd getJacobian(const moveit::core::RobotState* self, const std::string& joint_model_group_name,
                            const Eigen::Vector3d& reference_point_position);

bool setToDefaultValues(moveit::core::RobotState* self, const std::string& joint_model_group_name,
                        const std::string& state_name);

void initRobotState(py::module& m);
}  // namespace bind_robot_state
}  // namespace moveit_py
