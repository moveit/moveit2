/*******************************************************************************
 *      Title     : enforce_limits.h
 *      Project   : moveit_servo
 *      Created   : 7/5/2021
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson, Tyler Weaver
 *
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

#pragma once

#include <Eigen/Core>
#include <sensor_msgs/msg/joint_state.hpp>

#include <moveit/robot_model/joint_model_group.h>

namespace moveit_servo
{
/**
 * @brief      Scale the delta theta to match joint velocity/acceleration limits
 *
 * @param[in]  joint_model_group   The joint model group
 * @param[in]  publish_period      The publish period of servo (used for calculating joint velocity)
 * @param[in]  delta_theta         The delta theta input
 *
 * @return     The delta theta output
 */
Eigen::ArrayXd enforceVelocityLimits(const moveit::core::JointModelGroup* joint_model_group, double publish_period,
                                     const Eigen::ArrayXd& delta_theta);

}  // namespace moveit_servo
