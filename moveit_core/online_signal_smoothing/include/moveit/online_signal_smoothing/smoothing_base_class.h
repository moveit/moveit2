/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik Inc.
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

/* Author: Andy Zelenak
   Description: Defines a pluginlib interface for smoothing algorithms.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/macros/class_forward.h>

#include <moveit_smoothing_base_export.h>

namespace moveit
{
namespace core
{
MOVEIT_CLASS_FORWARD(RobotModel);
}  // namespace core
}  // namespace moveit

namespace online_signal_smoothing
{
class SmoothingBaseClass
{
public:
  SmoothingBaseClass();
  virtual ~SmoothingBaseClass();

  /**
   * Initialize the smoothing algorithm
   * @param node ROS node, typically used for parameter retrieval
   * @param robot_model typically used to retrieve vel/accel/jerk limits
   * @param num_joints number of actuated joints in the JointGroup Servo controls
   * @return True if initialization was successful
   */
  virtual bool initialize(rclcpp::Node::SharedPtr node, moveit::core::RobotModelConstPtr robot_model,
                          size_t num_joints) = 0;

  /**
   * Smooth an array of joint position deltas
   * @param position_vector array of joint position commands
   * @return True if initialization was successful
   */
  virtual bool doSmoothing(std::vector<double>& position_vector) = 0;

  /**
   * Reset to a given joint state
   * @param joint_positions reset the filters to these joint positions
   * @return True if reset was successful
   */
  virtual bool reset(const std::vector<double>& joint_positions) = 0;
};
}  // namespace online_signal_smoothing
