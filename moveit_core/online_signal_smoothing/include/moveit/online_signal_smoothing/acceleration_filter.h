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

/* Author: Paul Gesel
   Description: Applies smoothing by limiting the  acceleration between consecutive commands
 */

#pragma once

#include <cstddef>

// Auto-generated
#include <moveit_acceleration_parameters.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/online_signal_smoothing/smoothing_base_class.h>

namespace online_signal_smoothing
{

// Plugin
class AccelerationLimitedPlugin : public SmoothingBaseClass
{
public:
  /**
   * Initialize the acceleration based smoothing plugin
   * @param node ROS node, used for parameter retrieval
   * @param robot_model typically used to retrieve vel/accel/jerk limits
   * @param num_joints number of actuated joints in the JointGroup Servo controls
   * @return True if initialization was successful
   */
  bool initialize(rclcpp::Node::SharedPtr node, moveit::core::RobotModelConstPtr robot_model,
                  size_t num_joints) override;

  /**
   * Smooth the command signals for all DOF. This function estimates the velocity and acceleration using the two
   * previous positions passed in. It is designed to be called in a control loop at a constant interval.
   * @param positions array of joint position commands
   * @param velocities (unused)
   * @param accelerations (unused)
   * @return True if smoothing was successful
   */
  bool doSmoothing(Eigen::VectorXd& positions, Eigen::VectorXd& velocities, Eigen::VectorXd& accelerations) override;

  /**
   * Reset to a given joint state. This method must be called before doSmoothing.
   * @param positions reset the filters to these joint positions
   * @param velocities (unused)
   * @param accelerations (unused)
   * @return True if reset was successful
   */
  bool reset(const Eigen::VectorXd& positions, const Eigen::VectorXd& velocities,
             const Eigen::VectorXd& accelerations) override;

private:
  rclcpp::Node::SharedPtr node_;
  long num_joints_;
  std::array<std::pair<Eigen::VectorXd, std::optional<rclcpp::Time>>, 2> last_positions_;
  Eigen::VectorXd prev_velocity_;
  Eigen::VectorXd cur_velocity_;
  Eigen::VectorXd cur_acceleration;
  double joint_acceleration_limit_;
  double update_timeout_;
};
}  // namespace online_signal_smoothing
