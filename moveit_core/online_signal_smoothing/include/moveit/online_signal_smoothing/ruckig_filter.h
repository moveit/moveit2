/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Andrew Zelenak
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
Description: Applies jerk/acceleration/velocity limits to online motion commands
 */

#pragma once

#include <cstddef>

#include <moveit/robot_model/robot_model.h>
#include <moveit/online_signal_smoothing/smoothing_base_class.h>
#include <moveit_ruckig_filter_parameters.hpp>

#include <ruckig/ruckig.hpp>

namespace online_signal_smoothing
{

class RuckigFilterPlugin : public SmoothingBaseClass
{
public:
  /**
   * Initialize the smoothing algorithm
   * @param node ROS node, used for parameter retrieval
   * @param robot_model used to retrieve vel/accel/jerk limits
   * @param num_joints number of actuated joints in the JointGroup
   * @return True if initialization was successful
   */
  bool initialize(rclcpp::Node::SharedPtr node, moveit::core::RobotModelConstPtr robot_model,
                  size_t num_joints) override;

  /**
   * Smooth the command signals for all DOF
   * @param positions array of joint position commands
   * @param velocities array of joint velocity commands
   * @param accelerations array of joint acceleration commands
   * @return True if initialization was successful
   */
  bool doSmoothing(Eigen::VectorXd& positions, Eigen::VectorXd& velocities, Eigen::VectorXd& accelerations) override;

  /**
   * Reset to a given joint state
   * @param positions reset the filters to these joint positions
   * @param velocities (unused)
   * @param accelerations (unused)
   * @return True if reset was successful
   */
  bool reset(const Eigen::VectorXd& positions, const Eigen::VectorXd& velocities,
             const Eigen::VectorXd& accelerations) override;

private:
  /**
   * A utility to print Ruckig's internal state
   */
  void printRuckigState();

  /**
   * A utility to get velocity/acceleration/jerk bounds from the robot model
   * @return true if all bounds are defined
   */
  bool getVelAccelJerkBounds(std::vector<double>& joint_velocity_bounds, std::vector<double>& joint_acceleration_bounds,
                             std::vector<double>& joint_jerk_bounds);

  /** \brief Parameters loaded from yaml file at runtime */
  online_signal_smoothing::Params params_;
  /** \brief The robot model contains the vel/accel/jerk limits that Ruckig requires */
  moveit::core::RobotModelConstPtr robot_model_;
  bool have_initial_ruckig_output_ = false;
  std::optional<ruckig::Ruckig<ruckig::DynamicDOFs>> ruckig_;
  std::optional<ruckig::InputParameter<ruckig::DynamicDOFs>> ruckig_input_;
  std::optional<ruckig::OutputParameter<ruckig::DynamicDOFs>> ruckig_output_;
};
}  // namespace online_signal_smoothing
