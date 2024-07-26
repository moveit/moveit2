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
   Description: A first-order Butterworth low-pass filter. There is only one parameter to tune.
   The first-order Butterworth filter has the nice property that it will not overshoot.
 */

#pragma once

#include <cstddef>

#include <moveit_butterworth_filter_parameters.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/online_signal_smoothing/smoothing_base_class.h>

namespace online_signal_smoothing
{
/**
 * Class ButterworthFilter - Implementation of a signal filter to soften jerks.
 * This is a first-order Butterworth low-pass filter. First-order was chosen for 2 reasons:
 * - It doesn't overshoot
 * - Computational efficiency
 * This filter has been parameterized so there is only one parameter to tune.
 * See "Digital Implementation of Butterworth First–Order Filter Type IIR" by
 * Horvath, Cervenanska, and Kotianova, 2019 and
 * Mienkina, M., Filter-Based Algorithm for Metering Applications,
 * https://www.nxp.com/docs/en/application-note/AN4265.pdf, 2016
 * It comes from finding the bilinear transform equivalent of the analog transfer function and
 * further applying the inverse z-transform.
 * The parameter "low_pass_filter_coeff" equals (2*pi / tan(omega_d * T))
 * where omega_d is the cutoff frequency and T is the sampling period in sec.
 */
class ButterworthFilter
{
public:
  /**
   * Constructor.
   * @param low_pass_filter_coeff Larger filter_coeff-> more smoothing of commands, but more lag.
   * low_pass_filter_coeff = (2*pi / tan(omega_d * T))
   * where omega_d is the cutoff frequency and T is the sampling period in sec.
   */
  ButterworthFilter(double low_pass_filter_coeff);
  ButterworthFilter() = delete;

  double filter(double new_measurement);

  void reset(const double data);

private:
  static constexpr std::size_t FILTER_LENGTH = 2;
  std::array<double, FILTER_LENGTH> previous_measurements_;
  double previous_filtered_measurement_;
  // Scale and feedback term are calculated from supplied filter coefficient
  double scale_term_;
  double feedback_term_;
};

// Plugin
class ButterworthFilterPlugin : public SmoothingBaseClass
{
public:
  /**
   * Initialize the smoothing algorithm
   * @param node ROS node, used for parameter retrieval
   * @param robot_model typically used to retrieve vel/accel/jerk limits
   * @param num_joints number of actuated joints in the JointGroup Servo controls
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
  rclcpp::Node::SharedPtr node_;
  std::vector<ButterworthFilter> position_filters_;
  size_t num_joints_;
};
}  // namespace online_signal_smoothing
