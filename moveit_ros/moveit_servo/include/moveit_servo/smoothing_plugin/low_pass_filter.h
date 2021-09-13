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
 */

#pragma once

#include <cstddef>

#include <moveit/robot_model/robot_model.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/smoothing_plugin/smoothing_base_class.h>

namespace moveit_servo
{
/**
 * Class LowpassFilterImpl - Implementation of a signal filter to soften jerks.
 * This is a first-order Butterworth low-pass filter. First-order was chosen for 2 reasons:
 * - It doesn't overshoot
 * - Computational efficiency
 */
class LowpassFilterImpl
{
public:
  /**
   * Constructor.
   * @param low_pass_filter_coeff Larger filter_coeff-> more smoothing of servo commands, but more lag.
   * Rough plot, with cutoff frequency on the y-axis:
   * https://www.wolframalpha.com/input/?i=plot+arccot(c)
   */
  LowpassFilterImpl(double low_pass_filter_coeff);

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
class LowPassFilter : public SmoothingBaseClass
{
public:
  LowPassFilter(){};

  /**
   * Initialize the smoothing algorithm
   * @param node ROS node, used for parameter retrieval
   * @param robot_model typically used to retrieve vel/accel/jerk limits
   * @param num_joints number of actuated joints in the JointGroup Servo controls
   * @param parameters access to all Servo parameters that have been parsed from yaml
   * @return True if initialization was successful
   */
  bool initialize(rclcpp::Node::SharedPtr node, moveit::core::RobotModelConstPtr robot_model, size_t num_joints,
                  const std::shared_ptr<const moveit_servo::ServoParameters>& parameters) override;

  /**
   * Smooth the command signals for all DOF
   * @param delta_theta array of incremental joint position commands
   * @return True if initialization was successful
   */
  bool doSmoothing(std::vector<double>& position_vector) override;

  /**
   * Reset to a given joint state
   * @param joint_positions reset the filters to these joint positions
   * @return True if reset was successful
   */
  bool reset(const std::vector<double>& joint_positions) override;

private:
  rclcpp::Node::SharedPtr node_;
  std::vector<LowpassFilterImpl> position_filters_;
  size_t num_joints_;
};
}  // namespace moveit_servo
