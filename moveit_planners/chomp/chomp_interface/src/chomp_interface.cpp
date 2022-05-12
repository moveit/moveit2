/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: E. Gil Jones */

#include <chomp_interface/chomp_interface.h>

namespace chomp_interface
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("chomp_optimizer");

CHOMPInterface::CHOMPInterface(rclcpp::Node::SharedPtr node) : ChompPlanner(), node_(node)
{
  loadParams();
}

void CHOMPInterface::loadParams()
{
  node_->get_parameter_or("chomp.planning_time_limit", params_.planning_time_limit_, 10.0);
  node_->get_parameter_or("chomp.max_iterations", params_.max_iterations_, 200);
  node_->get_parameter_or("chomp.max_iterations_after_collision_free", params_.max_iterations_after_collision_free_, 5);

  node_->get_parameter_or("chomp.smoothness_cost_weight", params_.smoothness_cost_weight_, 0.1);
  node_->get_parameter_or("chomp.obstacle_cost_weight", params_.obstacle_cost_weight_, 1.0);
  node_->get_parameter_or("chomp.learning_rate", params_.learning_rate_, 0.01);

  node_->get_parameter_or("chomp.smoothness_cost_velocity", params_.smoothness_cost_velocity_, 0.0);
  node_->get_parameter_or("chomp.smoothness_cost_acceleration", params_.smoothness_cost_acceleration_, 1.0);
  node_->get_parameter_or("chomp.smoothness_cost_jerk", params_.smoothness_cost_jerk_, 0.0);
  node_->get_parameter_or("chomp.ridge_factor", params_.ridge_factor_, 0.0);
  node_->get_parameter_or("chomp.use_pseudo_inverse", params_.use_pseudo_inverse_, false);
  node_->get_parameter_or("chomp.pseudo_inverse_ridge_factor", params_.pseudo_inverse_ridge_factor_, 1e-4);

  node_->get_parameter_or("chomp.joint_update_limit", params_.joint_update_limit_, 0.1);
  node_->get_parameter_or("chomp.collision_clearance", params_.min_clearance_, 0.2);
  node_->get_parameter_or("chomp.collision_threshold", params_.collision_threshold_, 0.07);
  node_->get_parameter_or("chomp.use_stochastic_descent", params_.use_stochastic_descent_, true);
  params_.trajectory_initialization_method_ = "quintic-spline";
  std::string method;
  if (node_->get_parameter("chomp.trajectory_initialization_method", method) &&
      !params_.setTrajectoryInitializationMethod(method))
  {
    RCLCPP_ERROR(LOGGER,
                 "Attempted to set trajectory_initialization_method to invalid value '%s'. Using default '%s' "
                 "instead.",
                 method.c_str(), params_.trajectory_initialization_method_.c_str());
  }
  node_->get_parameter_or("chomp.enable_failure_recovery", params_.enable_failure_recovery_, false);
  node_->get_parameter_or("chomp.max_recovery_attempts", params_.max_recovery_attempts_, 5);
}
}  // namespace chomp_interface
