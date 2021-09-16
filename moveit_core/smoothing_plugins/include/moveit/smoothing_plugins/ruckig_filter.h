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
   Description: Jerk-limited smoothing with the Ruckig library.
 */

#pragma once

#include <moveit/robot_model/robot_model.h>
#include <moveit/smoothing_plugins/smoothing_base_class.h>

#include <ruckig/ruckig.hpp>

namespace smoothing_plugins
{
/**
 * Class RuckigFilter - Implementation of a signal filter to soften jerks.
 */
class RuckigFilter
{
public:
  RuckigFilter();

  double filter();

  void reset();
};

// Plugin
class RuckigFilterPlugin : public SmoothingBaseClass
{
public:
  RuckigFilterPlugin(){};

  bool initialize(rclcpp::Node::SharedPtr node, const moveit::core::JointModelGroup& group, size_t num_dof,
                  double timestep) override;

  bool doSmoothing(std::vector<double>& /*unused*/, std::vector<double>& velocity_vector) override;

  bool reset(const std::vector<double>& joint_positions) override;

private:
  size_t num_dof_;
  std::shared_ptr<ruckig::Ruckig<0, true /*debug*/>> ruckig_;
  std::shared_ptr<ruckig::InputParameter<0>> ruckig_input_;
  std::shared_ptr<ruckig::OutputParameter<0>> ruckig_output_;
};
}  // namespace smoothing_plugins
