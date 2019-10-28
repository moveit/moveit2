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

/* Author: Ioan Sucan */

#include <moveit/pick_place/pick_place_params.h>
#include "rclcpp/rclcpp.hpp"

namespace pick_place
{
namespace
{
class DynamicReconfigureImpl
{
public:
  DynamicReconfigureImpl()
  {
    node_ = rclcpp::Node::make_shared("~/pick_place");
    node_->declare_parameter("max_attempted_states_per_pose");
    node_->declare_parameter("max_consecutive_fail_attempts");
    node_->declare_parameter("cartesian_motion_step_size");
    node_->declare_parameter("jump_factor");

    auto pick_place_parameters = std::make_shared<rclcpp::SyncParametersClient>(node_);

    auto set_parameters_results = pick_place_parameters->set_parameters({
        rclcpp::Parameter("max_attempted_states_per_pose", 5), rclcpp::Parameter("max_consecutive_fail_attempts", 3),
        rclcpp::Parameter("cartesian_motion_step_size", 0.02), rclcpp::Parameter("jump_factor", 2.0),
    });
  }

  const PickPlaceParams& getParams() const
  {
    return params_;
  }

private:
  PickPlaceParams params_;
  rclcpp::Node::SharedPtr node_;

  void dynamicReconfigureCallback()
  {
    auto pick_place_parameters = std::make_shared<rclcpp::SyncParametersClient>(node_);

    for (auto& parameter :
         pick_place_parameters->get_parameters({ "max_attempted_states_per_pose", "max_consecutive_fail_attempts",
                                                 "cartesian_motion_step_size", "jump_factor" }))
    {
      if (parameter.get_name().compare("max_attempted_states_per_pose") == 0)
      {
        params_.max_goal_count_ = parameter.as_int();
      }
      if (parameter.get_name().compare("max_consecutive_fail_attempts") == 0)
      {
        params_.max_fail_ = parameter.as_int();
      }
      if (parameter.get_name().compare("cartesian_motion_step_size") == 0)
      {
        params_.max_step_ = parameter.as_double();
      }
      if (parameter.get_name().compare("jump_factor") == 0)
      {
        params_.jump_factor_ = parameter.as_double();
      }
    }
  }
};
}  // namespace
}  // namespace pick_place

pick_place::PickPlaceParams::PickPlaceParams() : max_goal_count_(5), max_fail_(3), max_step_(0.02), jump_factor_(2.0)
{
}

const pick_place::PickPlaceParams& pick_place::GetGlobalPickPlaceParams()
{
  static DynamicReconfigureImpl pick_place_params;
  return pick_place_params.getParams();
}
