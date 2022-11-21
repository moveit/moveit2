/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
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
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
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

#include <pilz_industrial_motion_planner/limits_container.h>
#include <rclcpp/logger.hpp>

namespace pilz_industrial_motion_planner
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("pilz_industrial_motion_planner.limits_container");

LimitsContainer::LimitsContainer() : has_joint_limits_(false), has_cartesian_limits_(false)
{
}

bool LimitsContainer::hasJointLimits() const
{
  return has_joint_limits_;
}

void LimitsContainer::setJointLimits(JointLimitsContainer& joint_limits)
{
  has_joint_limits_ = true;
  joint_limits_ = joint_limits;
}

const JointLimitsContainer& LimitsContainer::getJointLimitContainer() const
{
  return joint_limits_;
}

void LimitsContainer::printCartesianLimits() const
{
  RCLCPP_DEBUG(LOGGER,
               "Pilz Cartesian Limits - Max Trans Vel : %f, Max Trans Acc : %f, Max Trans Dec : %f, Max Rot Vel : %f",
               cartesian_limits_.max_trans_vel, cartesian_limits_.max_trans_acc, cartesian_limits_.max_trans_dec,
               cartesian_limits_.max_rot_vel);
}

void LimitsContainer::setCartesianLimits(cartesian_limits::Params& cartesian_limits)
{
  has_cartesian_limits_ = true;
  cartesian_limits_ = cartesian_limits;
}

const cartesian_limits::Params& LimitsContainer::getCartesianLimits() const
{
  return cartesian_limits_;
}

}  // namespace pilz_industrial_motion_planner
