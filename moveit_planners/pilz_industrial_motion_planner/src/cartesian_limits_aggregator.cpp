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

#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter_value.hpp>
#include <limits>

#include "pilz_industrial_motion_planner/cartesian_limits_aggregator.h"

static const std::string PARAM_CARTESIAN_LIMITS_NS = "cartesian_limits";

static const std::string PARAM_MAX_TRANS_VEL = "max_trans_vel";
static const std::string PARAM_MAX_TRANS_ACC = "max_trans_acc";
static const std::string PARAM_MAX_TRANS_DEC = "max_trans_dec";
static const std::string PARAM_MAX_ROT_VEL = "max_rot_vel";
static const std::string PARAM_MAX_ROT_ACC = "max_rot_acc";
static const std::string PARAM_MAX_ROT_DEC = "max_rot_dec";

// TODO(sjahr) Refactor and use repository wide solution
bool declareAndGetParam(double& output_value, const std::string& param_name, const rclcpp::Node::SharedPtr& node)
{
  try
  {
    if (!node->has_parameter(param_name))
    {
      node->declare_parameter<double>(param_name, std::numeric_limits<double>::quiet_NaN());
    }
    node->get_parameter<double>(param_name, output_value);
    if (std::isnan(output_value))
    {
      RCLCPP_ERROR(node->get_logger(), "Parameter \'%s\', is not set in the config file.", param_name.c_str());
      return false;
    }
    return true;
  }
  catch (const rclcpp::exceptions::InvalidParameterTypeException& e)
  {
    RCLCPP_WARN(node->get_logger(), "InvalidParameterTypeException(\'%s\'): %s", param_name.c_str(), e.what());
    RCLCPP_ERROR(node->get_logger(), "Error getting parameter \'%s\', check parameter type in YAML file",
                 param_name.c_str());
    throw e;
  }
}

pilz_industrial_motion_planner::CartesianLimit
pilz_industrial_motion_planner::CartesianLimitsAggregator::getAggregatedLimits(const rclcpp::Node::SharedPtr& node,
                                                                               const std::string& param_namespace)
{
  std::string param_prefix = param_namespace + "." + PARAM_CARTESIAN_LIMITS_NS + ".";

  pilz_industrial_motion_planner::CartesianLimit cartesian_limit;

  // translational velocity
  double max_trans_vel;
  if (declareAndGetParam(max_trans_vel, param_prefix + PARAM_MAX_TRANS_VEL, node))
  {
    cartesian_limit.setMaxTranslationalVelocity(max_trans_vel);
  }

  // translational acceleration
  double max_trans_acc;
  if (declareAndGetParam(max_trans_acc, param_prefix + PARAM_MAX_TRANS_ACC, node))
  {
    cartesian_limit.setMaxTranslationalAcceleration(max_trans_acc);
  }

  // translational deceleration
  double max_trans_dec;
  if (declareAndGetParam(max_trans_dec, param_prefix + PARAM_MAX_TRANS_DEC, node))
  {
    cartesian_limit.setMaxTranslationalDeceleration(max_trans_dec);
  }

  // rotational velocity
  double max_rot_vel;
  if (declareAndGetParam(max_rot_vel, param_prefix + PARAM_MAX_ROT_VEL, node))
  {
    cartesian_limit.setMaxRotationalVelocity(max_rot_vel);
  }

  // rotational acceleration + deceleration deprecated
  // LCOV_EXCL_START
  if (node->has_parameter(param_prefix + PARAM_MAX_ROT_ACC) || node->has_parameter(param_prefix + PARAM_MAX_ROT_DEC))
  {
    RCLCPP_WARN(node->get_logger(),
                "Ignoring cartesian limits parameters for rotational acceleration / deceleration; these parameters are "
                "deprecated and are automatically calculated from translational to rotational ratio.");
  }
  // LCOV_EXCL_STOP

  return cartesian_limit;
}
