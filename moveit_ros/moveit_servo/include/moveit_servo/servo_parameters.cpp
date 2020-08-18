/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

/* Author    : Adam Pettinger
   Desc      : Declares, loads, and checks ServoParameters for Servo
   Title     : servo_parameters.cpp
   Project   : moveit_servo
   Created   : 07/02/2020
*/


#include <rclcpp/rclcpp.hpp>

#include <type_traits>

#include <moveit_servo/servo_parameters.h>

namespace moveit_servo
{
template <typename T>
bool declareAndGetParam(const std::string& param_name, T& output_value, rclcpp::Node::SharedPtr& node,
                        const rclcpp::Logger& logger)
{
  bool got_param = false;
  try
  {
    node->declare_parameter<T>(param_name, T{});
    got_param = node->get_parameter(param_name, output_value);
  }
  catch(const rclcpp::exceptions::InvalidParameterTypeException& e)
  {
    // Catch a <double> parameter written in the yaml as "1" being considered an <int>
    if(std::is_same<T, double>::value)
    {
      int value;
      node->undeclare_parameter(param_name);
      node->declare_parameter<int>(param_name, 0);
      got_param = node->get_parameter(param_name, value);
      output_value = value;
    }
    else
    {
      throw e;
    }
  }

  if (got_param)
  {
    return true;
  }
  else
  {
    RCLCPP_WARN_STREAM(logger, "Unable to get parameter: \'" << param_name << "\'. Please check YAML file");
    return false;
  }
}

/**
 * Declares, reads, and validates parameters used for moveit_servo
 * @param node Shared ptr to node that will the parameters will be declared in. Params should be defined in launch/config files
 * @param logger Logger for outputting warnings about the parameters
 * @param parameters The set up parameters that will be updated. After this call, they can be used to start a Servo instance
 * @param ns Parameter namespace (as loaded in launch files). Defaults to "moveit_servo" but can be changed to allow multiple arms/instances
 * @return true if all parameters were loaded and verified successfully, false otherwise
 */
bool readParameters(rclcpp::Node::SharedPtr& node, const rclcpp::Logger& logger, ServoParametersPtr& parameters, std::string ns = "moveit_servo")
{
  bool error = false;

  // Get the parameters (organized same order as YAML file)
  error |= !declareAndGetParam<bool>(ns + ".use_gazebo", parameters->use_gazebo, node, logger);
  error |= !declareAndGetParam<std::string>(ns + ".status_topic", parameters->status_topic, node, logger);

  // Properties of incoming commands
  error |= !declareAndGetParam<std::string>(ns + ".cartesian_command_in_topic", parameters->cartesian_command_in_topic, node,
                                           logger);
  error |= !declareAndGetParam<std::string>(ns + ".joint_command_in_topic", parameters->joint_command_in_topic, node, logger);
  error |=
      !declareAndGetParam<std::string>(ns + ".robot_link_command_frame", parameters->robot_link_command_frame, node, logger);
  error |= !declareAndGetParam<std::string>(ns + ".command_in_type", parameters->command_in_type, node, logger);
  error |= !declareAndGetParam<double>(ns + ".scale.linear", parameters->linear_scale, node, logger);
  error |= !declareAndGetParam<double>(ns + ".scale.rotational", parameters->rotational_scale, node, logger);
  error |= !declareAndGetParam<double>(ns + ".scale.joint", parameters->joint_scale, node, logger);

  // Properties of outgoing commands
  error |= !declareAndGetParam<std::string>(ns + ".command_out_topic", parameters->command_out_topic, node, logger);
  error |= !declareAndGetParam<double>(ns + ".publish_period", parameters->publish_period, node, logger);
  error |= !declareAndGetParam<std::string>(ns + ".command_out_type", parameters->command_out_type, node, logger);
  error |= !declareAndGetParam<bool>(ns + ".publish_joint_positions", parameters->publish_joint_positions, node, logger);
  error |= !declareAndGetParam<bool>(ns + ".publish_joint_velocities", parameters->publish_joint_velocities, node, logger);
  error |=
      !declareAndGetParam<bool>(ns + ".publish_joint_accelerations", parameters->publish_joint_accelerations, node, logger);

  // Incoming Joint State properties
  error |= !declareAndGetParam<std::string>(ns + ".joint_topic", parameters->joint_topic, node, logger);
  error |= !declareAndGetParam<double>(ns + ".low_pass_filter_coeff", parameters->low_pass_filter_coeff, node, logger);

  // MoveIt properties
  error |= !declareAndGetParam<std::string>(ns + ".move_group_name", parameters->move_group_name, node, logger);
  error |= !declareAndGetParam<std::string>(ns + ".planning_frame", parameters->planning_frame, node, logger);

  // Stopping behaviour
  error |= !declareAndGetParam<double>(ns + ".incoming_command_timeout", parameters->incoming_command_timeout, node, logger);
  error |= !declareAndGetParam<int>(ns + ".num_outgoing_halt_msgs_to_publish", parameters->num_outgoing_halt_msgs_to_publish,
                                   node, logger);

  // Configure handling of singularities and joint limits
  error |=
      !declareAndGetParam<double>(ns + ".lower_singularity_threshold", parameters->lower_singularity_threshold, node, logger);
  error |= !declareAndGetParam<double>(ns + ".hard_stop_singularity_threshold", parameters->hard_stop_singularity_threshold,
                                      node, logger);
  error |= !declareAndGetParam<double>(ns + ".joint_limit_margin", parameters->joint_limit_margin, node, logger);

  // Collision checking
  error |= !declareAndGetParam<bool>(ns + ".check_collisions", parameters->check_collisions, node, logger);
  error |= !declareAndGetParam<double>(ns + ".collision_check_rate", parameters->collision_check_rate, node, logger);
  error |= !declareAndGetParam<std::string>(ns + ".collision_check_type", parameters->collision_check_type, node, logger);
  error |= !declareAndGetParam<double>(ns + ".self_collision_proximity_threshold",
                                      parameters->self_collision_proximity_threshold, node, logger);
  error |= !declareAndGetParam<double>(ns + ".scene_collision_proximity_threshold",
                                      parameters->scene_collision_proximity_threshold, node, logger);
  error |= !declareAndGetParam<double>(ns + ".collision_distance_safety_factor", parameters->collision_distance_safety_factor,
                                      node, logger);
  error |= !declareAndGetParam<double>(ns + ".min_allowable_collision_distance", parameters->min_allowable_collision_distance,
                                      node, logger);

  // Only continue if all parameters were found
  if (error)
  {
    RCLCPP_ERROR_STREAM(logger, "One or more Servo parameters missing, check YAML file before proceeding");
    return false;
  }

  // Begin input checking
  if (parameters->publish_period <= 0.)
  {
    RCLCPP_WARN(logger, "Parameter 'publish_period' should be "
                        "greater than zero. Check yaml file.");
    return false;
  }
  if (parameters->num_outgoing_halt_msgs_to_publish < 0)
  {
    RCLCPP_WARN(logger, "Parameter 'num_outgoing_halt_msgs_to_publish' should be greater than zero. Check yaml file.");
    return false;
  }
  if (parameters->hard_stop_singularity_threshold < parameters->lower_singularity_threshold)
  {
    RCLCPP_WARN(logger, "Parameter 'hard_stop_singularity_threshold' "
                        "should be greater than 'lower_singularity_threshold.' "
                        "Check yaml file.");
    return false;
  }
  if ((parameters->hard_stop_singularity_threshold < 0.) || (parameters->lower_singularity_threshold < 0.))
  {
    RCLCPP_WARN(logger, "Parameters 'hard_stop_singularity_threshold' "
                        "and 'lower_singularity_threshold' should be "
                        "greater than zero. Check yaml file.");
    return false;
  }
  if (parameters->low_pass_filter_coeff < 0.)
  {
    RCLCPP_WARN(logger, "Parameter 'low_pass_filter_coeff' should be "
                        "greater than zero. Check yaml file.");
    return false;
  }
  if (parameters->joint_limit_margin < 0.)
  {
    RCLCPP_WARN(logger, "Parameter 'joint_limit_margin' should be "
                        "greater than zero. Check yaml file.");
    return false;
  }
  if (parameters->command_in_type != "unitless" && parameters->command_in_type != "speed_units")
  {
    RCLCPP_WARN(logger, "command_in_type should be 'unitless' or "
                        "'speed_units'. Check yaml file.");
    return false;
  }
  if (parameters->command_out_type != "trajectory_msgs/JointTrajectory" &&
      parameters->command_out_type != "std_msgs/Float64MultiArray")
  {
    RCLCPP_WARN(logger, "Parameter command_out_type should be "
                        "'trajectory_msgs/JointTrajectory' or "
                        "'std_msgs/Float64MultiArray'. Check yaml file.");
    return false;
  }
  if (!parameters->publish_joint_positions && !parameters->publish_joint_velocities &&
      !parameters->publish_joint_accelerations)
  {
    RCLCPP_WARN(logger, "At least one of publish_joint_positions / "
                        "publish_joint_velocities / "
                        "publish_joint_accelerations must be true. Check "
                        "yaml file.");
    return false;
  }
  if ((parameters->command_out_type == "std_msgs/Float64MultiArray") && parameters->publish_joint_positions &&
      parameters->publish_joint_velocities)
  {
    RCLCPP_WARN(logger, "When publishing a std_msgs/Float64MultiArray, "
                        "you must select positions OR velocities.");
    return false;
  }
  // Collision checking
  if (parameters->collision_check_type != "threshold_distance" && parameters->collision_check_type != "stop_distance")
  {
    RCLCPP_WARN(logger, "collision_check_type must be 'threshold_distance' or 'stop_distance'");
    return false;
  }
  if (parameters->self_collision_proximity_threshold < 0.)
  {
    RCLCPP_WARN(logger, "Parameter 'self_collision_proximity_threshold' should be "
                        "greater than zero. Check yaml file.");
    return false;
  }
  if (parameters->scene_collision_proximity_threshold < 0.)
  {
    RCLCPP_WARN(logger, "Parameter 'scene_collision_proximity_threshold' should be "
                        "greater than zero. Check yaml file.");
    return false;
  }
  if (parameters->scene_collision_proximity_threshold < parameters->self_collision_proximity_threshold)
  {
    RCLCPP_WARN(logger, "Parameter 'self_collision_proximity_threshold' should probably be less "
                        "than or equal to 'scene_collision_proximity_threshold'. Check yaml file.");
  }
  if (parameters->collision_check_rate < 0)
  {
    RCLCPP_WARN(logger, "Parameter 'collision_check_rate' should be "
                        "greater than zero. Check yaml file.");
    return false;
  }
  if (parameters->collision_distance_safety_factor < 1)
  {
    RCLCPP_WARN(logger, "Parameter 'collision_distance_safety_factor' should be "
                        "greater than or equal to 1. Check yaml file.");
    return false;
  }
  if (parameters->min_allowable_collision_distance < 0)
  {
    RCLCPP_WARN(logger, "Parameter 'min_allowable_collision_distance' should be "
                        "greater than zero. Check yaml file.");
    return false;
  }

  return true;
}

}  // namespace moveit_servo
