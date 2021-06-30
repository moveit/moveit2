/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
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

/* Author    : Adam Pettinger
   Desc      : Declares, loads, and checks ServoParameters for Servo
   Title     : servo_parameters.cpp
   Project   : moveit_servo
   Created   : 07/02/2020
*/

#include <moveit_servo/servo_parameters.h>
#include <rclcpp/rclcpp.hpp>
#include <type_traits>

using namespace std::placeholders;  // for _1, _2 etc.

namespace moveit_servo
{
template <typename T>
void declareOrGetParam(T& output_value, const std::string& param_name, const rclcpp::Node::SharedPtr& node,
                       const rclcpp::Logger& logger)
{
  try
  {
    if (node->has_parameter(param_name))
    {
      node->get_parameter<T>(param_name, output_value);
    }
    else
    {
      output_value = node->declare_parameter<T>(param_name, T{});
    }
  }
  catch (const rclcpp::exceptions::InvalidParameterTypeException& e)
  {
    RCLCPP_WARN_STREAM(logger, "InvalidParameterTypeException(" << param_name << "): " << e.what());
    RCLCPP_ERROR_STREAM(logger, "Error getting parameter \'" << param_name << "\', check parameter type in YAML file");
    throw e;
  }

  RCLCPP_INFO_STREAM(logger, "Found parameter - " << param_name << ": " << output_value);
}

rcl_interfaces::msg::SetParametersResult
ServoParameters::setParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  const std::lock_guard<std::mutex> guard(callback_mutex_);
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;

  for (const auto& parameter : parameters)
  {
    auto search = set_parameter_callbacks_.find(parameter.get_name());
    if (search != set_parameter_callbacks_.end())
    {
      RCLCPP_INFO_STREAM(logger_, "setParametersCallback - "
                                      << parameter.get_name() << "<" << parameter.get_type_name()
                                      << ">: " << rclcpp::to_string(parameter.get_parameter_value()));
      for (const auto& callback : search->second)
      {
        result = callback(parameter);

        if (!result.successful)
        {
          // Handle automatic parameter set failure
          return result;
        }
      }
    }
  }

  return result;
}

ServoParameters::SharedConstPtr ServoParameters::makeServoParameters(const rclcpp::Node::SharedPtr& node,
                                                                     const rclcpp::Logger& logger,
                                                                     std::string ns, /* = "moveit_servo"*/
                                                                     bool dynamic_parameters /* = true */)
{
  auto parameters = std::shared_ptr<ServoParameters>(new ServoParameters(logger, ns));

  // Get the parameters (organized same order as YAML file)
  declareOrGetParam<bool>(parameters->use_gazebo, ns + ".use_gazebo", node, logger);
  declareOrGetParam<std::string>(parameters->status_topic, ns + ".status_topic", node, logger);

  // Properties of incoming commands
  declareOrGetParam<std::string>(parameters->cartesian_command_in_topic, ns + ".cartesian_command_in_topic", node,
                                 logger);
  declareOrGetParam<std::string>(parameters->joint_command_in_topic, ns + ".joint_command_in_topic", node, logger);
  declareOrGetParam<std::string>(parameters->robot_link_command_frame, ns + ".robot_link_command_frame", node, logger);
  declareOrGetParam<std::string>(parameters->command_in_type, ns + ".command_in_type", node, logger);
  declareOrGetParam<double>(parameters->linear_scale, ns + ".scale.linear", node, logger);
  declareOrGetParam<double>(parameters->rotational_scale, ns + ".scale.rotational", node, logger);
  declareOrGetParam<double>(parameters->joint_scale, ns + ".scale.joint", node, logger);

  // Properties of outgoing commands
  declareOrGetParam<std::string>(parameters->command_out_topic, ns + ".command_out_topic", node, logger);
  declareOrGetParam<double>(parameters->publish_period, ns + ".publish_period", node, logger);
  declareOrGetParam<std::string>(parameters->command_out_type, ns + ".command_out_type", node, logger);
  declareOrGetParam<bool>(parameters->publish_joint_positions, ns + ".publish_joint_positions", node, logger);
  declareOrGetParam<bool>(parameters->publish_joint_velocities, ns + ".publish_joint_velocities", node, logger);
  declareOrGetParam<bool>(parameters->publish_joint_accelerations, ns + ".publish_joint_accelerations", node, logger);
  declareOrGetParam<bool>(parameters->low_latency_mode, ns + ".low_latency_mode", node, logger);

  // Incoming Joint State properties
  declareOrGetParam<std::string>(parameters->joint_topic, ns + ".joint_topic", node, logger);
  declareOrGetParam<double>(parameters->low_pass_filter_coeff, ns + ".low_pass_filter_coeff", node, logger);

  // MoveIt properties
  declareOrGetParam<std::string>(parameters->move_group_name, ns + ".move_group_name", node, logger);
  declareOrGetParam<std::string>(parameters->planning_frame, ns + ".planning_frame", node, logger);
  declareOrGetParam<std::string>(parameters->ee_frame_name, ns + ".ee_frame_name", node, logger);

  // Stopping behaviour
  declareOrGetParam<double>(parameters->incoming_command_timeout, ns + ".incoming_command_timeout", node, logger);
  declareOrGetParam<int>(parameters->num_outgoing_halt_msgs_to_publish, ns + ".num_outgoing_halt_msgs_to_publish", node,
                         logger);

  // Configure handling of singularities and joint limits
  declareOrGetParam<double>(parameters->lower_singularity_threshold, ns + ".lower_singularity_threshold", node, logger);
  declareOrGetParam<double>(parameters->hard_stop_singularity_threshold, ns + ".hard_stop_singularity_threshold", node,
                            logger);
  declareOrGetParam<double>(parameters->joint_limit_margin, ns + ".joint_limit_margin", node, logger);

  // Collision checking
  declareOrGetParam<bool>(parameters->check_collisions, ns + ".check_collisions", node, logger);
  declareOrGetParam<double>(parameters->collision_check_rate, ns + ".collision_check_rate", node, logger);
  declareOrGetParam<std::string>(parameters->collision_check_type, ns + ".collision_check_type", node, logger);
  declareOrGetParam<double>(parameters->self_collision_proximity_threshold, ns + ".self_collision_proximity_threshold",
                            node, logger);
  declareOrGetParam<double>(parameters->scene_collision_proximity_threshold,
                            ns + ".scene_collision_proximity_threshold", node, logger);
  declareOrGetParam<double>(parameters->collision_distance_safety_factor, ns + ".collision_distance_safety_factor",
                            node, logger);
  declareOrGetParam<double>(parameters->min_allowable_collision_distance, ns + ".min_allowable_collision_distance",
                            node, logger);

  // Begin input checking
  if (parameters->publish_period <= 0.)
  {
    RCLCPP_WARN(logger, "Parameter 'publish_period' should be "
                        "greater than zero. Check yaml file.");
    return nullptr;
  }
  if (parameters->num_outgoing_halt_msgs_to_publish < 0)
  {
    RCLCPP_WARN(logger, "Parameter 'num_outgoing_halt_msgs_to_publish' should be greater than zero. Check yaml file.");
    return nullptr;
  }
  if (parameters->hard_stop_singularity_threshold <= parameters->lower_singularity_threshold)
  {
    RCLCPP_WARN(logger, "Parameter 'hard_stop_singularity_threshold' "
                        "should be greater than 'lower_singularity_threshold.' "
                        "Check yaml file.");
    return nullptr;
  }
  if ((parameters->hard_stop_singularity_threshold <= 0.) || (parameters->lower_singularity_threshold <= 0.))
  {
    RCLCPP_WARN(logger, "Parameters 'hard_stop_singularity_threshold' "
                        "and 'lower_singularity_threshold' should be "
                        "greater than zero. Check yaml file.");
    return nullptr;
  }
  if (parameters->low_pass_filter_coeff <= 0.)
  {
    RCLCPP_WARN(logger, "Parameter 'low_pass_filter_coeff' should be "
                        "greater than zero. Check yaml file.");
    return nullptr;
  }
  if (parameters->joint_limit_margin < 0.)
  {
    RCLCPP_WARN(logger, "Parameter 'joint_limit_margin' should usually be  greater than or equal to zero, "
                        "although negative values can be used if the specified joint limits are actually soft. "
                        "Check yaml file.");
  }
  if (parameters->command_in_type != "unitless" && parameters->command_in_type != "speed_units")
  {
    RCLCPP_WARN(logger, "command_in_type should be 'unitless' or "
                        "'speed_units'. Check yaml file.");
    return nullptr;
  }
  if (parameters->command_out_type != "trajectory_msgs/JointTrajectory" &&
      parameters->command_out_type != "std_msgs/Float64MultiArray")
  {
    RCLCPP_WARN(logger, "Parameter command_out_type should be "
                        "'trajectory_msgs/JointTrajectory' or "
                        "'std_msgs/Float64MultiArray'. Check yaml file.");
    return nullptr;
  }
  if (!parameters->publish_joint_positions && !parameters->publish_joint_velocities &&
      !parameters->publish_joint_accelerations)
  {
    RCLCPP_WARN(logger, "At least one of publish_joint_positions / "
                        "publish_joint_velocities / "
                        "publish_joint_accelerations must be true. Check "
                        "yaml file.");
    return nullptr;
  }
  if ((parameters->command_out_type == "std_msgs/Float64MultiArray") && parameters->publish_joint_positions &&
      parameters->publish_joint_velocities)
  {
    RCLCPP_WARN(logger, "When publishing a std_msgs/Float64MultiArray, "
                        "you must select positions OR velocities.");
    return nullptr;
  }
  // Collision checking
  if (parameters->collision_check_type != "threshold_distance" && parameters->collision_check_type != "stop_distance")
  {
    RCLCPP_WARN(logger, "collision_check_type must be 'threshold_distance' or 'stop_distance'");
    return nullptr;
  }
  if (parameters->self_collision_proximity_threshold <= 0.)
  {
    RCLCPP_WARN(logger, "Parameter 'self_collision_proximity_threshold' should be "
                        "greater than zero. Check yaml file.");
    return nullptr;
  }
  if (parameters->scene_collision_proximity_threshold <= 0.)
  {
    RCLCPP_WARN(logger, "Parameter 'scene_collision_proximity_threshold' should be "
                        "greater than zero. Check yaml file.");
    return nullptr;
  }
  if (parameters->scene_collision_proximity_threshold < parameters->self_collision_proximity_threshold)
  {
    RCLCPP_WARN(logger, "Parameter 'self_collision_proximity_threshold' should probably be less "
                        "than or equal to 'scene_collision_proximity_threshold'. Check yaml file.");
  }
  if (parameters->collision_check_rate <= 0)
  {
    RCLCPP_WARN(logger, "Parameter 'collision_check_rate' should be "
                        "greater than zero. Check yaml file.");
    return nullptr;
  }
  if (parameters->collision_distance_safety_factor < 1)
  {
    RCLCPP_WARN(logger, "Parameter 'collision_distance_safety_factor' should be "
                        "greater than or equal to 1. Check yaml file.");
    return nullptr;
  }
  if (parameters->min_allowable_collision_distance <= 0)
  {
    RCLCPP_WARN(logger, "Parameter 'min_allowable_collision_distance' should be "
                        "greater than zero. Check yaml file.");
    return nullptr;
  }

  // register parameter change callback
  if (dynamic_parameters)
  {
    parameters->on_set_parameters_callback_handler_ =
        node->add_on_set_parameters_callback(std::bind(&ServoParameters::setParametersCallback, parameters.get(), _1));
  }

  return parameters;
}

}  // namespace moveit_servo
