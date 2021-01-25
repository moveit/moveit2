/*******************************************************************************
 *      Title     : servo_parameters.h
 *      Project   : moveit_servo
 *      Created   : 1/11/2019
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#pragma once

namespace moveit_servo
{
// Size of queues used in ros pub/sub/service
constexpr size_t ROS_QUEUE_SIZE = 2;

// ROS params to be read. See the yaml file in /config for a description of each.
struct ServoParameters
{
  bool use_gazebo;
  std::string status_topic;
  // Properties of incoming commands
  std::string cartesian_command_in_topic;
  std::string joint_command_in_topic;
  std::string robot_link_command_frame;
  std::string command_in_type;
  double linear_scale;
  double rotational_scale;
  double joint_scale;
  // Properties of outgoing commands
  std::string command_out_topic;
  double publish_period;
  std::string command_out_type;
  bool publish_joint_positions;
  bool publish_joint_velocities;
  bool publish_joint_accelerations;
  // Incoming Joint State properties
  std::string joint_topic;
  double low_pass_filter_coeff;
  // MoveIt properties
  std::string move_group_name;
  std::string planning_frame;
  std::string ee_frame_name;
  // Stopping behaviour
  double incoming_command_timeout;
  int num_outgoing_halt_msgs_to_publish;
  // Configure handling of singularities and joint limits
  double lower_singularity_threshold;
  double hard_stop_singularity_threshold;
  double joint_limit_margin;
  // Collision checking
  bool check_collisions;
  double collision_check_rate;
  std::string collision_check_type;
  double self_collision_proximity_threshold;
  double scene_collision_proximity_threshold;
  double collision_distance_safety_factor;
  double min_allowable_collision_distance;
};

using ServoParametersPtr = std::shared_ptr<ServoParameters>;

template <typename T>
void declareOrGetParam(T& output_value, const std::string& param_name, const rclcpp::Node::SharedPtr& node,
                       const rclcpp::Logger& logger)
{
  try
  {
    if (node->has_parameter(param_name))
      node->get_parameter<T>(param_name, output_value);
    else
      output_value = node->declare_parameter<T>(param_name, T{});
  }
  catch (const rclcpp::exceptions::InvalidParameterTypeException& e)
  {
    // Catch a <double> parameter written in the yaml as "1" being considered an <int>
    if (std::is_same<T, double>::value)
    {
      node->undeclare_parameter(param_name);
      output_value = node->declare_parameter<int>(param_name, 0);
    }
    else
    {
      RCLCPP_ERROR_STREAM(logger, "Error getting parameter \'" << param_name << "\', check parameter type in YAML file");
      throw e;
    }
  }
  RCLCPP_INFO_STREAM(logger, "Found parameter - " << param_name << ": " << output_value);
}

/**
 * Declares, reads, and validates parameters used for moveit_servo
 * @param node Shared ptr to node that will the parameters will be declared in. Params should be defined in
 * launch/config files
 * @param logger Logger for outputting warnings about the parameters
 * @param parameters The set up parameters that will be updated. After this call, they can be used to start a Servo
 * instance
 * @param ns Parameter namespace (as loaded in launch files). Defaults to "moveit_servo" but can be changed to allow
 * multiple arms/instances
 * @return true if all parameters were loaded and verified successfully, false otherwise
 */
bool readParameters(ServoParametersPtr& parameters, const rclcpp::Node::SharedPtr& node, const rclcpp::Logger& logger,
                    std::string ns = "moveit_servo");

}  // namespace moveit_servo
