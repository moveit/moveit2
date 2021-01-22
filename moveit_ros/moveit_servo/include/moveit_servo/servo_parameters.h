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

#include <thread>
#include <mutex>
#include <vector>
#include <rclcpp/rclcpp.hpp>

namespace moveit_servo
{
// Size of queues used in ros pub/sub/service
constexpr size_t ROS_QUEUE_SIZE = 2;

using SetParameterCallbackType = std::function<rcl_interfaces::msg::SetParametersResult(const rclcpp::Parameter&)>;

// Helper template for declaring and getting ros param
template <typename T>
void declareOrGetParam(T& output_value, const std::string& param_name, const rclcpp::Node::SharedPtr& node,
                       const rclcpp::Logger& logger);

// ROS params to be read. See the yaml file in /config for a description of each.
struct ServoParameters
{
  using SharedConstPtr = std::shared_ptr<const ServoParameters>;

  // Parameter namespace
  const std::string ns;

  // ROS Parameters
  // Note that all of these are effectively const because the only way to create one of these
  //  is as a shared_ptr to a constant struct.
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
  bool low_latency_mode;
  // Collision checking
  bool check_collisions;
  double collision_check_rate;
  std::string collision_check_type;
  double self_collision_proximity_threshold;
  double scene_collision_proximity_threshold;
  double collision_distance_safety_factor;
  double min_allowable_collision_distance;

  /**
   * Declares, reads, and validates parameters used for moveit_servo
   * @param node Shared ptr to node that will the parameters will be declared in. Params should be defined in
   * launch/config files
   * @param logger Logger for outputting warnings about the parameters
   * @param parameters The set up parameters that will be updated. After this call, they can be used to start a Servo
   * instance
   * @param ns Parameter namespace (as loaded in launch files). Defaults to "moveit_servo" but can be changed to allow
   * multiple arms/instances
   * @param dynamic_parameters Enable dynamic parameter handling. (default: true)
   * @return std::shared_ptr<ServoParameters> if all parameters were loaded and verified successfully, nullptr otherwise
   */
  static SharedConstPtr makeServoParameters(const rclcpp::Node::SharedPtr& node, const rclcpp::Logger& logger,
                                            std::string ns = "moveit_servo", bool dynamic_parameters = true);

  /**
   * Register a callback for a parameter set event.
   * Note that these callbacks do not change any of the parameters struct.
   * Use a local variable for tracking the state of the dynamic parameter after initial bringup.
   * @param name Name of parameter (key used for callback in map)
   * @param callback function to call when parameter is changed
   */
  void registerSetParameterCallback(const std::string name, SetParameterCallbackType callback) const
  {
    const std::lock_guard<std::mutex> guard(callback_mutex_);
    set_parameter_callbacks_[name].push_back(callback);
  }

private:
  // Private constructor because we only want this object to be created through the builder method makeServoParameters
  ServoParameters(const rclcpp::Logger& logger, std::string ns) : ns(ns), logger_(logger)
  {
  }
  const rclcpp::Logger& logger_;

  // callback handler for the on set parameters callback
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handler_;

  // mutable so the callback can be registered objects that have const versions of this struct
  mutable std::mutex callback_mutex_;
  mutable std::map<std::string, std::vector<SetParameterCallbackType>> set_parameter_callbacks_;

  // For registering with add_on_set_parameters_callback after initializing data
  rcl_interfaces::msg::SetParametersResult setParametersCallback(std::vector<rclcpp::Parameter> parameters);
};

}  // namespace moveit_servo
