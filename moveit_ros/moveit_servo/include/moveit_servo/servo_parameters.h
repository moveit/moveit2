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

#include <rclcpp/node.hpp>
#include <mutex>
#include <thread>
#include <vector>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
namespace moveit_servo
{
using SetParameterCallbackType = std::function<rcl_interfaces::msg::SetParametersResult(const rclcpp::Parameter&)>;

// ROS params to be read. See the yaml file in /config for a description of each.
struct ServoParameters
{
  using SharedConstPtr = std::shared_ptr<const ServoParameters>;

  // Parameter namespace
  const std::string ns;

  // ROS Parameters
  // Note that all of these are effectively const because the only way to create one of these
  //  is as a shared_ptr to a constant struct.
  bool use_gazebo{ false };
  std::string status_topic{ "~/status" };
  // Properties of incoming commands
  std::string cartesian_command_in_topic{ "~/delta_twist_cmds" };
  std::string joint_command_in_topic{ "~/delta_joint_cmds" };
  std::string robot_link_command_frame{ "panda_link0" };
  std::string command_in_type{ "unitless" };
  double linear_scale{ 0.4 };
  double rotational_scale{ 0.8 };
  double joint_scale{ 0.5 };
  // Properties of Servo calculations
  double override_velocity_scaling_factor{ 0.0 };
  // Properties of outgoing commands
  std::string command_out_topic{ "/panda_arm_controller/joint_trajectory" };
  double publish_period{ 0.034 };
  std::string command_out_type{ "trajectory_msgs/JointTrajectory" };
  bool publish_joint_positions{ true };
  bool publish_joint_velocities{ true };
  bool publish_joint_accelerations{ false };
  // Plugins for smoothing outgoing commands
  std::string joint_topic{ "/joint_states" };
  std::string smoothing_filter_plugin_name{ "online_signal_smoothing::ButterworthFilterPlugin" };
  // MoveIt properties
  std::string move_group_name{ "panda_arm" };
  std::string planning_frame{ "panda_link0" };
  std::string ee_frame_name{ "panda_link8" };
  bool is_primary_planning_scene_monitor = { true };
  std::string monitored_planning_scene_topic{
    planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_TOPIC
  };
  // Stopping behaviour
  double incoming_command_timeout{ 0.1 };
  int num_outgoing_halt_msgs_to_publish{ 4 };
  bool halt_all_joints_in_joint_mode{ true };
  bool halt_all_joints_in_cartesian_mode{ true };
  // Configure handling of singularities and joint limits
  double lower_singularity_threshold{ 17.0 };
  double hard_stop_singularity_threshold{ 30.0 };
  double leaving_singularity_threshold_multiplier{ 2.0 };
  double joint_limit_margin{ 0.1 };
  bool low_latency_mode{ false };
  // Collision checking
  bool check_collisions{ true };
  double collision_check_rate{ 10.0 };
  double self_collision_proximity_threshold{ 0.01 };
  double scene_collision_proximity_threshold{ 0.02 };

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
  static SharedConstPtr makeServoParameters(const rclcpp::Node::SharedPtr& node, std::string ns = "moveit_servo",
                                            bool dynamic_parameters = true);

  /**
   * Register a callback for a parameter set event.
   * Note that these callbacks do not change any of the parameters struct.
   * Use a local variable for tracking the state of the dynamic parameter after initial bringup.
   * @param name Name of parameter (key used for callback in map)
   * @param callback function to call when parameter is changed
   */
  [[nodiscard]] bool registerSetParameterCallback(const std::string name, SetParameterCallbackType callback) const
  {
    if (callback_handler_)
    {
      const std::lock_guard<std::mutex> guard{ callback_handler_->mutex_ };
      callback_handler_->set_parameter_callbacks_[name].push_back(callback);
      return true;
    }
    return false;
  }
  static ServoParameters get(const std::string& ns,
                             const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_parameters);
  static std::optional<ServoParameters> validate(ServoParameters parameters);

private:
  // Private constructor because we only want this object to be created through the builder method makeServoParameters
  ServoParameters()
  {
  }

  struct CallbackHandler
  {
    // callback handler for the on set parameters callback
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handler_;

    // mutable so the callback can be registered objects that have const versions of this struct
    mutable std::mutex mutex_;
    mutable std::map<std::string, std::vector<SetParameterCallbackType>> set_parameter_callbacks_;

    // For registering with add_on_set_parameters_callback after initializing data
    rcl_interfaces::msg::SetParametersResult setParametersCallback(const std::vector<rclcpp::Parameter>& parameters);
  };

  std::shared_ptr<CallbackHandler> callback_handler_;

  static void declare(const std::string& ns,
                      const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_parameters);
};

}  // namespace moveit_servo
