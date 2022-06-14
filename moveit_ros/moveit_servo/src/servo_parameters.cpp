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
#include <moveit_servo/parameter_descriptor_builder.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>
#include <type_traits>

namespace moveit_servo
{
namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.servo_parameters");
}

rcl_interfaces::msg::SetParametersResult
ServoParameters::CallbackHandler::setParametersCallback(const std::vector<rclcpp::Parameter>& parameters)
{
  const std::lock_guard<std::mutex> guard(mutex_);
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;

  for (const auto& parameter : parameters)
  {
    auto search = set_parameter_callbacks_.find(parameter.get_name());
    if (search != set_parameter_callbacks_.end())
    {
      RCLCPP_INFO_STREAM(LOGGER, "setParametersCallback - "
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

void ServoParameters::declare(const std::string& ns,
                              const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_parameters)
{
  using rclcpp::ParameterValue;
  using rclcpp::ParameterType::PARAMETER_BOOL;
  using rclcpp::ParameterType::PARAMETER_DOUBLE;
  using rclcpp::ParameterType::PARAMETER_INTEGER;
  using rclcpp::ParameterType::PARAMETER_STRING;
  auto parameters = ServoParameters{};

  // ROS Parameters
  node_parameters->declare_parameter(
      ns + ".use_gazebo", ParameterValue{ parameters.use_gazebo },
      ParameterDescriptorBuilder{}
          .type(PARAMETER_BOOL)
          .description("Whether the robot is started in a Gazebo simulation environment"));
  node_parameters->declare_parameter(
      ns + ".status_topic", ParameterValue{ parameters.status_topic },
      ParameterDescriptorBuilder{}.type(PARAMETER_STRING).description("Publish status to this topic"));
  // Properties of incoming commands
  node_parameters->declare_parameter(
      ns + ".cartesian_command_in_topic", ParameterValue{ parameters.cartesian_command_in_topic },
      ParameterDescriptorBuilder{}.type(PARAMETER_STRING).description("Topic for incoming Cartesian twist commands"));
  node_parameters->declare_parameter(
      ns + ".joint_command_in_topic", ParameterValue{ parameters.joint_command_in_topic },
      ParameterDescriptorBuilder{}.type(PARAMETER_STRING).description("Topic for incoming joint angle commands"));
  node_parameters->declare_parameter(
      ns + ".robot_link_command_frame", ParameterValue{ parameters.robot_link_command_frame },
      ParameterDescriptorBuilder{}
          .type(PARAMETER_STRING)
          .description("Commands must be given in the frame of a robot link. Usually either the base or end effector"));
  node_parameters->declare_parameter(ns + ".command_in_type", ParameterValue{ parameters.command_in_type },
                                     ParameterDescriptorBuilder{}
                                         .type(PARAMETER_STRING)
                                         .description("\"unitless\"> in the range [-1:1], as if from joystick. "
                                                      "\"speed_units\"> cmds are in m/s and rad/s"));
  node_parameters->declare_parameter(ns + ".scale.linear", ParameterValue{ parameters.linear_scale },
                                     ParameterDescriptorBuilder{}
                                         .type(PARAMETER_DOUBLE)
                                         .description("Max linear velocity. Unit is [m/s]. "
                                                      "Only used for Cartesian commands."));
  node_parameters->declare_parameter(ns + ".scale.rotational", ParameterValue{ parameters.rotational_scale },
                                     ParameterDescriptorBuilder{}
                                         .type(PARAMETER_DOUBLE)
                                         .description("Max angular velocity. Unit is [rad/s]. "
                                                      "Only used for Cartesian commands."));
  node_parameters->declare_parameter(ns + ".scale.joint", ParameterValue{ parameters.joint_scale },
                                     ParameterDescriptorBuilder{}
                                         .type(PARAMETER_DOUBLE)
                                         .description("Max joint angular/linear velocity. Only used for joint "
                                                      "commands on joint_command_in_topic."));

  // Properties of Servo calculations
  node_parameters->declare_parameter(ns + ".override_velocity_scaling_factor",
                                     ParameterValue{ parameters.override_velocity_scaling_factor },
                                     ParameterDescriptorBuilder{}
                                         .type(PARAMETER_DOUBLE)
                                         .description("Override constant scalar of how fast the robot should jog."
                                                      "Valid values are between 0-1.0"));

  // Properties of outgoing commands
  node_parameters->declare_parameter(
      ns + ".command_out_topic", ParameterValue{ parameters.command_out_topic },
      ParameterDescriptorBuilder{}.type(PARAMETER_STRING).description("Publish outgoing commands here"));
  node_parameters->declare_parameter(
      ns + ".publish_period", ParameterValue{ parameters.publish_period },
      ParameterDescriptorBuilder{}.type(PARAMETER_DOUBLE).description("1/Nominal publish rate [seconds]"));
  node_parameters->declare_parameter(
      ns + ".command_out_type", ParameterValue{ parameters.command_out_type },
      ParameterDescriptorBuilder{}
          .type(PARAMETER_STRING)
          .description("What type of topic does your robot driver expect? Currently supported are "
                       "std_msgs/Float64MultiArray or trajectory_msgs/JointTrajectory"));
  node_parameters->declare_parameter(
      ns + ".publish_joint_positions", ParameterValue{ parameters.publish_joint_positions },
      ParameterDescriptorBuilder{}
          .type(PARAMETER_BOOL)
          .description("What to publish? Can save some bandwidth as most robots only require positions or velocities"));
  node_parameters->declare_parameter(
      ns + ".publish_joint_velocities", ParameterValue{ parameters.publish_joint_velocities },
      ParameterDescriptorBuilder{}
          .type(PARAMETER_BOOL)
          .description("What to publish? Can save some bandwidth as most robots only require positions or velocities"));
  node_parameters->declare_parameter(
      ns + ".publish_joint_accelerations", ParameterValue{ parameters.publish_joint_accelerations },
      ParameterDescriptorBuilder{}
          .type(PARAMETER_BOOL)
          .description("What to publish? Can save some bandwidth as most robots only require positions or velocities"));
  node_parameters->declare_parameter(ns + ".low_latency_mode", ParameterValue{ parameters.low_latency_mode },
                                     ParameterDescriptorBuilder{}.type(PARAMETER_BOOL).description("Low latency mode"));

  // Incoming Joint State properties
  node_parameters->declare_parameter(ns + ".joint_topic", ParameterValue{ parameters.joint_topic },
                                     ParameterDescriptorBuilder{}.type(PARAMETER_STRING).description("Joint topic"));
  node_parameters->declare_parameter(
      ns + ".smoothing_filter_plugin_name", ParameterValue{ parameters.smoothing_filter_plugin_name },
      ParameterDescriptorBuilder{}.type(PARAMETER_STRING).description("Plugins for smoothing outgoing commands"));

  // MoveIt properties
  node_parameters->declare_parameter(
      ns + ".move_group_name", ParameterValue{ parameters.move_group_name },
      ParameterDescriptorBuilder{}.type(PARAMETER_STRING).description("Often 'manipulator' or 'arm'"));
  node_parameters->declare_parameter(ns + ".planning_frame", ParameterValue{ parameters.planning_frame },
                                     ParameterDescriptorBuilder{}
                                         .type(PARAMETER_STRING)
                                         .description("The MoveIt planning frame. Often 'base_link' or 'world'"));
  node_parameters->declare_parameter(ns + ".ee_frame_name", ParameterValue{ parameters.ee_frame_name },
                                     ParameterDescriptorBuilder{}
                                         .type(PARAMETER_STRING)
                                         .description("The name of the end effector link, used to return the EE pose"));
  node_parameters->declare_parameter(
      ns + ".is_primary_planning_scene_monitor", ParameterValue{ parameters.is_primary_planning_scene_monitor },
      ParameterDescriptorBuilder{}.type(PARAMETER_BOOL).description("Is primary planning scene monitor"));
  node_parameters->declare_parameter(
      ns + ".monitored_planning_scene_topic", ParameterValue{ parameters.monitored_planning_scene_topic },
      ParameterDescriptorBuilder{}.type(PARAMETER_STRING).description("Monitored planning scene topic"));

  // Stopping behaviour
  node_parameters->declare_parameter(
      ns + ".incoming_command_timeout", ParameterValue{ parameters.incoming_command_timeout },
      ParameterDescriptorBuilder{}
          .type(PARAMETER_DOUBLE)
          .description("Stop servoing if X seconds elapse without a new command. If 0, republish commands forever even "
                       "if the robot is stationary.Otherwise, specify num.to publish. Important because ROS may drop "
                       "some messages and we need the robot to halt reliably."));
  node_parameters->declare_parameter(
      ns + ".num_outgoing_halt_msgs_to_publish", ParameterValue{ parameters.num_outgoing_halt_msgs_to_publish },
      ParameterDescriptorBuilder{}.type(PARAMETER_INTEGER).description("Num outgoing halt msgs to publish"));
  node_parameters->declare_parameter(
      ns + ".halt_all_joints_in_joint_mode", ParameterValue{ parameters.halt_all_joints_in_joint_mode },
      ParameterDescriptorBuilder{}.type(PARAMETER_BOOL).description("Halt all joints in joint mode"));
  node_parameters->declare_parameter(
      ns + ".halt_all_joints_in_cartesian_mode", ParameterValue{ parameters.halt_all_joints_in_cartesian_mode },
      ParameterDescriptorBuilder{}.type(PARAMETER_BOOL).description("Halt all joints in cartesian mode"));

  // Configure handling of singularities and joint limits
  node_parameters->declare_parameter(
      ns + ".lower_singularity_threshold", ParameterValue{ parameters.lower_singularity_threshold },
      ParameterDescriptorBuilder{}
          .type(PARAMETER_DOUBLE)
          .description("Start decelerating when the condition number hits this (close to singularity)"));
  node_parameters->declare_parameter(
      ns + ".hard_stop_singularity_threshold", ParameterValue{ parameters.hard_stop_singularity_threshold },
      ParameterDescriptorBuilder{}.type(PARAMETER_DOUBLE).description("Stop when the condition number hits this"));
  node_parameters->declare_parameter(
      ns + ".joint_limit_margin", ParameterValue{ parameters.joint_limit_margin },
      ParameterDescriptorBuilder{}
          .type(PARAMETER_DOUBLE)
          .description("Added as a buffer to joint limits [radians]. If moving quickly, make this larger."));

  // Collision checking
  node_parameters->declare_parameter(ns + ".check_collisions", ParameterValue{ parameters.check_collisions },
                                     ParameterDescriptorBuilder{}.type(PARAMETER_BOOL).description("Check collisions?"));
  node_parameters->declare_parameter(
      ns + ".collision_check_rate", ParameterValue(parameters.collision_check_rate),
      ParameterDescriptorBuilder{}
          .type(PARAMETER_DOUBLE)
          .description("[Hz] Collision-checking can easily bog down a CPU if done too often. Collision checking begins "
                       "slowing down when nearer than a specified distance."));
  node_parameters->declare_parameter(ns + ".self_collision_proximity_threshold",
                                     ParameterValue{ parameters.self_collision_proximity_threshold },
                                     ParameterDescriptorBuilder{}
                                         .type(PARAMETER_DOUBLE)
                                         .description("Start decelerating when a self-collision is this far [m]"));
  node_parameters->declare_parameter(ns + ".scene_collision_proximity_threshold",
                                     ParameterValue{ parameters.scene_collision_proximity_threshold },
                                     ParameterDescriptorBuilder{}
                                         .type(PARAMETER_DOUBLE)
                                         .description("Start decelerating when a scene collision is this far [m]"));
}

ServoParameters ServoParameters::get(const std::string& ns,
                                     const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_parameters)
{
  auto parameters = ServoParameters{};

  // ROS Parameters
  parameters.use_gazebo = node_parameters->get_parameter(ns + ".use_gazebo").as_bool();
  parameters.status_topic = node_parameters->get_parameter(ns + ".status_topic").as_string();

  // Properties of incoming commands
  parameters.cartesian_command_in_topic =
      node_parameters->get_parameter(ns + ".cartesian_command_in_topic").as_string();
  parameters.joint_command_in_topic = node_parameters->get_parameter(ns + ".joint_command_in_topic").as_string();
  parameters.robot_link_command_frame = node_parameters->get_parameter(ns + ".robot_link_command_frame").as_string();
  parameters.command_in_type = node_parameters->get_parameter(ns + ".command_in_type").as_string();
  parameters.linear_scale = node_parameters->get_parameter(ns + ".scale.linear").as_double();
  parameters.rotational_scale = node_parameters->get_parameter(ns + ".scale.rotational").as_double();
  parameters.joint_scale = node_parameters->get_parameter(ns + ".scale.joint").as_double();

  // Properties of Servo calculations
  parameters.override_velocity_scaling_factor =
      node_parameters->get_parameter(ns + ".override_velocity_scaling_factor").as_double();

  // Properties of outgoing commands
  parameters.command_out_topic = node_parameters->get_parameter(ns + ".command_out_topic").as_string();
  parameters.publish_period = node_parameters->get_parameter(ns + ".publish_period").as_double();
  parameters.command_out_type = node_parameters->get_parameter(ns + ".command_out_type").as_string();
  parameters.publish_joint_positions = node_parameters->get_parameter(ns + ".publish_joint_positions").as_bool();
  parameters.publish_joint_velocities = node_parameters->get_parameter(ns + ".publish_joint_velocities").as_bool();
  parameters.publish_joint_accelerations =
      node_parameters->get_parameter(ns + ".publish_joint_accelerations").as_bool();
  parameters.low_latency_mode = node_parameters->get_parameter(ns + ".low_latency_mode").as_bool();

  // Incoming Joint State properties
  parameters.joint_topic = node_parameters->get_parameter(ns + ".joint_topic").as_string();
  parameters.smoothing_filter_plugin_name =
      node_parameters->get_parameter(ns + ".smoothing_filter_plugin_name").as_string();

  // MoveIt properties
  parameters.move_group_name = node_parameters->get_parameter(ns + ".move_group_name").as_string();
  parameters.planning_frame = node_parameters->get_parameter(ns + ".planning_frame").as_string();
  parameters.ee_frame_name = node_parameters->get_parameter(ns + ".ee_frame_name").as_string();
  parameters.is_primary_planning_scene_monitor =
      node_parameters->get_parameter(ns + ".is_primary_planning_scene_monitor").as_bool();
  parameters.monitored_planning_scene_topic =
      node_parameters->get_parameter(ns + ".monitored_planning_scene_topic").as_string();

  // Stopping behaviour
  parameters.incoming_command_timeout = node_parameters->get_parameter(ns + ".incoming_command_timeout").as_double();
  parameters.num_outgoing_halt_msgs_to_publish =
      node_parameters->get_parameter(ns + ".num_outgoing_halt_msgs_to_publish").as_int();
  parameters.halt_all_joints_in_joint_mode =
      node_parameters->get_parameter(ns + ".halt_all_joints_in_joint_mode").as_bool();
  parameters.halt_all_joints_in_cartesian_mode =
      node_parameters->get_parameter(ns + ".halt_all_joints_in_cartesian_mode").as_bool();

  // Configure handling of singularities and joint limits
  parameters.lower_singularity_threshold =
      node_parameters->get_parameter(ns + ".lower_singularity_threshold").as_double();
  parameters.hard_stop_singularity_threshold =
      node_parameters->get_parameter(ns + ".hard_stop_singularity_threshold").as_double();
  parameters.joint_limit_margin = node_parameters->get_parameter(ns + ".joint_limit_margin").as_double();

  // Collision checking
  parameters.check_collisions = node_parameters->get_parameter(ns + ".check_collisions").as_bool();
  parameters.collision_check_rate = node_parameters->get_parameter(ns + ".collision_check_rate").as_double();
  parameters.self_collision_proximity_threshold =
      node_parameters->get_parameter(ns + ".self_collision_proximity_threshold").as_double();
  parameters.scene_collision_proximity_threshold =
      node_parameters->get_parameter(ns + ".scene_collision_proximity_threshold").as_double();

  return parameters;
}

std::optional<ServoParameters> ServoParameters::validate(ServoParameters parameters)
{
  // Begin input checking
  if (parameters.publish_period <= 0.)
  {
    RCLCPP_WARN(LOGGER, "Parameter 'publish_period' should be "
                        "greater than zero. Check yaml file.");
    return std::nullopt;
  }
  if (parameters.num_outgoing_halt_msgs_to_publish < 0)
  {
    RCLCPP_WARN(LOGGER, "Parameter 'num_outgoing_halt_msgs_to_publish' should be greater than zero. Check yaml file.");
    return std::nullopt;
  }
  if (parameters.hard_stop_singularity_threshold <= parameters.lower_singularity_threshold)
  {
    RCLCPP_WARN(LOGGER, "Parameter 'hard_stop_singularity_threshold' "
                        "should be greater than 'lower_singularity_threshold.' "
                        "Check yaml file.");
    return std::nullopt;
  }
  if ((parameters.hard_stop_singularity_threshold <= 0.) || (parameters.lower_singularity_threshold <= 0.))
  {
    RCLCPP_WARN(LOGGER, "Parameters 'hard_stop_singularity_threshold' "
                        "and 'lower_singularity_threshold' should be "
                        "greater than zero. Check yaml file.");
    return std::nullopt;
  }
  if (parameters.smoothing_filter_plugin_name.empty())
  {
    RCLCPP_WARN(LOGGER, "A smoothing plugin is required.");
    return std::nullopt;
  }
  if (parameters.joint_limit_margin < 0.)
  {
    RCLCPP_WARN(LOGGER, "Parameter 'joint_limit_margin' should usually be  greater than or equal to zero, "
                        "although negative values can be used if the specified joint limits are actually soft. "
                        "Check yaml file.");
  }
  if (parameters.command_in_type != "unitless" && parameters.command_in_type != "speed_units")
  {
    RCLCPP_WARN(LOGGER, "command_in_type should be 'unitless' or "
                        "'speed_units'. Check yaml file.");
    return std::nullopt;
  }
  if (parameters.command_out_type != "trajectory_msgs/JointTrajectory" &&
      parameters.command_out_type != "std_msgs/Float64MultiArray")
  {
    RCLCPP_WARN(LOGGER, "Parameter command_out_type should be "
                        "'trajectory_msgs/JointTrajectory' or "
                        "'std_msgs/Float64MultiArray'. Check yaml file.");
    return std::nullopt;
  }
  if (!parameters.publish_joint_positions && !parameters.publish_joint_velocities &&
      !parameters.publish_joint_accelerations)
  {
    RCLCPP_WARN(LOGGER, "At least one of publish_joint_positions / "
                        "publish_joint_velocities / "
                        "publish_joint_accelerations must be true. Check "
                        "yaml file.");
    return std::nullopt;
  }
  if ((parameters.command_out_type == "std_msgs/Float64MultiArray") && parameters.publish_joint_positions &&
      parameters.publish_joint_velocities)
  {
    RCLCPP_WARN(LOGGER, "When publishing a std_msgs/Float64MultiArray, "
                        "you must select positions OR velocities.");
    return std::nullopt;
  }
  // Collision checking
  if (parameters.self_collision_proximity_threshold <= 0.)
  {
    RCLCPP_WARN(LOGGER, "Parameter 'self_collision_proximity_threshold' should be "
                        "greater than zero. Check yaml file.");
    return std::nullopt;
  }
  if (parameters.scene_collision_proximity_threshold <= 0.)
  {
    RCLCPP_WARN(LOGGER, "Parameter 'scene_collision_proximity_threshold' should be "
                        "greater than zero. Check yaml file.");
    return std::nullopt;
  }
  if (parameters.scene_collision_proximity_threshold < parameters.self_collision_proximity_threshold)
  {
    RCLCPP_WARN(LOGGER, "Parameter 'self_collision_proximity_threshold' should probably be less "
                        "than or equal to 'scene_collision_proximity_threshold'. Check yaml file.");
  }
  if (parameters.collision_check_rate <= 0)
  {
    RCLCPP_WARN(LOGGER, "Parameter 'collision_check_rate' should be "
                        "greater than zero. Check yaml file.");
    return std::nullopt;
  }
  return parameters;
}

ServoParameters::SharedConstPtr ServoParameters::makeServoParameters(const rclcpp::Node::SharedPtr& node,
                                                                     std::string ns, /* = "moveit_servo"*/
                                                                     bool dynamic_parameters /* = true */)
{
  auto node_parameters = node->get_node_parameters_interface();

  // Get the parameters
  declare(ns, node_parameters);
  auto parameters = validate(get(ns, node_parameters));

  if (parameters)
  {
    auto parameters_ptr = std::make_shared<ServoParameters>(parameters.value());
    parameters_ptr->callback_handler_ = std::make_shared<ServoParameters::CallbackHandler>();

    // register parameter change callback
    if (dynamic_parameters)
    {
      parameters_ptr->callback_handler_->on_set_parameters_callback_handler_ = node->add_on_set_parameters_callback(
          [ptr = parameters_ptr->callback_handler_.get()](const std::vector<rclcpp::Parameter>& parameters) {
            return ptr->setParametersCallback(parameters);
          });
    }

    return parameters_ptr;
  }
  return nullptr;
}

}  // namespace moveit_servo
