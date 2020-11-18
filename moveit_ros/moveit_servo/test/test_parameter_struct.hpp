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

/*      Title     : test_parameter_struct.hpp
 *      Project   : moveit_servo
 *      Created   : 07/23/2020
 *      Author    : Adam Pettinger
 *      Desc      : Returns a ServoParameters object for testing
 */

#pragma once

#include <moveit_servo/servo_parameters.h>

/** \brief Returns a shared ptr to a ServoParams object with valid (and 'default') fields
 * A ServoParameters of this type should match what is loaded from config/panda_simulated_config.yaml
 */
moveit_servo::ServoParametersPtr getTestParameters()
{
  auto output = std::make_shared<moveit_servo::ServoParameters>();

  // Populate the fields
  output->use_gazebo = true;
  output->status_topic = "~/status";
  output->cartesian_command_in_topic = "~/delta_twist_cmds";
  output->joint_command_in_topic = "~/delta_joint_cmds";
  output->robot_link_command_frame = "panda_link0";
  output->command_in_type = "unitless";
  output->linear_scale = 0.4;
  output->rotational_scale = 0.8;
  output->joint_scale = 0.5;
  output->command_out_topic = "/fake_joint_trajectory_controller/joint_trajectory";
  output->publish_period = 0.034;
  output->command_out_type = "trajectory_msgs/JointTrajectory";
  output->publish_joint_positions = true;
  output->publish_joint_velocities = false;
  output->publish_joint_accelerations = false;
  output->joint_topic = "/joint_states";
  output->low_pass_filter_coeff = 2;
  output->move_group_name = "panda_arm";
  output->ee_frame_name = "panda_link8";
  output->planning_frame = "panda_link0";
  output->incoming_command_timeout = 0.1;
  output->num_outgoing_halt_msgs_to_publish = 4;
  output->lower_singularity_threshold = 17;
  output->hard_stop_singularity_threshold = 30;
  output->joint_limit_margin = 0.1;
  output->check_collisions = true;
  output->collision_check_rate = 10;
  output->collision_check_type = "threshold_distance";
  output->self_collision_proximity_threshold = 0.01;
  output->scene_collision_proximity_threshold = 0.02;
  output->collision_distance_safety_factor = 1000;
  output->min_allowable_collision_distance = 0.01;

  return output;
}

/** \brief Checks 2 ServoParameter objects for equality
 */
bool operator==(moveit_servo::ServoParameters& lhs, moveit_servo::ServoParameters& rhs)
{
  return (lhs.use_gazebo == rhs.use_gazebo && lhs.status_topic == rhs.status_topic &&
          lhs.cartesian_command_in_topic == rhs.cartesian_command_in_topic &&
          lhs.joint_command_in_topic == rhs.joint_command_in_topic &&
          lhs.robot_link_command_frame == rhs.robot_link_command_frame && lhs.command_in_type == rhs.command_in_type &&
          lhs.linear_scale == rhs.linear_scale && lhs.rotational_scale == rhs.rotational_scale &&
          lhs.joint_scale == rhs.joint_scale && lhs.command_out_topic == rhs.command_out_topic &&
          lhs.publish_period == rhs.publish_period && lhs.command_out_type == rhs.command_out_type &&
          lhs.publish_joint_positions == rhs.publish_joint_positions &&
          lhs.publish_joint_velocities == rhs.publish_joint_velocities &&
          lhs.publish_joint_accelerations == rhs.publish_joint_accelerations && lhs.joint_topic == rhs.joint_topic &&
          lhs.low_pass_filter_coeff == rhs.low_pass_filter_coeff && lhs.move_group_name == rhs.move_group_name &&
          lhs.ee_frame_name == rhs.ee_frame_name && lhs.planning_frame == rhs.planning_frame &&
          lhs.incoming_command_timeout == rhs.incoming_command_timeout &&
          lhs.num_outgoing_halt_msgs_to_publish == rhs.num_outgoing_halt_msgs_to_publish &&
          lhs.lower_singularity_threshold == rhs.lower_singularity_threshold &&
          lhs.hard_stop_singularity_threshold == rhs.hard_stop_singularity_threshold &&
          lhs.joint_limit_margin == rhs.joint_limit_margin && lhs.check_collisions == rhs.check_collisions &&
          lhs.collision_check_rate == rhs.collision_check_rate &&
          lhs.collision_check_type == rhs.collision_check_type &&
          lhs.self_collision_proximity_threshold == rhs.self_collision_proximity_threshold &&
          lhs.scene_collision_proximity_threshold == rhs.scene_collision_proximity_threshold &&
          lhs.collision_distance_safety_factor == rhs.collision_distance_safety_factor &&
          lhs.min_allowable_collision_distance == rhs.min_allowable_collision_distance);
}

bool operator!=(moveit_servo::ServoParameters& lhs, moveit_servo::ServoParameters& rhs)
{
  return !(lhs == rhs);
}
