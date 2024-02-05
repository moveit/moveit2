/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik Inc.
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

/* Author: Sebastian Jahr, Adam Pettinger
 */

#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit/local_constraint_solver_plugins/servo_solver.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/utils/logger.hpp>
#include <local_planner_parameters.hpp>

namespace moveit::hybrid_planning
{
namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("servo_solver");
}
}  // namespace

bool ServoSolver::initialize(const rclcpp::Node::SharedPtr& node,
                             const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                             const std::string& /*group_name*/)
{
  planning_scene_monitor_ = planning_scene_monitor;
  node_ = node;

  const std::shared_ptr<const servo_solver_parameters::ParamListener> solver_param_listener =
      std::make_shared<const servo_solver_parameters::ParamListener>(node, "");
  solver_parameters_ = solver_param_listener->get_params();

  // Get Servo Parameters
  const std::string param_namespace = "moveit_servo";
  const std::shared_ptr<const servo::ParamListener> servo_param_listener =
      std::make_shared<const servo::ParamListener>(node, param_namespace);
  servo_parameters_ = servo_param_listener->get_params();

  // Create Servo and start it
  servo_ = std::make_unique<moveit_servo::Servo>(node_, servo_param_listener, planning_scene_monitor_);

  // Set servo publish_period
  auto local_param_listener = local_planner_parameters::ParamListener(node_, "");
  const auto local_config = local_param_listener.get_params();

  // Publish period is the time difference used for numerical integration. It is recommended that the period 3x smaller
  // than the actual publish period.
  const double servo_publish_rate = 1.0 / 3.0 * local_config.local_planning_frequency;
  if (!node_->has_parameter("publish_period"))
  {
    RCLCPP_ERROR(getLogger(), "Node used by servo solver doesn't seem to have a parameter called 'publish_period' that "
                              "shouldn't happen!");
    return false;
  }
  node_->set_parameter(rclcpp::Parameter("publish_period", servo_publish_rate));
  return true;
}

bool ServoSolver::reset()
{
  RCLCPP_INFO(getLogger(), "Reset Servo Solver");
  joint_cmd_rolling_window_.clear();
  return true;
};

moveit_msgs::action::LocalPlanner::Feedback
ServoSolver::solve(const robot_trajectory::RobotTrajectory& local_trajectory,
                   const std::shared_ptr<const moveit_msgs::action::LocalPlanner::Goal> /*local_goal*/,
                   trajectory_msgs::msg::JointTrajectory& local_solution)
{
  // Create Feedback
  moveit_msgs::action::LocalPlanner::Feedback feedback_result;

  // Transform next robot trajectory waypoint into JointJog message
  moveit_msgs::msg::RobotTrajectory robot_command;
  local_trajectory.getRobotTrajectoryMsg(robot_command);

  if (robot_command.joint_trajectory.points.empty())
  {
    feedback_result.feedback = std::string("Reference trajectory does not contain any points");
    return feedback_result;
  }

  // Get current state
  const auto current_state = planning_scene_monitor_->getStateMonitor()->getCurrentState();

  // Create goal state
  moveit::core::RobotState target_state = *current_state;
  target_state.setVariablePositions(robot_command.joint_trajectory.joint_names,
                                    robot_command.joint_trajectory.points.at(0).positions);
  target_state.update();

  // TF planning_frame -> current EE
  Eigen::Isometry3d current_pose = current_state->getFrameTransform(solver_parameters_.reference_frame);
  // TF planning -> target EE
  Eigen::Isometry3d target_pose = target_state.getFrameTransform(solver_parameters_.reference_frame);

  // current EE -> planning frame * planning frame -> target EE
  Eigen::Isometry3d diff_pose = current_pose.inverse() * target_pose;
  Eigen::AngleAxisd axis_angle(diff_pose.linear());

  const double trans_gain = solver_parameters_.trans_gain_scaling / diff_pose.translation().norm();
  const double rot_gain = solver_parameters_.rot_gain_scaling / diff_pose.rotation().norm();

  // Calculate Cartesian command delta
  // Transform current pose to command frame
  // Transform goal pose to command frame
  servo_->setCommandType(moveit_servo::CommandType::TWIST);
  moveit_servo::TwistCommand target_twist{
    solver_parameters_.reference_frame,
    { diff_pose.translation().x() * trans_gain, diff_pose.translation().y() * trans_gain,
      diff_pose.translation().z() * trans_gain, axis_angle.axis().x() * axis_angle.angle() * rot_gain,
      axis_angle.axis().y() * axis_angle.angle() * rot_gain, axis_angle.axis().z() * axis_angle.angle() * rot_gain }
  };

  std::optional<trajectory_msgs::msg::JointTrajectory> trajectory_msg;

  // Clear all commands that are older than older than one publish period
  const auto cutoff_timestamp = node_->now() - rclcpp::Duration::from_seconds(3.0 * servo_parameters_.publish_period);
  const auto cutoff_iterator = std::find_if(joint_cmd_rolling_window_.rbegin(), joint_cmd_rolling_window_.rend(),
                                            [&cutoff_timestamp](moveit_servo::KinematicState state) {
                                              return state.time_stamp < cutoff_timestamp;
                                            })
                                   .base();

  if (cutoff_iterator != joint_cmd_rolling_window_.end())
  {
    // Erase elements from the beginning to the found position
    joint_cmd_rolling_window_.erase(joint_cmd_rolling_window_.begin(), cutoff_iterator);
  }

  // Create servo commands until a trajectory message can be generated
  while (!trajectory_msg)
  {
    // Calculate next servo command
    moveit_servo::KinematicState joint_state = servo_->getNextJointState(current_state, target_twist);
    const auto status = servo_->getStatus();
    // Servo solver feedback is always the status of the first servo iteration
    if (feedback_result.feedback.empty() && status != moveit_servo::StatusCode::NO_WARNING)
    {
      feedback_result.feedback = moveit_servo::SERVO_STATUS_CODE_MAP.at(status);
    }
    // If servo couldn't compute the next joint state, exit local solver without a solution
    if (status == moveit_servo::StatusCode::INVALID)
    {
      return feedback_result;
    }
    moveit_servo::updateSlidingWindow(joint_state, joint_cmd_rolling_window_, servo_parameters_.max_expected_latency,
                                      node_->now());
    if (!joint_cmd_rolling_window_.empty())
    {
      current_state->setJointGroupPositions(current_state->getJointModelGroup(servo_parameters_.move_group_name),
                                            joint_cmd_rolling_window_.back().positions);
      current_state->setJointGroupVelocities(current_state->getJointModelGroup(servo_parameters_.move_group_name),
                                             joint_cmd_rolling_window_.back().velocities);
    }
    trajectory_msg = moveit_servo::composeTrajectoryMessage(servo_parameters_, joint_cmd_rolling_window_);
  }
  local_solution = trajectory_msg.value();
  return feedback_result;
}
}  // namespace moveit::hybrid_planning

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(moveit::hybrid_planning::ServoSolver, moveit::hybrid_planning::LocalConstraintSolverInterface);
