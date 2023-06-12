/*******************************************************************************
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

/*      Title     : command_processor.hpp
 *      Project   : moveit_servo
 *      Created   : 06/04/2023
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson, V Mohammed Ibrahim
 */

#include <moveit_servo/command_processor.hpp>

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.command_processor");
}

namespace moveit_servo
{

CommandProcessor::CommandProcessor(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                                   const moveit::core::JointModelGroup* joint_model_group,
                                   moveit::core::RobotStatePtr& current_state, servo::Params& servo_params,
                                   StatusCode& servo_status)
  : planning_scene_monitor_{ planning_scene_monitor }
  , joint_model_group_{ joint_model_group }
  , robot_state_{ current_state }
  , servo_params_{ servo_params }
  , servo_status_{ servo_status }
{
  num_joints_ = joint_model_group_->getActiveJointModelNames().size();
  setIKSolver();
}

Eigen::VectorXd CommandProcessor::jointDeltaFromCommand(const JointJog& command)
{
  // Find the target joint position based on the commanded joint velocity
  Eigen::VectorXd joint_position_delta(num_joints_);
  joint_position_delta.setZero();

  if (isValidCommand(command))
  {
    // The incoming command should be in rad/s
    joint_position_delta = command * servo_params_.publish_period;
  }
  else
  {
    servo_status_ = StatusCode::INVALID;
    RCLCPP_WARN_STREAM(LOGGER, "Invalid joint velocity command");
  }
  return joint_position_delta;
}

Eigen::VectorXd CommandProcessor::jointDeltaFromCommand(const Twist& command)
{
  Eigen::VectorXd joint_position_delta(num_joints_);
  joint_position_delta.setZero();
  Eigen::VectorXd cartesian_position_delta;

  const bool valid_command = isValidCommand(command);
  const bool is_planning_frame = (command.frame_id == servo_params_.planning_frame);
  if (is_planning_frame && valid_command)
  {
    // Compute the Cartesian position delta based on incoming command, assumed to be in m/s
    cartesian_position_delta = command.velocities * servo_params_.publish_period;

    // Compute the required change in joint angles.
    joint_position_delta = jointDeltaFromIK(cartesian_position_delta);

    // Get velocity scaling information for singularity.
    std::pair<double, StatusCode> singularity_scaling_info =
        velocityScalingFactorForSingularity(joint_model_group_, robot_state_, cartesian_position_delta, servo_params_);
    // Apply velocity scaling for singularity, if there was any scaling.
    if (singularity_scaling_info.second != StatusCode::NO_WARNING)
    {
      servo_status_ = singularity_scaling_info.second;
      RCLCPP_WARN_STREAM(LOGGER, getStatusMessage());
      joint_position_delta *= singularity_scaling_info.first;
    }
  }
  else
  {
    servo_status_ = StatusCode::INVALID;
    if (!valid_command)
      RCLCPP_WARN_STREAM(LOGGER, "Invalid twist command.");
    if (!is_planning_frame)
      RCLCPP_WARN_STREAM(LOGGER,
                         "Command frame is: " << command.frame_id << " expected: " << servo_params_.planning_frame);
  }
  return joint_position_delta;
}

Eigen::VectorXd CommandProcessor::jointDeltaFromCommand(const Pose& command)
{
  Eigen::VectorXd joint_position_delta(num_joints_);
  joint_position_delta.setZero();

  const Eigen::Isometry3d ee_pose{ getEndEffectorPose() };
  const bool satisfies_angular_tolerance =
      ee_pose.rotation().isApprox(command.pose.rotation(), servo_params_.pose_tracking.linear_tolerance);
  const bool satisfies_linear_tolerance =
      ee_pose.translation().isApprox(command.pose.translation(), servo_params_.pose_tracking.angular_tolerance);
  const bool reached = satisfies_angular_tolerance && satisfies_linear_tolerance;

  const bool valid_command = isValidCommand(command);
  const bool is_planning_frame = command.frame_id == servo_params_.planning_frame;

  if (is_planning_frame)
  {
    if (reached)
    {
      servo_status_ = StatusCode::POSE_ACHIEVED;
    }
    else if (valid_command && !reached)
    {
      Eigen::Vector<double, 6> cartesian_position_delta;

      // Compute linear and angular change needed.
      cartesian_position_delta.head<3>() = command.pose.translation() - ee_pose.translation();

      Eigen::Quaterniond q_current(ee_pose.rotation()), q_target(command.pose.rotation());
      Eigen::Quaterniond q_error = q_target * q_current.inverse();
      Eigen::AngleAxisd angle_axis_error(q_error);
      cartesian_position_delta.tail<3>() = angle_axis_error.axis() * angle_axis_error.angle();

      // Compute the required change in joint angles.
      joint_position_delta = jointDeltaFromIK(cartesian_position_delta);
    }
  }
  else
  {
    servo_status_ = StatusCode::INVALID;
    if (!valid_command)
      RCLCPP_WARN_STREAM(LOGGER, "Invalid pose command.");
    if (!is_planning_frame)
      RCLCPP_WARN_STREAM(LOGGER,
                         "Command frame is: " << command.frame_id << " expected: " << servo_params_.planning_frame);
  }
  return joint_position_delta;
}

Eigen::VectorXd CommandProcessor::jointDeltaFromIK(const Eigen::VectorXd& cartesian_position_delta)
{
  Eigen::VectorXd delta_theta(num_joints_);

  if (ik_solver_)
  {
    std::vector<double> current_joint_positions(num_joints_);

    robot_state_->copyJointGroupPositions(joint_model_group_, current_joint_positions);

    const Eigen::Isometry3d base_to_tip_frame_transform =
        robot_state_->getGlobalLinkTransform(ik_solver_->getBaseFrame()).inverse() *
        robot_state_->getGlobalLinkTransform(ik_solver_->getTipFrame());

    geometry_msgs::msg::Pose next_pose = poseFromCartesianDelta(cartesian_position_delta, base_to_tip_frame_transform);

    // setup for IK call
    std::vector<double> solution(num_joints_);
    moveit_msgs::msg::MoveItErrorCodes err;
    kinematics::KinematicsQueryOptions opts;
    opts.return_approximate_solution = true;
    if (ik_solver_->searchPositionIK(next_pose, current_joint_positions, servo_params_.publish_period / 2.0, solution,
                                     err, opts))
    {
      // find the difference in joint positions that will get us to the desired pose
      for (size_t i = 0; i < num_joints_; ++i)
      {
        delta_theta.coeffRef(i) = solution.at(i) - current_joint_positions[i];
      }
    }
    else
    {
      RCLCPP_WARN_STREAM(LOGGER, "Could not find IK solution for requested motion, got error code " << err.val);
    }
  }
  else
  {
    // Robot does not have an IK solver, use inverse Jacobian to compute IK.
    Eigen::MatrixXd jacobian = robot_state_->getJacobian(joint_model_group_);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd =
        Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd matrix_s = svd.singularValues().asDiagonal();
    Eigen::MatrixXd pseudo_inverse = svd.matrixV() * matrix_s.inverse() * svd.matrixU().transpose();

    delta_theta = pseudo_inverse * cartesian_position_delta;
  }

  return delta_theta;
}

void CommandProcessor::setIKSolver()
{
  // Get the IK solver for the group
  ik_solver_ = joint_model_group_->getSolverInstance();
  if (!ik_solver_)
  {
    RCLCPP_WARN(
        LOGGER,
        "No kinematics solver instantiated for group '%s'. Will use inverse Jacobian for servo calculations instead.",
        joint_model_group_->getName().c_str());
  }
  else if (!ik_solver_->supportsGroup(joint_model_group_))
  {
    ik_solver_ = nullptr;
    RCLCPP_WARN(LOGGER,
                "The loaded kinematics plugin does not support group '%s'. Will use inverse Jacobian for servo "
                "calculations instead.",
                joint_model_group_->getName().c_str());
  }
  else
  {
    RCLCPP_INFO(LOGGER, "IK solver available for robot, will use it.");
  }
}

StatusCode CommandProcessor::getStatus()
{
  return servo_status_;
}

const std::string CommandProcessor::getStatusMessage()
{
  return SERVO_STATUS_CODE_MAP.at(servo_status_);
}

const Eigen::Isometry3d CommandProcessor::getEndEffectorPose()
{
  // Robot base (panda_link0) to end effector frame (panda_link8)
  robot_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  return robot_state_->getGlobalLinkTransform(servo_params_.ee_frame);
}

}  // namespace moveit_servo
