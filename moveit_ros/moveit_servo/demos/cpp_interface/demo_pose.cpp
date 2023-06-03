/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, PickNik Inc.
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

/*      Title     : demo_pose.cpp
 *      Project   : moveit_servo
 *      Created   : 07/06/2023
 *      Author    : V Mohammed Ibrahim
 *      Description : Example of controlling a robot through pose commands via the C++ API.
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils.hpp>

using namespace moveit_servo;

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.pose_demo");
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // The servo object expects to get a ROS node.
  auto demo_node = std::make_shared<rclcpp::Node>("moveit_servo_demo");

  // Get the servo parameters.
  std::string param_namespace = "moveit_servo";
  auto servo_param_listener = std::make_shared<const servo::ParamListener>(demo_node, param_namespace);
  auto servo_params = servo_param_listener->get_params();

  // The publisher to send trajectory message to the robot controller.
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_outgoing_cmd_pub =
      demo_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(servo_params.command_out_topic,
                                                                         rclcpp::SystemDefaultsQoS());

  // Create the servo object
  auto servo = Servo(demo_node, servo_param_listener);

  // Wait for some time, so that we can actually see when the robot moves.
  // This is just for convenience, should not be used for sync in real application.
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // Frquency at which the commands will be send to robot controller.
  rclcpp::WallRate rate(1.0 / servo_params.publish_period);

  // Set the command type for servo.
  servo.incomingCommandType(CommandType::POSE);

  // 1.  The workflow for when command is not in planning frame.
  Pose target_pose_ee_frame;
  target_pose_ee_frame.frame_id = servo_params.ee_frame_name;
  target_pose_ee_frame.pose.setIdentity();
  // Set the target pose to +10 cm in the z direction in ee frame.
  target_pose_ee_frame.pose.translate(Eigen::Vector3d(0.0, 0.0, 0.1));
  // Set rotation to +45 degrees from current angle about z
  target_pose_ee_frame.pose.rotate(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
  // Servo only accepts commands in planning_frame, so use convert it before sending.
  target_pose_ee_frame = servo.toPlanningFrame(target_pose_ee_frame);

  // 2. The workflow for when command is in planning frame.
  Pose target_pose_planning_frame;
  target_pose_planning_frame.frame_id = servo_params.planning_frame;
  // Set position to 10 cm in +z direction from current ee frame position in planning frame.
  target_pose_planning_frame.pose = servo.getEndEffectorPose();
  target_pose_planning_frame.pose.translate(Eigen::Vector3d(0.0, 0.0, -0.1));
  // Set angle to 45 degree from current angle about z, in planning frame.
  target_pose_planning_frame.pose.rotate(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()));

  // Set tolerances for the target.
  const double linear_tolerance{ 0.001 }, angular_tolerance{ 0.01 };

  auto move_to_pose = [&](const auto& target_pose) {
    bool reached = false;
    while (rclcpp::ok() && servo.getStatus() == StatusCode::NO_WARNING)
    {
      const bool satisfies_angular_tolerance =
          servo.getEndEffectorPose().rotation().isApprox(target_pose.pose.rotation(), angular_tolerance);
      const bool satisfies_linear_tolerance =
          servo.getEndEffectorPose().translation().isApprox(target_pose.pose.translation(), linear_tolerance);
      reached = satisfies_angular_tolerance && satisfies_linear_tolerance;
      if (reached)
      {
        break;
      }
      else
      {
        auto joint_state = servo.getNextJointState(target_pose);
        // Send the command to robot controller only if the command was valid.
        if (servo.getStatus() != StatusCode::INVALID)
        {
          auto joint_trajectory = composeTrajectoryMessage(servo_params, joint_state);
          trajectory_outgoing_cmd_pub->publish(joint_trajectory);
        }
      }
      rate.sleep();
    }
    servo.resetPoseControllers();
    return reached;
  };

  // Move down and then up.
  RCLCPP_INFO_STREAM(LOGGER, servo.getStatusMessage());
  if (move_to_pose(target_pose_ee_frame))
  {
    RCLCPP_INFO_STREAM(LOGGER, "End-Effector frame target Pose achieved");
  }
  if (move_to_pose(target_pose_planning_frame))
  {
    RCLCPP_INFO_STREAM(LOGGER, "Planning frame target pose achieved");
  }
  RCLCPP_INFO_STREAM(LOGGER, servo.getStatusMessage());
  rclcpp::shutdown();
}
