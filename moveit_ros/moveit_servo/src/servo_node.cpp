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

/*      Title     : servo_node.cpp
 *      Project   : moveit_servo
 *      Created   : 12/31/2018
 *      Author    : Andy Zelenak
 */

#include <moveit_servo/servo_node.h>
#include <moveit_servo/servo_parameters.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.servo_node");

namespace moveit_servo
{
ServoNode::ServoNode(const rclcpp::NodeOptions& options) : Node("servo_node", options), is_initialized_(false)
{
  if (!options.use_intra_process_comms())
  {
    RCLCPP_WARN_STREAM(LOGGER, "Intra-process communication is disabled, consider enabling it by adding: "
                               "\nextra_arguments=[{'use_intra_process_comms' : True}]\nto the Servo composable node "
                               "in the launch file");
  }

  // Set up services for interacting with Servo
  using std::placeholders::_1;
  using std::placeholders::_2;
  start_servo_service_ =
      this->create_service<std_srvs::srv::Trigger>("~/start_servo", std::bind(&ServoNode::startCB, this, _1, _2));
  stop_servo_service_ =
      this->create_service<std_srvs::srv::Trigger>("~/stop_servo", std::bind(&ServoNode::stopCB, this, _1, _2));
  pause_servo_service_ =
      this->create_service<std_srvs::srv::Trigger>("~/pause_servo", std::bind(&ServoNode::pauseCB, this, _1, _2));
  unpause_servo_service_ =
      this->create_service<std_srvs::srv::Trigger>("~/unpause_servo", std::bind(&ServoNode::unpauseCB, this, _1, _2));
}

bool ServoNode::init()
{
  bool performed_initialization = true;

  // Can set robot_description name from parameters
  std::string robot_description_name = "robot_description";
  this->get_parameter_or("robot_description_name", robot_description_name, robot_description_name);

  // Set up planning_scene_monitor
  auto node_ptr = shared_from_this();
  planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node_ptr, robot_description_name, "planning_scene_monitor");

  // Get the servo parameters
  auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node_ptr, LOGGER);
  if (servo_parameters == nullptr)
  {
    RCLCPP_FATAL(LOGGER, "Failed to load the servo parameters");
    performed_initialization = false;
  }

  // Start the planning scene monitor
  performed_initialization &= (planning_scene_monitor_->getPlanningScene() != nullptr);
  if (performed_initialization)
  {
    planning_scene_monitor_->startStateMonitor(servo_parameters->joint_topic);
    planning_scene_monitor_->setPlanningScenePublishingFrequency(25);
    planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                          "/moveit_servo/publish_planning_scene");
    planning_scene_monitor_->startSceneMonitor();
  }
  else
  {
    RCLCPP_FATAL(LOGGER, "Planning scene not configured");
  }

  // If we initialized properly, go ahead and start everything up
  if (performed_initialization)
  {
    // Create Servo
    servo_ = std::make_unique<moveit_servo::Servo>(node_ptr, servo_parameters, planning_scene_monitor_);
    is_initialized_ = true;
    servo_->start();
    return true;
  }
  else
  {
    RCLCPP_FATAL(LOGGER, "Servo Service failed to initialize properly, not starting servoing");
    return false;
  }
}

void ServoNode::reset()
{
  servo_.reset();
  planning_scene_monitor_.reset();
  is_initialized_ = false;
}

void ServoNode::startCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  // If we already initialized, reset servo before initializing again
  if (is_initialized_)
    reset();

  response->success = init();
}

void ServoNode::stopCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  reset();
  response->success = true;
}

void ServoNode::pauseCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (servo_)
    servo_->setPaused(true);

  response->success = true;
}

void ServoNode::unpauseCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (servo_)
    servo_->setPaused(false);

  response->success = true;
}

}  // namespace moveit_servo

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit_servo::ServoNode)
