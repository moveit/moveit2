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

/*      Title     : servo_server.cpp
 *      Project   : moveit_servo
 *      Created   : 12/31/2018
 *      Author    : Andy Zelenak
 */

#include <moveit_servo/servo_server.h>
#include <moveit_servo/servo_parameters.cpp>

// namespace
// {
// constexpr char LOGNAME[] = "servo_server";
// constexpr char ROS_THREADS = 8;

// }  // namespace

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.servo_server");

namespace moveit_servo
{

ServoServer::ServoServer(const rclcpp::NodeOptions& options)
  : Node("servo_service", options)
{
  if(!options.use_intra_process_comms())
  {
    RCLCPP_WARN_STREAM(LOGGER, "Intra-process communication is disabled, consider enabling it with: \n something here"); //TODO(adamp): figure out the instructions for enabling and update here
  }
  // Create tf buffer and planning scene monitor

  // Set up services for interacting with Servo
  using std::placeholders::_1;
  using std::placeholders::_2;
  start_servo_service_ = this->create_service<std_srvs::srv::Trigger>("start_servo", std::bind(&ServoServer::startCB, this, _1, _2));
  stop_servo_service_ = this->create_service<std_srvs::srv::Trigger>("stop_servo", std::bind(&ServoServer::stopCB, this, _1, _2));
  pause_servo_service_ = this->create_service<std_srvs::srv::Trigger>("pause_servo", std::bind(&ServoServer::pauseCB, this, _1, _2));
  unpause_servo_service_ = this->create_service<std_srvs::srv::Trigger>("unpause_servo", std::bind(&ServoServer::unpauseCB, this, _1, _2));

  // Set up the planning scene, get the parameters, and start servoing
  initialization_timer_ = this->create_wall_timer(std::chrono::duration<double>(2.0), std::bind(&ServoServer::initializeServo, this));
}

void ServoServer::initializeServo()
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto node_ptr = shared_from_this();
  planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_ptr, "robot_description", tf_buffer_, "planning_scene_monitor"); //TODO(adamp) parameterize this

  // Get the parameters
  bool initialized_properly = false;
  auto servo_parameters = std::make_shared<moveit_servo::ServoParameters>();
  if(moveit_servo::readParameters(*this, LOGGER, *servo_parameters))
  {
    initialized_properly = true;
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Could not get parameters");
  }

  // Start the planning scene monitor
  if (planning_scene_monitor_->getPlanningScene())
  {
    // TODO(adamp): parameterize this
    planning_scene_monitor_->startStateMonitor("/joint_states");
    planning_scene_monitor_->setPlanningScenePublishingFrequency(25);
    planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE, "/moveit_servo/publish_planning_scene");
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Planning scene not configured");
    initialized_properly = false;
  }

  // Create Servo
  servo_ = std::make_unique<moveit_servo::Servo>(node_ptr, servo_parameters, planning_scene_monitor_);
  
  // If we initialized properly, go ahead and start everything up
  if(initialized_properly)
  {
    servo_->start();
    initialization_timer_->cancel();
  }
  else
  {
    RCLCPP_WARN(LOGGER, "Servo Service failed to initialize properly, not starting servoing");
  }
}

void ServoServer::startCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  servo_->start();
  response->success = true;
}

void ServoServer::stopCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  servo_->stop();
  response->success = true;
}

void ServoServer::pauseCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  servo_->setPaused(true);
  response->success = true;
}

void ServoServer::unpauseCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  servo_->setPaused(false);
  response->success = true;
}

} // namespace moveit_servo

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit_servo::ServoServer)