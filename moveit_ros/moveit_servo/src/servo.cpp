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

/*      Title     : servo.cpp
 *      Project   : moveit_servo
 *      Created   : 3/9/2017
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 */

#include <moveit_servo/make_shared_from_pool.h>
#include <moveit_servo/servo.h>

namespace moveit_servo
{
namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.servo");
constexpr double ROBOT_STATE_WAIT_TIME = 10.0;  // seconds
}  // namespace

Servo::Servo(const rclcpp::Node::SharedPtr& node, const ServoParameters::SharedConstPtr& parameters,
             const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
  : planning_scene_monitor_{ planning_scene_monitor }
  , parameters_{ parameters }
  , servo_calcs_{ node, parameters, planning_scene_monitor_ }
  , collision_checker_{ node, parameters, planning_scene_monitor_ }
{
}

void Servo::start()
{
  if (!planning_scene_monitor_->getStateMonitor()->waitForCompleteState(parameters_->move_group_name,
                                                                        ROBOT_STATE_WAIT_TIME))
  {
    RCLCPP_ERROR(LOGGER, "Timeout waiting for current state");
    return;
  }

  setPaused(false);

  // Crunch the numbers in this timer
  servo_calcs_.start();

  // Check collisions in this timer
  if (parameters_->check_collisions)
    collision_checker_.start();
}

Servo::~Servo()
{
  setPaused(true);
}

void Servo::setPaused(bool paused)
{
<<<<<<< HEAD
  servo_calcs_.setPaused(paused);
  collision_checker_.setPaused(paused);
=======
  // Load the smoothing plugin
  try
  {
    smoother_loader_ = std::make_unique<pluginlib::ClassLoader<online_signal_smoothing::SmoothingBaseClass>>(
        "moveit_core", "online_signal_smoothing::SmoothingBaseClass");
    smoother_ = smoother_loader_->createUniqueInstance(servo_params_.smoothing_filter_plugin_name);
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(logger_, "Exception while loading the smoothing plugin '%s': '%s'",
                 servo_params_.smoothing_filter_plugin_name.c_str(), ex.what());
    std::exit(EXIT_FAILURE);
  }

  // Initialize the smoothing plugin
  moveit::core::RobotStatePtr robot_state = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  const int num_joints =
      robot_state->getJointModelGroup(servo_params_.move_group_name)->getActiveJointModelNames().size();
  if (!smoother_->initialize(node_, planning_scene_monitor_->getRobotModel(), num_joints))
  {
    RCLCPP_ERROR(logger_, "Smoothing plugin could not be initialized");
    std::exit(EXIT_FAILURE);
  }
>>>>>>> 92654dd1b (Fix severe warning from class loader in servo (#3577))
}

bool Servo::getCommandFrameTransform(Eigen::Isometry3d& transform)
{
  return servo_calcs_.getCommandFrameTransform(transform);
}

bool Servo::getCommandFrameTransform(geometry_msgs::msg::TransformStamped& transform)
{
  return servo_calcs_.getCommandFrameTransform(transform);
}

bool Servo::getEEFrameTransform(Eigen::Isometry3d& transform)
{
  return servo_calcs_.getEEFrameTransform(transform);
}

bool Servo::getEEFrameTransform(geometry_msgs::msg::TransformStamped& transform)
{
  return servo_calcs_.getEEFrameTransform(transform);
}

const ServoParameters::SharedConstPtr& Servo::getParameters() const
{
  return parameters_;
}

}  // namespace moveit_servo
