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

#include <moveit_servo/servo.h>

namespace moveit_servo
{
Servo::Servo(const rclcpp::Node::SharedPtr& node, ServoParametersPtr parameters,
             planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor)
  : planning_scene_monitor_(planning_scene_monitor), parameters_(parameters)
{
  servo_calcs_ = std::make_unique<ServoCalcs>(node, parameters, planning_scene_monitor_);

  collision_checker_ = std::make_unique<CollisionCheck>(node, parameters, planning_scene_monitor_);
}

bool Servo::start()
{
  // Crunch the numbers in this timer
  if (servo_calcs_->start())
    setPaused(false);
  else
    return false;

  // Check collisions in this timer
  if (parameters_->check_collisions)
    collision_checker_->start();

  return true;
}

void Servo::stop()
{
  servo_calcs_->stop();
  collision_checker_->stop();
}

Servo::~Servo()
{
  stop();
}

void Servo::setPaused(bool paused)
{
  servo_calcs_->setPaused(paused);
  collision_checker_->setPaused(paused);
}

bool Servo::waitForInitialized(std::chrono::duration<double> wait_for)
{
  return servo_calcs_->waitForInitialized(wait_for);
}

bool Servo::getCommandFrameTransform(Eigen::Isometry3d& transform)
{
  return servo_calcs_->getCommandFrameTransform(transform);
}

const std::shared_ptr<moveit_servo::ServoParameters>& Servo::getParameters() const
{
  return parameters_;
}

sensor_msgs::msg::JointState::ConstSharedPtr Servo::getLatestJointState() const
{
  return servo_calcs_->getLatestJointState();
}

}  // namespace moveit_servo
