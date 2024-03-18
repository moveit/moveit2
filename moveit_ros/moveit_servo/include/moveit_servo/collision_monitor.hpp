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
/*
 * Title      : collision_monitor.hpp
 * Project    : moveit_servo
 * Created    : 06/08/2023
 * Author     : Brian O'Neil, Andy Zelenak, Blake Anderson, V Mohammed Ibrahim
 *
 * Description: Monitors the planning scene for collision and publishes the velocity scaling.
 */

#pragma once

#include <moveit_servo_lib_parameters.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>

namespace moveit_servo
{

class CollisionMonitor
{
public:
  CollisionMonitor(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                   const servo::Params& servo_params, std::atomic<double>& collision_velocity_scale);

  void start();

  void stop();

private:
  /**
   * \brief The collision checking function, this will run in a separate thread.
   */
  void checkCollisions();

  // Variables

  const servo::Params& servo_params_;

  const planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  moveit::core::RobotState robot_state_;

  // The collision monitor thread.
  std::thread monitor_thread_;
  // The flag used for stopping the collision monitor thread.
  std::atomic<bool> stop_requested_;

  // The scaling factor when approaching a collision.
  std::atomic<double>& collision_velocity_scale_;

  // The data structures used to get information about robot self collisions.
  collision_detection::CollisionRequest self_collision_request_;
  collision_detection::CollisionResult self_collision_result_;
  // The data structures used to get information about robot collision with other objects in the collision scene.
  collision_detection::CollisionRequest scene_collision_request_;
  collision_detection::CollisionResult scene_collision_result_;
};

}  // namespace moveit_servo
