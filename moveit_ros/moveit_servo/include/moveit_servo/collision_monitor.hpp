/*********************************************************************************
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
 * FOR ANY DIRECT, INDIRECT, ICIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVERb
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

/*      Title       : collision_monitor.hpp
 *      Project     : moveit_servo
 *      Created     : 06/08/2023
 *      Author      : Brian O'Neil, Andy Zelenak, Blake Anderson, V Mohammed Ibrahim
 *
 *      Description : A collision monitor thread that reads the current collision scene
 *                    that updates the collision velocity scale.
 */

#pragma once

#include <moveit_servo/servo.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>

namespace moveit_servo
{

/**
 * \brief A collision monitor thread that reads the current collision scene
 *        and updates the collision velocity scale.
 */
class CollisionMonitor
{
public:
  /**
   * \brief Constructor for CollisionMonitor.
   * \param planning_scene_monitor Pointer to the planning scene monitor.
   * \param servo_params Reference to the servo parameters.
   * \param collision_velocity_scale Reference to the atomic double storing the collision velocity scale.
   */
  CollisionMonitor(const planning_scene_monitor::Planning_SceneMonitorPtr& planning_scene_monitor,
                   const servo::Params& servo_params, std::atomic<double>& collision_velocity_scale);

  /**
   * \brief Starts the collision monitor thread.
   */
  void start();

  /**
   * \brief Stops the collision monitor thread.
   */
  void stop();

  // Disable copy construction.
  CollisionMonitor(const CollisionMonitor&) = delete;

  // Disable copy assignment.
  CollisionMonitor& operator=(CollisionMonitor&) = delete;

private:
  /**
   * \brief The collision monitor loop.
   */
  void checkCollisions();

  const planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  const servo::Params& servo_params_;

  // Thread controls
  std::atomic<bool> stop_collision_monitor_;
  std::thread collision_monitor_thread_;

  // The collision velocity`scale updated by the thread
  std::atomic<double>& collision_velocity_scale_;
};

}  // namespace moveit_servo
