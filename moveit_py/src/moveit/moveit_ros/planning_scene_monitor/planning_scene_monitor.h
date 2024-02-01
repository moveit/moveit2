/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Peter David Fagan
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

/* Author: Peter David Fagan */

#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <moveit_py/moveit_py_utils/copy_ros_msg.h>
#include <moveit_py/moveit_py_utils/ros_msg_typecasters.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace py = pybind11;

namespace moveit_py
{
namespace bind_planning_scene_monitor
{
class LockedPlanningSceneContextManagerRW
{
public:
  const planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  std::unique_ptr<const planning_scene_monitor::LockedPlanningSceneRW> ls_rw_;

  LockedPlanningSceneContextManagerRW(const planning_scene_monitor::PlanningSceneMonitorPtr& psm)
    : planning_scene_monitor_(psm)
  {
    ls_rw_ = std::make_unique<const planning_scene_monitor::LockedPlanningSceneRW>(planning_scene_monitor_);
  }

  const planning_scene::PlanningScenePtr& lockedPlanningSceneRwEnter();

  void lockedPlanningSceneRwExit(const py::object& type, const py::object& value, const py::object& traceback);
};

class LockedPlanningSceneContextManagerRO
{
public:
  const planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  std::unique_ptr<const planning_scene_monitor::LockedPlanningSceneRO> ls_ro_;

  LockedPlanningSceneContextManagerRO(const planning_scene_monitor::PlanningSceneMonitorPtr& psm)
    : planning_scene_monitor_(psm)
  {
    ls_ro_ = std::make_unique<const planning_scene_monitor::LockedPlanningSceneRO>(planning_scene_monitor_);
  }

  const planning_scene::PlanningSceneConstPtr& lockedPlanningSceneRoEnter() const;

  void lockedPlanningSceneRoExit(const py::object& type, const py::object& value, const py::object& traceback);
};

LockedPlanningSceneContextManagerRW
readWrite(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);

LockedPlanningSceneContextManagerRO
readOnly(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);

void initPlanningSceneMonitor(py::module& m);

void initContextManagers(py::module& m);
}  // namespace bind_planning_scene_monitor
}  // namespace moveit_py
