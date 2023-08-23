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

#include "planning_scene_monitor.h"

namespace moveit_py
{
namespace bind_planning_scene_monitor
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_py.bind_planning_scene_monitor");

LockedPlanningSceneContextManagerRO
read_only(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
{
  return LockedPlanningSceneContextManagerRO(planning_scene_monitor);
};

LockedPlanningSceneContextManagerRW
read_write(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
{
  return LockedPlanningSceneContextManagerRW(planning_scene_monitor);
};

const planning_scene::PlanningSceneConstPtr& LockedPlanningSceneContextManagerRO::lockedPlanningSceneRoEnter() const
{
  return static_cast<const planning_scene_monitor::PlanningSceneMonitor*>(planning_scene_monitor_.get())
      ->getPlanningScene();
}

const planning_scene::PlanningScenePtr& LockedPlanningSceneContextManagerRW::lockedPlanningSceneRwEnter()
{
  return planning_scene_monitor_->getPlanningScene();
}

void LockedPlanningSceneContextManagerRO::lockedPlanningSceneRoExit(const py::object& /*type*/,
                                                                    const py::object& /*value*/,
                                                                    const py::object& /*traceback*/)
{
  ls_ro_.reset();
}

void LockedPlanningSceneContextManagerRW::lockedPlanningSceneRwExit(const py::object& /*type*/,
                                                                    const py::object& /*value*/,
                                                                    const py::object& /*traceback*/)
{
  ls_rw_.reset();
}

// TODO: simplify with typecaster
void apply_planning_scene(std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor>& planning_scene_monitor,
                          const moveit_msgs::msg::PlanningScene& planning_scene)
{
  // lock planning scene
  {
    planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor);
    scene->usePlanningSceneMsg(planning_scene);
  }
}

void init_planning_scene_monitor(py::module& m)
{
  py::class_<planning_scene_monitor::PlanningSceneMonitor, planning_scene_monitor::PlanningSceneMonitorPtr>(
      m, "PlanningSceneMonitor", R"(
      Maintains the internal state of the planning scene.
      )")

      .def_property("name", &planning_scene_monitor::PlanningSceneMonitor::getName, nullptr,
                    R"(
                    str: The name of this planning scene monitor.
                    )")

      .def("start_scene_monitor", &planning_scene_monitor::PlanningSceneMonitor::startSceneMonitor,
           R"(
           Starts the scene monitor.
           )")

      .def("stop_scene_monitor", &planning_scene_monitor::PlanningSceneMonitor::stopSceneMonitor,
           R"(
           Stops the scene monitor.
           )")

      .def("start_state_monitor", &planning_scene_monitor::PlanningSceneMonitor::startStateMonitor,
           R"(
	   Starts the state monitor.
	   )")

      .def("stop_state_monitor", &planning_scene_monitor::PlanningSceneMonitor::stopStateMonitor,
           R"(
	       Stops the state monitor.
	   )")

      .def("wait_for_current_robot_state", &planning_scene_monitor::PlanningSceneMonitor::waitForCurrentRobotState,
           R"(
	   Waits for the current robot state to be received.
	   )")

      .def("clear_octomap", &planning_scene_monitor::PlanningSceneMonitor::clearOctomap,
           R"(
           Clears the octomap.
           )")

      .def("read_only", &moveit_py::bind_planning_scene_monitor::read_only,
           R"(
           Returns a read-only context manager for the planning scene.
           )")

      .def("read_write", &moveit_py::bind_planning_scene_monitor::read_write,
           R"(
           Returns a read-write context manager for the planning scene.
           )");
}

void init_context_managers(py::module& m)
{
  // In Python we lock the planning scene using a with statement as this allows us to have control over resources.
  // To this end each of the below manager classes binds special methods __enter__ and __exit__.
  // LockedPlanningSceneContextManagerRO
  py::class_<moveit_py::bind_planning_scene_monitor::LockedPlanningSceneContextManagerRO>(
      m, "LockedPlanningSceneContextManagerRO", R"(
      A context manager that locks the planning scene for reading.
      )")

      .def("__enter__",
           &moveit_py::bind_planning_scene_monitor::LockedPlanningSceneContextManagerRO::lockedPlanningSceneRoEnter,
           R"(
           Special method that is used with the with statement, provides access to a locked plannning scene instance.
           Returns:
               :py:class:`moveit_py.core.PlanningScene`: The locked planning scene.
        )")
      .def("__exit__",
           &moveit_py::bind_planning_scene_monitor::LockedPlanningSceneContextManagerRO::lockedPlanningSceneRoExit,
           R"(
           Special method that is used with the with statement, releases the lock on the planning scene.
           )");

  // LockedPlanningSceneContextManagerRW
  py::class_<moveit_py::bind_planning_scene_monitor::LockedPlanningSceneContextManagerRW>(
      m, "LockedPlanningSceneContextManagerRW", R"(
      A context manager that locks the planning scene for reading and writing.
      )")

      .def("__enter__",
           &moveit_py::bind_planning_scene_monitor::LockedPlanningSceneContextManagerRW::lockedPlanningSceneRwEnter,
           py::return_value_policy::take_ownership,
           R"(
           Special method that is used with the with statement, provides access to a locked plannning scene instance.
           Returns:
               :py:class:`moveit_py.core.PlanningScene`: The locked planning scene.
           )")

      .def("__exit__",
           &moveit_py::bind_planning_scene_monitor::LockedPlanningSceneContextManagerRW::lockedPlanningSceneRwExit,
           R"(
           Special method that is used with the with statement, releases the lock on the planning scene.
           )");
}
}  // namespace bind_planning_scene_monitor
}  // namespace moveit_py
