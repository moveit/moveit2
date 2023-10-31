/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Matthijs van der Burgh
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
 *   * Neither the name of the copyright holder nor the names of its
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

/* Author: Matthijs van der Burgh */

#include "trajectory_execution_manager.h"

namespace moveit_py
{
namespace bind_trajectory_execution_manager
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_py.bind_trajectory_execution_manager");

void initTrajectoryExecutionManager(py::module& m)
{
  py::class_<trajectory_execution_manager::TrajectoryExecutionManager,
             trajectory_execution_manager::TrajectoryExecutionManagerPtr>(m, "TrajectoryExecutionManager", R"(
      Manages the trajectory execution.
      )")

      .def("is_managing_controllers", &trajectory_execution_manager::TrajectoryExecutionManager::isManagingControllers,
           R"(
           If this function returns true, then this instance of the manager is allowed to load/unload/switch controllers.
           )")

      .def("process_event", &trajectory_execution_manager::TrajectoryExecutionManager::processEvent, py::arg("event"),
           R"(
           Execute a named event (e.g., 'stop').
           )")

      .def("ensure_active_controllers_for_group",
           &trajectory_execution_manager::TrajectoryExecutionManager::ensureActiveControllersForGroup, py::arg("group"),
           R"(
           Make sure the active controllers are such that trajectories that actuate joints in the specified group can be executed.

           If manage_controllers_ is false and the controllers that happen to be active do not cover the joints in the group to be actuated, this function fails.
           )")

      .def("ensure_active_controllers_for_joints",
           &trajectory_execution_manager::TrajectoryExecutionManager::ensureActiveControllersForJoints,
           py::arg("joints"),
           R"(
           Make sure the active controllers are such that trajectories that actuate joints in the specified set can be executed.

           If manage_controllers_ is false and the controllers that happen to be active do not cover the joints to be actuated, this function fails.
           )")

      .def("ensure_active_controller", &trajectory_execution_manager::TrajectoryExecutionManager::ensureActiveController,
           py::arg("controller"),
           R"(
           Make sure a particular controller is active.

           If manage_controllers_ is false and the controllers that happen to be active to not include the one specified as argument, this function fails.
           )")

      .def("ensure_active_controllers",
           &trajectory_execution_manager::TrajectoryExecutionManager::ensureActiveControllers, py::arg("controllers"),
           R"(
           Make sure a particular set of controllers are active.

           If manage_controllers_ is false and the controllers that happen to be active to not include the ones specified as argument, this function fails.
          )")

      .def("is_controller_active", &trajectory_execution_manager::TrajectoryExecutionManager::isControllerActive,
           py::arg("controller"),
           R"(
           Check if a controller is active.
           )")

      .def("are_controllers_active", &trajectory_execution_manager::TrajectoryExecutionManager::areControllersActive,
           py::arg("controllers"),
           R"(
           Check if a set of controllers are active
           )")

      .def("push",
           (bool(trajectory_execution_manager::TrajectoryExecutionManager::*)(const moveit_msgs::msg::RobotTrajectory&,
                                                                              const std::string&)) &
               trajectory_execution_manager::TrajectoryExecutionManager::push,
           py::arg("trajectory"), py::arg("controller") = "",
           R"(
           Add a trajectory for future execution. Optionally specify a controller to use for the trajectory.

           If no controller is specified, a default is used.
           )")

      .def("push",
           (bool(trajectory_execution_manager::TrajectoryExecutionManager::*)(
               const trajectory_msgs::msg::JointTrajectory&, const std::string&)) &
               trajectory_execution_manager::TrajectoryExecutionManager::push,
           py::arg("trajectory"), py::arg("controller") = "",
           R"(
           Add a trajectory for future execution. Optionally specify a controller to use for the trajectory.

           If no controller is specified, a default is used.
           )")

      .def("push",
           (bool(trajectory_execution_manager::TrajectoryExecutionManager::*)(const moveit_msgs::msg::RobotTrajectory&,
                                                                              const std::vector<std::string>&)) &
               trajectory_execution_manager::TrajectoryExecutionManager::push,
           py::arg("trajectory"), py::arg("controllers"),
           R"(
           Add a trajectory for future execution.

           Optionally specify a set of controllers to consider using for the trajectory.
           Multiple controllers can be used simultaneously to execute the different parts of the trajectory.
           If multiple controllers can be used, preference is given to the already loaded ones.
           If no controller is specified, a default is used.
           )")

      .def("push",
           (bool(trajectory_execution_manager::TrajectoryExecutionManager::*)(
               const trajectory_msgs::msg::JointTrajectory&, const std::vector<std::string>&)) &
               trajectory_execution_manager::TrajectoryExecutionManager::push,
           py::arg("trajectory"), py::arg("controllers"),
           R"(
           Add a trajectory for future execution.

           Optionally specify a set of controllers to consider using for the trajectory.
           Multiple controllers can be used simultaneously to execute the different parts of the trajectory.
           If multiple controllers can be used, preference is given to the already loaded ones.
           If no controller is specified, a default is used.
           )")

      // ToDo(MatthijsBurgh)
      // See https://github.com/ros-planning/moveit2/issues/2442
      // get_trajectories
      // execute
      // execute_and_wait
      // wait_for_execution
      // get_current_expected_trajectory_index
      // get_last_execution_status

      .def("stop_execution", &trajectory_execution_manager::TrajectoryExecutionManager::stopExecution,
           py::arg("auto_clear") = true,
           R"(
           Stop whatever executions are active, if any.
           )")

      .def("clear", &trajectory_execution_manager::TrajectoryExecutionManager::clear,
           R"(
           Clear the trajectories to execute.
           )")

      .def("enable_execution_duration_monitoring",
           &trajectory_execution_manager::TrajectoryExecutionManager::enableExecutionDurationMonitoring,
           py::arg("flag"),
           R"(
           Enable or disable the monitoring of trajectory execution duration.

           If a controller takes longer than expected, the trajectory is canceled.
           )")

      .def("set_allowed_execution_duration_scaling",
           &trajectory_execution_manager::TrajectoryExecutionManager::setAllowedExecutionDurationScaling,
           py::arg("scaling"),
           R"(
           When determining the expected duration of a trajectory, this multiplicative factor is applied to get the allowed duration of execution.
           )")

      .def("set_allowed_goal_duration_margin",
           &trajectory_execution_manager::TrajectoryExecutionManager::setAllowedGoalDurationMargin, py::arg("margin"),
           R"(
           When determining the expected duration of a trajectory, this additional margin s applied after scalign to allow more than the expected execution time before triggering trajectory cancel.
           )")

      .def("set_execution_velocity_scaling",
           &trajectory_execution_manager::TrajectoryExecutionManager::setExecutionVelocityScaling, py::arg("scaling"),
           R"(
           Before sending a trajectory to a controller, scale the velocities by the factor specified.

           By default, this is 1.0
           )")

      .def("set_allowed_start_tolerance",
           &trajectory_execution_manager::TrajectoryExecutionManager::setAllowedStartTolerance, py::arg("tolerance"),
           R"(
           Set joint-value tolerance for validating trajectory's start point against current robot state.
           )")

      .def("set_wait_for_trajectory_completion",
           &trajectory_execution_manager::TrajectoryExecutionManager::setWaitForTrajectoryCompletion, py::arg("flag"),
           R"(
           Enable or disable waiting for trajectory completion.
           )");

  // ToDo(MatthijsBurgh)
  // https://github.com/ros-planning/moveit2/issues/2442
  // get_controller_manager_node
}
}  // namespace bind_trajectory_execution_manager
}  // namespace moveit_py
