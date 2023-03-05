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

#include "robot_trajectory.h"
#include <moveit_py/moveit_py_utils/ros_msg_typecasters.h>

namespace moveit_py
{
namespace bind_robot_trajectory
{
moveit_msgs::msg::RobotTrajectory
get_robot_trajectory_msg(const robot_trajectory::RobotTrajectoryConstPtr& robot_trajectory,
                         const std::vector<std::string>& joint_filter)
{
  moveit_msgs::msg::RobotTrajectory msg;
  robot_trajectory->getRobotTrajectoryMsg(msg, joint_filter);
  return msg;
}

void init_robot_trajectory(py::module& m)
{
  py::module robot_trajectory = m.def_submodule("robot_trajectory");

  py::class_<robot_trajectory::RobotTrajectory, std::shared_ptr<robot_trajectory::RobotTrajectory>>(robot_trajectory,
                                                                                                    "RobotTrajectory",
                                                                                                    R"(
                                                    Maintains a sequence of waypoints and the durations between these waypoints.
                                                    )")

      .def("__getitem__", &robot_trajectory::RobotTrajectory::getWayPoint, py::arg("idx"),
           R"(
           Get the waypoint at the specified index in the trajectory.

           Returns:
               :py:class:`moveit_py.core.RobotState`: The robot state corresponding to a waypoint at the specified index in the trajectory.
           )")
      .def(
          "__iter__",
          [](robot_trajectory::RobotTrajectory& self) { return py::make_iterator(self.begin(), self.end()); },
          py::keep_alive<0, 1>() /* Essential: keep object alive while iterator exists */,
          R"(
           Iterate over the waypoints in the trajectory.
           )")

      .def("__len__", &robot_trajectory::RobotTrajectory::getWayPointCount,
           R"(
                    Returns:
                        int: The number of waypoints in the trajectory.
                    )")

      .def("__reverse__", &robot_trajectory::RobotTrajectory::reverse,
           R"(
     	   Reverse the trajectory.
     	   )")

      .def_property("joint_model_group_name", &robot_trajectory::RobotTrajectory::getGroupName,
                    &robot_trajectory::RobotTrajectory::setGroupName,
                    R"(
                    str: The name of the joint model group that this trajectory is for.
                    )")

      .def_property("robot_model", &robot_trajectory::RobotTrajectory::getRobotModel, nullptr,
                    R"(
                    :py:class:`moveit_py.core.RobotModel`: The robot model that this trajectory is for.
                    )")

      .def_property("duration", &robot_trajectory::RobotTrajectory::getDuration, nullptr,
                    R"(
                    float: The duration of the trajectory.
                    )")

      .def_property("average_segment_duration", &robot_trajectory::RobotTrajectory::getAverageSegmentDuration, nullptr,
                    R"(
                    float: The average duration of the segments in the trajectory.
                    )")

      .def("unwind", py::overload_cast<>(&robot_trajectory::RobotTrajectory::unwind),
           R"(
           Unwind the trajectory.
      	   )")

      .def("get_waypoint_durations", &robot_trajectory::RobotTrajectory::getWayPointDurations,
           R"(
           Get the durations from the previous waypoint in the trajectory.
           Returns:
               list of float: The duration from previous of each waypoint in the trajectory.
           )")
      .def("get_robot_trajectory_msg", &moveit_py::bind_robot_trajectory::get_robot_trajectory_msg,
           py::arg("joint_filter") = std::vector<std::string>(),
           R"(
           Get the trajectory as a `moveit_msgs.msg.RobotTrajectory` message.
           Returns:
               moveit_msgs.msg.RobotTrajectory: A ROS robot trajectory message.
           )");
  // TODO (peterdavidfagan): support other methods such as appending trajectories
}
}  // namespace bind_robot_trajectory
}  // namespace moveit_py
