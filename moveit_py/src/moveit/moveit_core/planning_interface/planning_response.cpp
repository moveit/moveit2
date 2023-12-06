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

#include "planning_response.h"

namespace moveit_py
{
namespace bind_planning_interface
{
std::shared_ptr<robot_trajectory::RobotTrajectory>
getMotionPlanResponseTrajectory(std::shared_ptr<planning_interface::MotionPlanResponse>& response)
{
  return response->trajectory;
}

moveit_msgs::msg::RobotState
getMotionPlanResponseStartState(std::shared_ptr<planning_interface::MotionPlanResponse>& response)
{
  moveit_msgs::msg::RobotState robot_state_msg = response->start_state;
  return robot_state_msg;
}

moveit_msgs::msg::MoveItErrorCodes
getMotionPlanResponseErrorCode(std::shared_ptr<planning_interface::MotionPlanResponse>& response)
{
  moveit_msgs::msg::MoveItErrorCodes error_code_msg =
      static_cast<moveit_msgs::msg::MoveItErrorCodes>(response->error_code);
  return error_code_msg;
}

double getMotionPlanResponsePlanningTime(std::shared_ptr<planning_interface::MotionPlanResponse>& response)
{
  return response->planning_time;
}

std::string getMotionPlanResponsePlannerId(std::shared_ptr<planning_interface::MotionPlanResponse>& response)
{
  return response->planner_id;
}

void initMotionPlanResponse(py::module& m)
{
  py::module planning_interface = m.def_submodule("planning_interface");

  py::class_<planning_interface::MotionPlanResponse, std::shared_ptr<planning_interface::MotionPlanResponse>>(
      planning_interface, "MotionPlanResponse", R"()")

      //.def(py::init<>(), R"()")

      .def_property("trajectory", &moveit_py::bind_planning_interface::getMotionPlanResponseTrajectory, nullptr,
                    py::return_value_policy::copy, R"()")

      .def_readonly("planning_time", &planning_interface::MotionPlanResponse::planning_time,
                    py::return_value_policy::copy, R"()")

      .def_property("error_code", &moveit_py::bind_planning_interface::getMotionPlanResponseErrorCode, nullptr,
                    py::return_value_policy::copy, R"()")

      .def_property("start_state", &moveit_py::bind_planning_interface::getMotionPlanResponseStartState, nullptr,
                    py::return_value_policy::copy, R"()")

      .def_readonly("planner_id", &planning_interface::MotionPlanResponse::planner_id, py::return_value_policy::copy,
                    R"()")

      .def("__bool__", [](std::shared_ptr<planning_interface::MotionPlanResponse>& response) {
        return response->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      });
}
}  // namespace bind_planning_interface
}  // namespace moveit_py
