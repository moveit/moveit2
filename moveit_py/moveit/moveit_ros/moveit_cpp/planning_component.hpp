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
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/functional.h>
#include <moveit_py/moveit_py_utils/copy_ros_msg.hpp>
#include <moveit_py/moveit_py_utils/ros_msg_typecasters.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/constraints.hpp>

#include "moveit_cpp.hpp"
#include "../planning_scene_monitor/planning_scene_monitor.hpp"
#include "../../moveit_core/planning_interface/planning_response.hpp"

namespace py = pybind11;

namespace moveit_py
{
namespace bind_planning_component
{
moveit_cpp::PlanningComponent::PlanSolution
plan(std::shared_ptr<moveit_cpp::PlanningComponent>& planning_component,
     std::shared_ptr<moveit_cpp::PlanningComponent::PlanRequestParameters>& single_plan_parameters);

bool setGoal(std::shared_ptr<moveit_cpp::PlanningComponent>& planning_component,
             std::optional<std::string> configuration_name, std::optional<moveit::core::RobotState> robot_state,
             std::optional<geometry_msgs::msg::PoseStamped> pose_stamped_msg, std::optional<std::string> pose_link,
             std::optional<std::vector<moveit_msgs::msg::Constraints>> motion_plan_constraints);

bool setStartState(std::shared_ptr<moveit_cpp::PlanningComponent>& planning_component,
                   std::optional<std::string> configuration_name, std::optional<moveit::core::RobotState> robot_state);

void initPlanRequestParameters(py::module& m);

// void initMultiPlanRequestParameters(py::module& m);

void initPlanningComponent(py::module& m);
}  // namespace bind_planning_component
}  // namespace moveit_py
