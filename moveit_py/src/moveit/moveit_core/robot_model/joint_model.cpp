/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Jafar Uruç
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

/* Author: Jafar Uruç */

#include "joint_model.h"
#include <moveit/robot_model/joint_model.h>

namespace moveit_py
{
namespace bind_robot_model
{

void init_joint_model(py::module& m)
{
  py::module robot_model = m.def_submodule("robot_model");

  py::class_<moveit::core::VariableBounds, std::shared_ptr<moveit::core::VariableBounds>>(robot_model, "VariableBounds")
      .def_readonly("min_position", &moveit::core::VariableBounds::min_position_)
      .def_readonly("max_position", &moveit::core::VariableBounds::max_position_)
      .def_readonly("position_bounded", &moveit::core::VariableBounds::position_bounded_)
      .def_readonly("min_velocity", &moveit::core::VariableBounds::min_velocity_)
      .def_readonly("max_velocity", &moveit::core::VariableBounds::max_velocity_)
      .def_readonly("velocity_bounded", &moveit::core::VariableBounds::velocity_bounded_)
      .def_readonly("min_acceleration", &moveit::core::VariableBounds::min_acceleration_)
      .def_readonly("max_acceleration", &moveit::core::VariableBounds::max_acceleration_)
      .def_readonly("acceleration_bounded", &moveit::core::VariableBounds::acceleration_bounded_)
      .def_readonly("min_jerk", &moveit::core::VariableBounds::min_jerk_)
      .def_readonly("max_jerk", &moveit::core::VariableBounds::max_jerk_)
      .def_readonly("jerk_bounded", &moveit::core::VariableBounds::jerk_bounded_);
}
}  // namespace bind_robot_model
}  // namespace moveit_py
