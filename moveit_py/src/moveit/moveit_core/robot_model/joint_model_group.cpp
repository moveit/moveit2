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

#include "joint_model_group.h"

namespace moveit_py
{
namespace bind_robot_model
{
bool satisfies_position_bounds(const moveit::core::JointModelGroup* jmg, const Eigen::VectorXd& joint_positions,
                               const double margin)
{
  assert(joint_positions.size() == jmg->getActiveVariableCount());
  return jmg->satisfiesPositionBounds(joint_positions.data(), margin);
}

void init_joint_model_group(py::module& m)
{
  py::module robot_model = m.def_submodule("robot_model");

  py::class_<moveit::core::JointModelGroup>(robot_model, "JointModelGroup",
                                            R"(
          Representation of a group of joints that are part of a robot model.
          )")

      .def_property("name", &moveit::core::JointModelGroup::getName, nullptr,
                    R"(
                    str: The name of the joint model group.
                    )")

      .def_property("joint_model_names", &moveit::core::JointModelGroup::getJointModelNames, nullptr,
                    R"(
                    list[str]: The names of the joint models in the group.
                    )")
      .def_property("active_joint_model_names", &moveit::core::JointModelGroup::getActiveJointModelNames, nullptr)
      .def_property("active_joint_model_bounds", &moveit::core::JointModelGroup::getActiveJointModelsBounds, nullptr,
                    py::return_value_policy::reference_internal)
      .def("satisfies_position_bounds", &moveit_py::bind_robot_model::satisfies_position_bounds, py::arg("values"),
           py::arg("margin") = 0.0);
}
}  // namespace bind_robot_model
}  // namespace moveit_py
