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

#include "robot_model.h"
#include <fstream>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <srdfdom/srdf_writer.h>
#include <moveit/robot_model/robot_model.h>

namespace moveit_py
{
namespace bind_robot_model
{
void initRobotModel(py::module& m)
{
  py::module robot_model = m.def_submodule("robot_model");

  py::class_<moveit::core::RobotModel, std::shared_ptr<moveit::core::RobotModel>>(robot_model, "RobotModel",
                                                                                  R"(
          Representation of a kinematic model.
          )")
      // TODO (peterdavidfagan): rewrite with RobotModelLoader.
      .def(py::init([](std::string& urdf_xml_path, std::string& srdf_xml_path) {
             // Read in URDF
             std::string xml_string;
             std::fstream xml_file(urdf_xml_path.c_str(), std::fstream::in);
             while (xml_file.good())
             {
               std::string line;
               std::getline(xml_file, line);
               xml_string += (line + "\n");
             }
             xml_file.close();

             urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(xml_string);

             // Read in SRDF
             srdf::Model srdf_model;
             srdf_model.initFile(*urdf_model, srdf_xml_path);

             // Instantiate robot model
             return std::make_shared<moveit::core::RobotModel>(
                 urdf_model, std::make_shared<const srdf::Model>(std::move(srdf_model)));
           }),
           py::arg("urdf_xml_path"), py::arg("srdf_xml_path"),
           R"(
           Initializes a kinematic model for a robot.

           Args:
               urdf_xml_path (str): The filepath to the urdf file describing the robot.
               srdf_xml_path (str): The filepath to the srdf file describing the robot semantics.
           Returns:
               moveit_py.core.RobotModel: The kinematic model for the robot.
       )")

      .def_property("name", &moveit::core::RobotModel::getName, nullptr,
                    R"(
                    str: The name of the robot model.
                    )")

      .def_property("model_frame", &moveit::core::RobotModel::getModelFrame, nullptr,
                    R"(
                    str: Get the frame in which the transforms for this model are computed (when using a :py:class:`moveit_py.core.RobotState`).
                    This frame depends on the root joint. As such, the frame is either extracted from SRDF, or it is assumed to be the name of the root link.
                   )")

      // TODO (peterdavidfagan): make available when JointModel is binded
      //.def_property("root_joint", &moveit::core::RobotModel::getRootJoint, nullptr, "The root joint of the robot model.")

      .def_property("root_joint_name", &moveit::core::RobotModel::getRootJointName, nullptr,
                    R"(
                    str: The name of the root joint.
                    )")

      .def(
          "get_model_info",
          [](std::shared_ptr<moveit::core::RobotModel>& s) {
            std::stringstream ss;
            s->printModelInfo(ss);
            return ss.str();
          },
          py::return_value_policy::move,
          R"(
          Gets a formatted string containing a summary of relevant information from the robot model.
          Returns:
              str: Formatted string containing generic robot model information.
          )")

      // Interacting with joint model groups
      .def_property("joint_model_group_names", &moveit::core::RobotModel::getJointModelGroupNames, nullptr,
                    R"(
                    list of str: The names of the joint model groups in the robot model.
                    )")

      .def_property("joint_model_groups", py::overload_cast<>(&moveit::core::RobotModel::getJointModelGroups), nullptr,
                    py::return_value_policy::reference_internal,
                    R"(
                    list of moveit_py.core.JointModelGroup: The joint model groups available in the robot model.
                    )")

      .def_property("end_effectors", &moveit::core::RobotModel::getEndEffectors, nullptr,
                    py::return_value_policy::reference_internal,
                    R"(
                    TODO
                    )")

      .def("has_joint_model_group", &moveit::core::RobotModel::hasJointModelGroup, py::arg("joint_model_group_name"),
           R"(
           Checks if a joint model group with the given name exists in the robot model.
           Returns:
                   bool: true if joint model group exists.
           )")

      .def("get_joint_model_group",
           py::overload_cast<const std::string&>(&moveit::core::RobotModel::getJointModelGroup),
           py::arg("joint_model_group_name"), py::return_value_policy::reference_internal,
           R"(
           Gets a joint model group instance by name.
           Args:
               joint_model_group_name (str): The name of the joint model group to return.
           Returns:
               :py:class:`moveit_py.core.JointModelGroup`: joint model group instance that corresponds with joint_model_group_name parameter.
           )");
}
}  // namespace bind_robot_model
}  // namespace moveit_py
