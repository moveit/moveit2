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

#include "utils.h"
#include <rclcpp/rclcpp.hpp>
#include <moveit_py/moveit_py_utils/ros_msg_typecasters.h>
#include <moveit/kinematic_constraints/utils.h>

namespace moveit_py
{
namespace bind_kinematic_constraints
{
moveit_msgs::msg::Constraints construct_link_constraint(const std::string& link_name, const std::string& source_frame,
                                                        std::optional<std::vector<double>> cartesian_position,
                                                        std::optional<double> cartesian_position_tolerance,
                                                        std::optional<std::vector<double>> orientation,
                                                        std::optional<double> orientation_tolerance)
{
  // check that link cartesian and/or orientation constraints are specified
  if (!cartesian_position && !orientation)
  {
    throw std::invalid_argument("No link cartesian or orientation constraints specified");
  }

  moveit_msgs::msg::Constraints constraints_cpp;

  // merge constraints if necessary
  if (cartesian_position && orientation)
  {
    // define point stamped message
    geometry_msgs::msg::PointStamped point;
    point.header.frame_id = source_frame;
    point.point.x = cartesian_position.value()[0];
    point.point.y = cartesian_position.value()[1];
    point.point.z = cartesian_position.value()[2];

    moveit_msgs::msg::Constraints position_constraints =
        kinematic_constraints::constructGoalConstraints(link_name, point, cartesian_position_tolerance.value());

    // define quaternion message
    geometry_msgs::msg::QuaternionStamped quaternion;
    quaternion.header.frame_id = source_frame;
    quaternion.quaternion.x = orientation.value()[0];
    quaternion.quaternion.y = orientation.value()[1];
    quaternion.quaternion.z = orientation.value()[2];
    quaternion.quaternion.w = orientation.value()[3];

    moveit_msgs::msg::Constraints orientation_constraints =
        kinematic_constraints::constructGoalConstraints(link_name, quaternion, orientation_tolerance.value());

    constraints_cpp = kinematic_constraints::mergeConstraints(position_constraints, orientation_constraints);
  }

  // generate cartesian constraint
  else if (cartesian_position)
  {
    // define point stamped message
    geometry_msgs::msg::PointStamped point;
    point.header.frame_id = source_frame;
    point.point.x = cartesian_position.value()[0];
    point.point.y = cartesian_position.value()[1];
    point.point.z = cartesian_position.value()[2];

    // instantiate logger
    auto logger = rclcpp::get_logger("moveit_py");
    // check point with logger
    RCLCPP_DEBUG(rclcpp::get_logger("moveit_py"), "Point: %f, %f, %f", point.point.x, point.point.y, point.point.z);

    constraints_cpp = kinematic_constraints::constructGoalConstraints(link_name, point, *cartesian_position_tolerance);
  }

  // generate orientation constraint
  else
  {
    // define quaternion message
    geometry_msgs::msg::QuaternionStamped quaternion;
    quaternion.header.frame_id = source_frame;
    quaternion.quaternion.x = orientation.value()[0];
    quaternion.quaternion.y = orientation.value()[1];
    quaternion.quaternion.z = orientation.value()[2];
    quaternion.quaternion.w = orientation.value()[3];
    constraints_cpp =
        kinematic_constraints::constructGoalConstraints(link_name, quaternion, orientation_tolerance.value());
  }

  return constraints_cpp;
}

moveit_msgs::msg::Constraints construct_joint_constraint(moveit::core::RobotState& robot_state,
                                                         moveit::core::JointModelGroup* joint_model_group,
                                                         double tolerance)
{
  // generate joint constraint message
  moveit_msgs::msg::Constraints joint_constraints =
      kinematic_constraints::constructGoalConstraints(robot_state, joint_model_group, tolerance);

  return joint_constraints;
}

moveit_msgs::msg::Constraints construct_constraints_from_node(const std::shared_ptr<rclcpp::Node>& node_name,
                                                              const std::string& ns)
{
  // construct constraint message
  moveit_msgs::msg::Constraints constraints_cpp;
  kinematic_constraints::constructConstraints(node_name, ns, constraints_cpp);

  return constraints_cpp;
}

void init_kinematic_constraints(py::module& m)
{
  py::module kinematic_constraints = m.def_submodule("kinematic_constraints");

  kinematic_constraints.def("construct_link_constraint", &construct_link_constraint, py::arg("link_name"),
                            py::arg("source_frame"), py::arg("cartesian_position") = nullptr,
                            py::arg("cartesian_position_tolerance") = nullptr, py::arg("orientation") = nullptr,
                            py::arg("orientation_tolerance") = nullptr, "Construct a link constraint message");
  kinematic_constraints.def("construct_joint_constraint", &construct_joint_constraint, py::arg("robot_state"),
                            py::arg("joint_model_group"), py::arg("tolerance") = 0.01,
                            "Construct a joint constraint message");
  kinematic_constraints.def("construct_constraints_from_node", &construct_constraints_from_node, py::arg("node_name"),
                            py::arg("ns"), "Construct a constraint message from a node");
}

}  // namespace bind_kinematic_constraints
}  // namespace moveit_py
