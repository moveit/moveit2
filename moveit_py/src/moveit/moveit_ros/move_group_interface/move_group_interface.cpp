/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Matthew Elwin
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

/* Author: Matthew Elwin*/

#include "move_group_interface.h"
#include <pybind11/stl.h>
#include <moveit_py/moveit_py_utils/ros_msg_typecasters.h>

namespace moveit_py
{
namespace bind_move_group_interface
{

void init_move_group_interface(py::module& m)
{
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_iface =
      py::class_<MoveGroupInterface, std::shared_ptr<MoveGroupInterface>>(
          m, "MoveGroupInterface",
          R"(The MoveGroupInterface class is a wrapper around the moveit::planning_interface::MoveGroupInterface C++ class.

               One difference from the C++ class is that the MoveGroupInterface creates it's own internal node
               with the given name (because a python node cannot easily be converted into a C++ node)")
          .def(py::init([](const std::string node_name, const std::string& group_name) {
                 if (!rclcpp::ok())
                 {
                   // TODO: add ability to pass arguments to the ros initialization.
                   rclcpp::init(0, nullptr);
                 }
                 return std::make_shared<MoveGroupInterface>(
                     std::make_shared<rclcpp::Node>(
                         node_name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
                     group_name);
               }),
               R"(Args:
               node_name - The name of the node that will be used to interface with an existing move_group node.
               group_name - The name of the move_group.)")
          .def("setPoseTarget",
               py::overload_cast<const geometry_msgs::msg::Pose&, const std::string&>(
                   &MoveGroupInterface::setPoseTarget),
               "Set the goal pose of the end-effector link. see C++ documentation for details.", py::arg("target"),
               py::arg("end_effector_link") = "")
          .def(
              "plan",
              [](MoveGroupInterface& mgi) {
                MoveGroupInterface::Plan p;
                // cast the moveit::core::MoveItErrorCode type into the message type so that it can be transferred to python
                const auto result = static_cast<moveit_msgs::msg::MoveItErrorCodes>(mgi.plan(p));
                return std::make_pair(result, p);
              },
              R"(Compute a motion plan from the current state to specified target.

                 Unlike the C++ counterpart, this method returns the plan directly rather than taking it as an output parameter.

                Returns:
                (MoveItErrorCode, Plan) containing the planning result status and the actual plan)")
          .def(
              "execute",
              [](MoveGroupInterface& mgi, const MoveGroupInterface::Plan& plan,
                 const std::vector<std::string>& controllers) {
                return static_cast<moveit_msgs::msg::MoveItErrorCodes>(mgi.execute(plan, controllers));
              },
              "Given a plan, execute it while waiting for completion.", py::arg("plan"),
              py::arg("controllers") = std::vector<std::string>());

  py::class_<MoveGroupInterface::Plan>(move_group_iface, "Plan")
      .def(py::init<>())
      .def_readwrite("start_state", &MoveGroupInterface::Plan::start_state,
                     R"(The full starting state used for planning)")
      .def_readwrite("trajectory", &MoveGroupInterface::Plan::trajectory, R"(The trajectory of the robot.)")
      .def_readwrite("planning_time", &MoveGroupInterface::Plan::planning_time,
                     R"(The amount of time it took to generate the plan)");
}
}  // namespace bind_move_group_interface
}  // namespace moveit_py
