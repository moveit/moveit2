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

#include "moveit_cpp.h"

namespace moveit_py
{
namespace bind_moveit_cpp
{
std::shared_ptr<moveit_cpp::PlanningComponent>
get_planning_component(std::shared_ptr<moveit_cpp::MoveItCpp>& moveit_cpp_ptr, const std::string& planning_component)
{
  return std::make_shared<moveit_cpp::PlanningComponent>(planning_component, moveit_cpp_ptr);
}

void init_moveit_py(py::module& m)
{
  auto utils = py::module::import("moveit.utils");

  py::class_<moveit_cpp::MoveItCpp, std::shared_ptr<moveit_cpp::MoveItCpp>>(m, "MoveItPy", R"(
  The MoveItPy class is the main interface to the MoveIt Python API. It is a wrapper around the MoveIt C++ API.
									     )")

      .def(py::init([](const std::string& node_name, const std::string& launch_params_filepath,
                       const py::object& config_dict, bool provide_planning_service) {
             static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_initializer");

             if (!rclcpp::ok())
             {
               RCLCPP_INFO(LOGGER, "Initialize rclcpp");
               rclcpp::init(0, nullptr);
             }

             RCLCPP_INFO(LOGGER, "Initialize node parameters");
             rclcpp::NodeOptions node_options;

             // This section is used to load the appropriate node parameters before spinning a moveit_cpp instance
             // Priority is given to parameters supplied directly via a config_dict, followed by launch parameters
             // and finally no supplied parameters.
             if (!config_dict.is(py::none()))
             {
               auto utils = py::module::import("moveit.utils");
               // TODO (peterdavidfagan): replace python method with C++ method
               std::string params_filepath =
                   utils.attr("create_params_file_from_dict")(config_dict, "moveit_py").cast<std::string>();
               RCLCPP_INFO(LOGGER, "params_filepath: %s", params_filepath.c_str());
               node_options.allow_undeclared_parameters(true)
                   .automatically_declare_parameters_from_overrides(true)
                   .arguments({ "--ros-args", "--params-file", params_filepath });
             }
             else if (!launch_params_filepath.empty())
             {
               node_options.allow_undeclared_parameters(true)
                   .automatically_declare_parameters_from_overrides(true)
                   .arguments({ "--ros-args", "--params-file", launch_params_filepath });
             }
             else
             {
               // TODO (peterdavidfagan): consider failing initialization if no params file is provided
               node_options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
             }
             RCLCPP_INFO(LOGGER, "Initialize node and executor");
             rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(node_name, "", node_options);
             std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor =
                 std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

             RCLCPP_INFO(LOGGER, "Spin separate thread");
             auto spin_node = [node, executor]() {
               executor->add_node(node);
               executor->spin();
             };
             std::thread execution_thread(spin_node);
             execution_thread.detach();

             auto custom_deleter = [executor](moveit_cpp::MoveItCpp* moveit_cpp) {
               executor->cancel();
               rclcpp::shutdown();
               delete moveit_cpp;
             };

             std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr(new moveit_cpp::MoveItCpp(node), custom_deleter);

             if (provide_planning_service)
             {
               moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();
             };

             return moveit_cpp_ptr;
           }),
           py::arg("node_name") = "moveit_py",
           py::arg("launch_params_filepath") = utils.attr("get_launch_params_filepath")().cast<std::string>(),
           py::arg("config_dict") = py::none(), py::arg("provide_planning_service") = true,
           py::return_value_policy::take_ownership,
           R"(
           Initialize moveit_cpp node and the planning scene service.
           )")
      .def("execute",
           py::overload_cast<const robot_trajectory::RobotTrajectoryPtr&, bool, const std::vector<std::string>&>(
               &moveit_cpp::MoveItCpp::execute),
           py::arg("robot_trajectory"), py::arg("blocking") = true, py::arg("controllers"),
           R"(
	   Execute a trajectory (planning group is inferred from robot trajectory object).
	   )")
      .def("get_planning_component", &moveit_py::bind_moveit_cpp::get_planning_component,
           py::arg("planning_component_name"), py::return_value_policy::take_ownership,
           R"(
           Creates a planning component instance.
           Args:
               planning_component_name (str): The name of the planning component.
           Returns:
               :py:class:`moveit_py.planning.PlanningComponent`: A planning component instance corresponding to the provided plan component name.
          )")

      .def(
          "shutdown", [](std::shared_ptr<moveit_cpp::MoveItCpp>& /*moveit_cpp*/) { rclcpp::shutdown(); },
          R"(
          Shutdown the moveit_cpp node.
          )")

      .def("get_planning_scene_monitor", &moveit_cpp::MoveItCpp::getPlanningSceneMonitor,
           py::return_value_policy::reference,
           R"(
           Returns the planning scene monitor.
           )")

      .def("get_robot_model", &moveit_cpp::MoveItCpp::getRobotModel, py::return_value_policy::reference,
           R"(
           Returns robot model.
        )");
}
}  // namespace bind_moveit_cpp
}  // namespace moveit_py
