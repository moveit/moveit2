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

/* Author: Peter David Fagan */

#include "moveit_cpp.hpp"
#include <atomic>
#include <chrono>
#include <mutex>
#include <pybind11/pytypes.h>
#include <moveit/utils/logger.hpp>
#include <string>
#include <thread>

namespace moveit_py
{
namespace bind_moveit_cpp
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.py.cpp_initializer");
}

namespace
{
// Keep the executor alive until its callbacks have returned, including when construction throws.
class ExecutorThread
{
public:
  explicit ExecutorThread(const rclcpp::Node::SharedPtr& node)
    : executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
    , stop_requested_(std::make_shared<std::atomic_bool>(false))
  {
    executor_->add_node(node);
    execution_thread_ = std::thread([node, executor = executor_, stop_requested = stop_requested_]() {
      try
      {
        while (!stop_requested->load() && rclcpp::ok(node->get_node_base_interface()->get_context()))
          executor->spin_once(std::chrono::milliseconds(100));
      }
      catch (const std::exception& exception)
      {
        if (!stop_requested->load() && rclcpp::ok(node->get_node_base_interface()->get_context()))
          RCLCPP_ERROR(getLogger(), "MoveItPy executor stopped: %s", exception.what());
      }
    });
    execution_thread_id_ = execution_thread_.get_id();
  }

  ~ExecutorThread()
  {
    stop();
  }

  ExecutorThread(const ExecutorThread&) = delete;
  ExecutorThread& operator=(const ExecutorThread&) = delete;

  bool isCurrentThread() const
  {
    return execution_thread_id_ == std::this_thread::get_id();
  }

  void stop()
  {
    std::lock_guard<std::mutex> lock(stop_mutex_);
    stop_requested_->store(true);
    if (!execution_thread_.joinable() || execution_thread_.get_id() == std::this_thread::get_id())
      return;

    try
    {
      executor_->cancel();
    }
    catch (const std::exception&)
    {
      // The bounded spin_once wait also allows cleanup after the ROS context has stopped.
    }
    execution_thread_.join();
  }

private:
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<std::atomic_bool> stop_requested_;
  std::mutex stop_mutex_;
  std::thread execution_thread_;
  std::thread::id execution_thread_id_;
};

struct MoveItPyDeleter
{
  std::shared_ptr<ExecutorThread> executor_thread;

  void operator()(moveit_cpp::MoveItCpp* moveit_cpp) const
  {
    if (executor_thread->isCurrentThread())
    {
      // A callback can release the last holder. Join it from another thread before deleting its state.
      std::thread([executor_thread = executor_thread, moveit_cpp]() {
        executor_thread->stop();
        delete moveit_cpp;
      }).detach();
      return;
    }
    executor_thread->stop();
    delete moveit_cpp;
  }
};
}  // namespace

std::shared_ptr<moveit_cpp::PlanningComponent>
getPlanningComponent(std::shared_ptr<moveit_cpp::MoveItCpp>& moveit_cpp_ptr, const std::string& planning_component)
{
  return std::make_shared<moveit_cpp::PlanningComponent>(planning_component, moveit_cpp_ptr);
}

void initMoveitPy(py::module& m)
{
  auto utils = py::module::import("moveit.utils");

  py::class_<moveit_cpp::MoveItCpp, std::shared_ptr<moveit_cpp::MoveItCpp>>(m, "MoveItPy", R"(
  The MoveItPy class is the main interface to the MoveIt Python API. It is a wrapper around the MoveIt C++ API.
									     )")

      .def(py::init([](const std::string& node_name, const std::string& name_space,
                       const std::vector<std::string>& launch_params_filepaths, const py::object& config_dict,
                       bool provide_planning_service,
                       const std::optional<std::map<std::string, std::string>>& remappings) {
             // This section is used to load the appropriate node parameters before spinning a moveit_cpp instance
             // Priority is given to parameters supplied directly via a config_dict, followed by launch parameters
             // and finally no supplied parameters.
             std::vector<std::string> launch_arguments;
             if (!config_dict.is(py::none()))
             {
               auto utils = py::module::import("moveit.utils");
               // TODO (peterdavidfagan): replace python method with C++ method
               std::string params_filepath =
                   utils.attr("create_params_file_from_dict")(config_dict, node_name).cast<std::string>();
               launch_arguments = { "--ros-args", "--params-file", params_filepath };
             }
             else if (!launch_params_filepaths.empty())
             {
               launch_arguments = { "--ros-args" };
               for (const auto& launch_params_filepath : launch_params_filepaths)
               {
                 launch_arguments.push_back("--params-file");
                 launch_arguments.push_back(launch_params_filepath);
               }
             }

             if (remappings.has_value())
             {
               for (const auto& [key, value] : *remappings)
               {
                 std::string argument = key;
                 argument.append(":=").append(value);
                 launch_arguments.push_back("--remap");
                 launch_arguments.push_back(std::move(argument));
               }
             }

             // Instance-specific parameters and remappings belong to NodeOptions, not the shared context.
             if (!rclcpp::ok())
             {
               rclcpp::init(0, nullptr);
               RCLCPP_INFO(getLogger(), "Initialize rclcpp");
             }

             // Build NodeOptions
             RCLCPP_INFO(getLogger(), "Initialize node parameters");
             rclcpp::NodeOptions node_options;
             node_options.allow_undeclared_parameters(true)
                 .automatically_declare_parameters_from_overrides(true)
                 .arguments(launch_arguments);

             RCLCPP_INFO(getLogger(), "Initialize node and executor");
             rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(node_name, name_space, node_options);

             RCLCPP_INFO(getLogger(), "Spin separate thread");
             auto executor_thread = std::make_shared<ExecutorThread>(node);
             std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr(new moveit_cpp::MoveItCpp(node),
                                                                   MoveItPyDeleter{ executor_thread });

             if (provide_planning_service)
             {
               const auto service_name = node->get_node_base_interface()->resolve_topic_or_service_name(
                   planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_SERVICE, true);
               moveit_cpp_ptr->getPlanningSceneMonitorNonConst()->providePlanningSceneService(service_name);
             };

             return moveit_cpp_ptr;
           }),
           py::arg("node_name") = "moveit_py", py::arg("name_space") = "",
           py::arg("launch_params_filepaths") =
               utils.attr("get_launch_params_filepaths")().cast<std::vector<std::string>>(),
           py::arg("config_dict") = py::none(), py::arg("provide_planning_service") = true,
           py::arg("remappings") = py::none(), py::return_value_policy::take_ownership,
           R"(
           Initialize moveit_cpp node and the planning scene service.
           )")
      .def("execute",
           py::overload_cast<const robot_trajectory::RobotTrajectoryPtr&, const std::vector<std::string>&>(
               &moveit_cpp::MoveItCpp::execute),
           py::arg("robot_trajectory"), py::arg("controllers"), py::call_guard<py::gil_scoped_release>(),
           R"(
	   Execute a trajectory (planning group is inferred from robot trajectory object).
	   )")
      .def("get_planning_component", &moveit_py::bind_moveit_cpp::getPlanningComponent,
           py::arg("planning_component_name"), py::return_value_policy::take_ownership,
           R"(
           Creates a planning component instance.
           Args:
               planning_component_name (str): The name of the planning component.
           Returns:
               :py:class:`moveit_py.planning.PlanningComponent`: A planning component instance corresponding to the provided plan component name.
          )")

      .def(
          "shutdown",
          [](std::shared_ptr<moveit_cpp::MoveItCpp>& moveit_cpp) {
            if (auto* deleter = std::get_deleter<MoveItPyDeleter>(moveit_cpp))
              deleter->executor_thread->stop();
          },
          py::call_guard<py::gil_scoped_release>(),
          R"(
          Stop this instance's executor and wait for its callbacks to finish.
          Repeated calls are safe. Other instances and the shared ROS context remain running.
          )")

      .def("get_planning_scene_monitor", &moveit_cpp::MoveItCpp::getPlanningSceneMonitorNonConst,
           py::return_value_policy::reference,
           R"(
           Returns the planning scene monitor.
           )")

      .def("get_trajectory_execution_manager", &moveit_cpp::MoveItCpp::getTrajectoryExecutionManagerNonConst,
           py::return_value_policy::reference,
           R"(
           Returns the trajectory execution manager.
           )")

      .def("get_robot_model", &moveit_cpp::MoveItCpp::getRobotModel, py::return_value_policy::reference,
           R"(
           Returns robot model.
        )");
}
}  // namespace bind_moveit_cpp
}  // namespace moveit_py
