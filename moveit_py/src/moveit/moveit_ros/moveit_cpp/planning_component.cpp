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

#include "planning_component.h"
#include <memory>

namespace moveit_py
{
namespace bind_planning_component
{
planning_interface::MotionPlanResponse
plan(std::shared_ptr<moveit_cpp::PlanningComponent>& planning_component,
     std::shared_ptr<moveit_cpp::PlanningComponent::PlanRequestParameters>& single_plan_parameters,
     std::shared_ptr<moveit_cpp::PlanningComponent::MultiPipelinePlanRequestParameters>& multi_plan_parameters,
     std::shared_ptr<planning_scene::PlanningScene>& planning_scene,
     std::optional<const moveit::planning_pipeline_interfaces::SolutionSelectionFunction> solution_selection_function,
     std::optional<moveit::planning_pipeline_interfaces::StoppingCriterionFunction> stopping_criterion_callback)
{
  // parameter argument checking
  if (single_plan_parameters && multi_plan_parameters)
  {
    throw std::invalid_argument("Cannot specify both single and multi plan parameters");
  }

  //  check whether single or multi pipeline
  if (single_plan_parameters)
  {
    // cast parameters
    std::shared_ptr<const moveit_cpp::PlanningComponent::PlanRequestParameters> const_single_plan_parameters =
        std::const_pointer_cast<const moveit_cpp::PlanningComponent::PlanRequestParameters>(single_plan_parameters);

    return planning_component->plan(*const_single_plan_parameters, planning_scene);
  }
  else if (multi_plan_parameters)
  {
    // cast parameters
    std::shared_ptr<const moveit_cpp::PlanningComponent::MultiPipelinePlanRequestParameters> const_multi_plan_parameters =
        std::const_pointer_cast<const moveit_cpp::PlanningComponent::MultiPipelinePlanRequestParameters>(
            multi_plan_parameters);

    if (solution_selection_function && stopping_criterion_callback)
    {
      return planning_component->plan(*const_multi_plan_parameters, std::ref(*solution_selection_function),
                                      *stopping_criterion_callback, planning_scene);
    }
    else if (solution_selection_function)
    {
      return planning_component->plan(*const_multi_plan_parameters, std::ref(*solution_selection_function), nullptr,
                                      planning_scene);
    }
    else if (stopping_criterion_callback)
    {
      return planning_component->plan(*const_multi_plan_parameters,
                                      moveit::planning_pipeline_interfaces::getShortestSolution,
                                      *stopping_criterion_callback, planning_scene);
    }
    else
    {
      return planning_component->plan(*const_multi_plan_parameters,
                                      moveit::planning_pipeline_interfaces::getShortestSolution, nullptr,
                                      planning_scene);
    }
  }
  else
  {
    if (planning_scene)
    {
      throw std::invalid_argument("Cannot specify planning scene without specifying plan parameters");
    }
    return planning_component->plan();
  }
}

bool setGoal(std::shared_ptr<moveit_cpp::PlanningComponent>& planning_component,
             std::optional<std::string> configuration_name, std::optional<moveit::core::RobotState> robot_state,
             std::optional<geometry_msgs::msg::PoseStamped> pose_stamped_msg, std::optional<std::string> pose_link,
             std::optional<std::vector<moveit_msgs::msg::Constraints>> motion_plan_constraints)
{
  // check that no more than one argument is specified
  if (configuration_name && robot_state)
  {
    throw std::invalid_argument("Cannot specify both configuration name and robot state");
  }
  else if (configuration_name && pose_stamped_msg)
  {
    throw std::invalid_argument("Cannot specify both configuration name and pose msg");
  }
  else if (configuration_name && motion_plan_constraints)
  {
    throw std::invalid_argument("Cannot specify both configuration name and motion plan constraints");
  }
  else if (robot_state && pose_stamped_msg)
  {
    throw std::invalid_argument("Cannot specify both robot state and pose msg");
  }
  else if (robot_state && motion_plan_constraints)
  {
    throw std::invalid_argument("Cannot specify both robot state and motion plan constraints");
  }
  else if (pose_stamped_msg && motion_plan_constraints)
  {
    throw std::invalid_argument("Cannot specify both pose goal and motion plan constraints");
  }
  else if ((pose_stamped_msg && !pose_link) || (!pose_stamped_msg && pose_link))
  {
    throw std::invalid_argument("Must specify both message and corresponding link");
  }

  // check that at least one argument is specified
  if (!configuration_name && !robot_state && !pose_stamped_msg && !pose_link && !motion_plan_constraints)
  {
    throw std::invalid_argument("Must specify at least one argument");
  }

  // 1. set goal from configuration name
  if (configuration_name)
  {
    return planning_component->setGoal(*configuration_name);
  }
  // 2. set goal from robot_state
  else if (robot_state)
  {
    return planning_component->setGoal(*robot_state);
  }
  // 3. set goal from pose_goal
  else if (pose_stamped_msg && pose_link)
  {
    return planning_component->setGoal(*pose_stamped_msg, *pose_link);
  }
  // 4. set goal from motion_plan_constraints
  else
  {
    return planning_component->setGoal(*motion_plan_constraints);
  }
}

bool setStartState(std::shared_ptr<moveit_cpp::PlanningComponent>& planning_component,
                   std::optional<std::string> configuration_name, std::optional<moveit::core::RobotState> robot_state)
{
  // check that no more than one argument is specified
  if (configuration_name && robot_state)
  {
    throw std::invalid_argument("Cannot specify both configuration name and robot state");
  }

  // check that at least one argument is specified
  if (!configuration_name && !robot_state)
  {
    throw std::invalid_argument("Must specify at least one argument");
  }

  // 1. set start state from configuration name
  if (configuration_name)
  {
    return planning_component->setStartState(*configuration_name);
  }
  // 2. set start state from robot_state
  else
  {
    return planning_component->setStartState(*robot_state);
  }
}

void initPlanRequestParameters(py::module& m)
{
  py::class_<moveit_cpp::PlanningComponent::PlanRequestParameters,
             std::shared_ptr<moveit_cpp::PlanningComponent::PlanRequestParameters>>(m, "PlanRequestParameters",
                                                                                    R"(
                             Planner parameters provided with a MotionPlanRequest.
                             )")
      .def(py::init([](std::shared_ptr<moveit_cpp::MoveItCpp>& moveit_cpp, const std::string& ns) {
        const rclcpp::Node::SharedPtr& node = moveit_cpp->getNode();
        moveit_cpp::PlanningComponent::PlanRequestParameters params;
        params.load(node, ns);
        return params;
      }))
      .def_readwrite("planner_id", &moveit_cpp::PlanningComponent::PlanRequestParameters::planner_id,
                     R"(
                     str: The planner id to use.
                     )")
      .def_readwrite("planning_pipeline", &moveit_cpp::PlanningComponent::PlanRequestParameters::planning_pipeline,
                     R"(
                     str: The planning pipeline to use.
                     )")
      .def_readwrite("planning_attempts", &moveit_cpp::PlanningComponent::PlanRequestParameters::planning_attempts,
                     R"(
                     int: The number of planning attempts to make.
                     )")
      .def_readwrite("planning_time", &moveit_cpp::PlanningComponent::PlanRequestParameters::planning_time,
                     R"(
                     float: The amount of time to spend planning.
                     )")
      .def_readwrite("max_velocity_scaling_factor",
                     &moveit_cpp::PlanningComponent::PlanRequestParameters::max_velocity_scaling_factor,
                     R"(
                     float: The maximum velocity scaling factor that can be used.
                     )")
      .def_readwrite("max_acceleration_scaling_factor",
                     &moveit_cpp::PlanningComponent::PlanRequestParameters::max_acceleration_scaling_factor,
                     R"(
                     float: The maximum scaling factor that can be used.
                     )");
}

void initMultiPlanRequestParameters(py::module& m)
{
  py::class_<moveit_cpp::PlanningComponent::MultiPipelinePlanRequestParameters,
             std::shared_ptr<moveit_cpp::PlanningComponent::MultiPipelinePlanRequestParameters>>(
      m, "MultiPipelinePlanRequestParameters",
      R"(
                             Planner parameters provided with a MotionPlanRequest.
                             )")
      .def(py::init([](std::shared_ptr<moveit_cpp::MoveItCpp>& moveit_cpp,
                       const std::vector<std::string>& planning_pipeline_names) {
        const rclcpp::Node::SharedPtr& node = moveit_cpp->getNode();
        moveit_cpp::PlanningComponent::MultiPipelinePlanRequestParameters params{ node, planning_pipeline_names };
        return params;
      }))
      .def_readonly("multi_plan_request_parameters",
                    &moveit_cpp::PlanningComponent::MultiPipelinePlanRequestParameters::plan_request_parameter_vector);
}
void initPlanningComponent(py::module& m)
{
  py::class_<moveit_cpp::PlanningComponent, std::shared_ptr<moveit_cpp::PlanningComponent>>(m, "PlanningComponent",
                                                                                            R"(
      Represents a joint model group and motion plans corresponding to this joint model group.
      )")

      .def(py::init<const std::string&, const std::shared_ptr<moveit_cpp::MoveItCpp>&>(),
           py::arg("joint_model_group_name"), py::arg("moveit_py_instance"),
           R"(
            Constructs a PlanningComponent instance.

	    Args:
                joint_model_group_name (str): The name of the joint model group to plan for.
                moveit_py_instance (:py:class:`moveit_py.core.MoveItPy`): The MoveItPy instance to use.
        )")

      .def("get_named_target_state_values", &moveit_cpp::PlanningComponent::getNamedTargetStateValues, py::arg("name"),
           R"(
           dict: The joint values for targets specified by name.
           )")

      .def_property("planning_group_name", &moveit_cpp::PlanningComponent::getPlanningGroupName, nullptr,
                    R"(
                    str: The name of the planning group to plan for.
                    )")

      .def_property("named_target_states", &moveit_cpp::PlanningComponent::getNamedTargetStates, nullptr,
                    R"(
                    list of str: The names of the named robot states available as targets.
                    )")

      // start state methods
      .def("set_start_state_to_current_state", &moveit_cpp::PlanningComponent::setStartStateToCurrentState,
           R"(
           Set the start state of the plan to the current state of the robot.
           )")

      .def("set_start_state", &moveit_py::bind_planning_component::setStartState,
           py::arg("configuration_name") = nullptr, py::arg("robot_state") = nullptr,
           R"(
           Set the start state of the plan to the given robot state.

	   Args:
               configuration_name (str): The name of the configuration to use as the start state.
               robot_state (:py:class:`moveit_msgs.msg.RobotState`): The robot state to use as the start state.
           )")

      .def("get_start_state", &moveit_cpp::PlanningComponent::getStartState,
           py::return_value_policy::reference_internal,
           R"(
           Returns the current start state for the planning component.
           )")

      // goal state methods
      .def("set_goal_state",
           py::overload_cast<std::shared_ptr<moveit_cpp::PlanningComponent>&, std::optional<std::string>,
                             std::optional<moveit::core::RobotState>, std::optional<geometry_msgs::msg::PoseStamped>,
                             std::optional<std::string>, std::optional<std::vector<moveit_msgs::msg::Constraints>>>(
               &moveit_py::bind_planning_component::setGoal),
           py::arg("configuration_name") = nullptr, py::arg("robot_state") = nullptr,
           py::arg("pose_stamped_msg") = nullptr, py::arg("pose_link") = nullptr,
           py::arg("motion_plan_constraints") = nullptr,
           R"(
           Set the goal state for the planning component.

	   Args:
               configuration_name (str): The name of the configuration to set the goal to.
               robot_state (moveit_py.core.RobotState): The state to set the goal to.
               pose_stamped_msg (geometry_msgs.msg.PoseStamped): A PoseStamped ros message.
               pose_link (str): The name of the link for which the pose constraint is specified.
               motion_plan_constraints (list): The motion plan constraints to set the goal to.
           )")

      // plan/execution methods

      // TODO (peterdavidfagan): improve the plan API
      .def("plan", &moveit_py::bind_planning_component::plan, py::arg("single_plan_parameters") = nullptr,
           py::arg("multi_plan_parameters") = nullptr, py::arg("planning_scene") = nullptr,
           py::arg("solution_selection_function") = nullptr, py::arg("stopping_criterion_callback") = nullptr,
           py::return_value_policy::move,
           R"(
           Plan a motion plan using the current start and goal states.

	   Args:
               plan_parameters (moveit_py.core.PlanParameters): The parameters to use for planning.
           )")

      .def("set_path_constraints", &moveit_cpp::PlanningComponent::setPathConstraints, py::arg("path_constraints"),
           py::return_value_policy::move,
           R"(
           Set the path constraints generated from a moveit msg Constraints.

	   Args:
               path_constraints (moveit_msgs.msg.Constraints): The path constraints.
        )")

      // Interacting with workspace
      .def("set_workspace", &moveit_cpp::PlanningComponent::setWorkspace, py::arg("min_x"), py::arg("min_y"),
           py::arg("min_z"), py::arg("max_x"), py::arg("max_y"), py::arg("max_z"),
           R"(
           Specify the workspace bounding box. The box is specified in the planning frame (i.e. relative to the robot root link start position). The workspace applies only to the root joint of a mobile robot (driving base, quadrotor) and does not limit the workspace of a robot arm.

	   Args:
               min_x (float): The minimum x value of the workspace.
               min_y (float): The minimum y value of the workspace.
               min_z (float): The minimum z value of the workspace.
               max_x (float): The maximum x value of the workspace.
               max_y (float): The maximum y value of the workspace.
               max_z (float): The maximum z value of the workspace.
        )")

      .def("unset_workspace", &moveit_cpp::PlanningComponent::unsetWorkspace,
           R"(
           Remove the workspace bounding box from planning.
           )");
}
}  // namespace bind_planning_component
}  // namespace moveit_py
