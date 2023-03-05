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

#include "robot_state.h"
#include <pybind11/stl.h>
#include <moveit_py/moveit_py_utils/ros_msg_typecasters.h>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit/robot_state/conversions.h>

namespace moveit_py
{
namespace bind_robot_state
{
void update(moveit::core::RobotState* self, bool force, std::string& category)
{
  if (category == "all")
  {
    self->update(force);
  }
  else if (category == "links_only")
  {
    self->updateLinkTransforms();
  }
  else if (category == "collisions_only")
  {
    self->updateCollisionBodyTransforms();
  }
  else
  {
    throw std::invalid_argument("Invalid category");
  }
}

Eigen::MatrixXd get_frame_transform(const moveit::core::RobotState* self, std::string& frame_id)
{
  bool frame_found;
  auto transformation = self->getFrameTransform(frame_id, &frame_found);
  return transformation.matrix();
}

Eigen::MatrixXd get_global_link_transform(const moveit::core::RobotState* self, std::string& link_name)
{
  auto transformation = self->getGlobalLinkTransform(link_name);
  return transformation.matrix();
}

geometry_msgs::msg::Pose get_pose(const moveit::core::RobotState* self, const std::string& link_name)
{
  Eigen::Isometry3d pose = self->getGlobalLinkTransform(link_name);
  return tf2::toMsg(pose);
}

std::map<std::string, double> get_joint_positions(const moveit::core::RobotState* self)
{
  std::map<std::string, double> joint_positions;
  const std::vector<std::string>& variable_name = self->getVariableNames();
  for (auto& name : variable_name)
  {
    joint_positions[name.c_str()] = self->getVariablePosition(name);
  }
  return joint_positions;
}

void set_joint_positions(moveit::core::RobotState* self, std::map<std::string, double>& joint_positions)
{
  for (const auto& item : joint_positions)
  {
    self->setVariablePosition(item.first, item.second);
  }
}

std::map<std::string, double> get_joint_velocities(const moveit::core::RobotState* self)
{
  std::map<std::string, double> joint_velocity;
  const std::vector<std::string>& variable_name = self->getVariableNames();
  for (auto& name : variable_name)
  {
    joint_velocity[name.c_str()] = self->getVariableVelocity(name);
  }
  return joint_velocity;
}

void set_joint_velocities(moveit::core::RobotState* self, std::map<std::string, double>& joint_velocities)
{
  for (const auto& item : joint_velocities)
  {
    self->setVariableVelocity(item.first, item.second);
  }
}

std::map<std::string, double> get_joint_accelerations(const moveit::core::RobotState* self)
{
  std::map<std::string, double> joint_acceleration;
  const std::vector<std::string>& variable_name = self->getVariableNames();
  for (auto& name : variable_name)
  {
    joint_acceleration[name.c_str()] = self->getVariableAcceleration(name);
  }
  return joint_acceleration;
}

void set_joint_accelerations(moveit::core::RobotState* self, std::map<std::string, double>& joint_accelerations)
{
  for (const auto& item : joint_accelerations)
  {
    self->setVariableAcceleration(item.first, item.second);
  }
}

std::map<std::string, double> get_joint_efforts(const moveit::core::RobotState* self)
{
  std::map<std::string, double> joint_effort;
  const std::vector<std::string>& variable_name = self->getVariableNames();
  for (auto& name : variable_name)
  {
    joint_effort[name.c_str()] = self->getVariableEffort(name);
  }
  return joint_effort;
}

void set_joint_efforts(moveit::core::RobotState* self, std::map<std::string, double>& joint_efforts)
{
  for (const auto& item : joint_efforts)
  {
    self->setVariableEffort(item.first, item.second);
  }
}

Eigen::VectorXd copy_joint_group_positions(const moveit::core::RobotState* self,
                                           const std::string& joint_model_group_name)
{
  Eigen::VectorXd values;
  self->copyJointGroupPositions(joint_model_group_name, values);
  return values;
}

Eigen::VectorXd copy_joint_group_velocities(const moveit::core::RobotState* self,
                                            const std::string& joint_model_group_name)
{
  Eigen::VectorXd values;
  self->copyJointGroupVelocities(joint_model_group_name, values);
  return values;
}

Eigen::VectorXd copy_joint_group_accelerations(const moveit::core::RobotState* self,
                                               const std::string& joint_model_group_name)
{
  Eigen::VectorXd values;
  self->copyJointGroupAccelerations(joint_model_group_name, values);
  return values;
}

Eigen::MatrixXd get_jacobian(const moveit::core::RobotState* self, const std::string& joint_model_group_name,
                             const Eigen::Vector3d& reference_point_position)
{
  const moveit::core::JointModelGroup* joint_model_group = self->getJointModelGroup(joint_model_group_name);
  return self->getJacobian(joint_model_group, reference_point_position);
}

Eigen::MatrixXd get_jacobian(const moveit::core::RobotState* self, const std::string& joint_model_group_name,
                             const std::string& link_model_name, const Eigen::Vector3d& reference_point_position,
                             bool use_quaternion_representation)
{
  Eigen::MatrixXd jacobian;
  const moveit::core::JointModelGroup* joint_model_group = self->getJointModelGroup(joint_model_group_name);
  const moveit::core::LinkModel* link_model = self->getLinkModel(link_model_name);
  self->getJacobian(joint_model_group, link_model, reference_point_position, jacobian, use_quaternion_representation);
  return jacobian;
}

bool set_to_default_values(moveit::core::RobotState* self, const std::string& joint_model_group_name,
                           const std::string& state_name)

{
  const moveit::core::JointModelGroup* joint_model_group = self->getJointModelGroup(joint_model_group_name);
  return self->setToDefaultValues(joint_model_group, state_name);
}

void init_robot_state(py::module& m)
{
  py::module robot_state = m.def_submodule("robot_state");

  robot_state.def(
      "robotStateToRobotStateMsg",
      [](const moveit::core::RobotState& state, bool copy_attached_bodies) {
        moveit_msgs::msg::RobotState state_msg;
        moveit::core::robotStateToRobotStateMsg(state, state_msg, copy_attached_bodies);
        return state_msg;
      },
      py::arg("state"), py::arg("copy_attached_bodies") = true);

  py::class_<moveit::core::RobotState, std::shared_ptr<moveit::core::RobotState>>(robot_state, "RobotState",
                                                                                  R"(
          Representation of a robot's state.
          At the lowest level, a state is a collection of variables. Each variable has a name and can have position, velocity, acceleration and effort associated to it. Effort and acceleration share the memory area for efficiency reasons (one should not set both acceleration and effort in the same state and expect things to work). Often variables correspond to joint names as well (joints with one degree of freedom have one variable), but joints with multiple degrees of freedom have more variables. Operations are allowed at variable level, joint level (see JointModel) and joint group level (see JointModelGroup).
                  For efficiency reasons a state computes forward kinematics in a lazy fashion. This can sometimes lead to problems if the update() function was not called on the state.
          )")

      .def(py::init<const std::shared_ptr<const moveit::core::RobotModel>&>(),
           R"(
           Initializes robot state from a robot model.

           Args:
               :py:class:`moveit_py.core.RobotModel`: The robot model associated to the instantiated robot state.

           )")
      .def("__copy__", [](const moveit::core::RobotState* self) { return moveit::core::RobotState{ *self }; })
      .def("__deepcopy__",
           [](const moveit::core::RobotState* self, py::dict /* memo */) { return moveit::core::RobotState{ *self }; })

      // Get underlying robot model, frame transformations and jacobian
      .def_property("robot_model", &moveit::core::RobotState::getRobotModel, nullptr,
                    py::return_value_policy::reference,
                    R"(
                    :py:class:`moveit_py.core.RobotModel`: The robot model instance associated to this robot state.
                    )")

      .def_property("dirty", &moveit::core::RobotState::dirty, nullptr,
                    R"(
            bool: True if the robot state is dirty.
            )")

      .def("get_frame_transform", &moveit_py::bind_robot_state::get_frame_transform, py::arg("frame_id"),
           py::return_value_policy::move,
           R"(
           Get the transformation matrix from the model frame (root of model) to the frame identified by frame_id.
           If frame_id was not found, frame_found is set to false and an identity transform is returned.
       This method is restricted to frames defined within the robot state and doesn't include collision object present in the collision world. Please use the PlanningScene.get_frame_transform method for collision world objects.
           Args:
               frame_id (str): The id of the frame to get the transform for.
           Returns:
               :py:class:`numpy.ndarray`: The transformation matrix from the model frame to the frame identified by frame_id.
           )")

      .def("get_pose", &moveit_py::bind_robot_state::get_pose, py::arg("link_name"),
           R"(
           Get the pose of a link that is defined in the robot model.
           Args:
               link_name (str): The name of the link to get the pose for.
           Returns:
               :py:class: `geometry_msgs.msg.Pose`: A ROS geometry message containing the pose of the link.
           )")

      .def("get_jacobian",
           py::overload_cast<const moveit::core::RobotState*, const std::string&, const Eigen::Vector3d&>(
               &moveit_py::bind_robot_state::get_jacobian),
           py::arg("joint_model_group_name"), py::arg("reference_point_position"), py::return_value_policy::move,
           R"(
           Compute the Jacobian with reference to the last link of a specified group.
           Args:
               joint_model_group_name (str): The name of the joint model group to compute the Jacobian for.
               reference_point_position (:py:class:`numpy.ndarray`): The position of the reference point in the link frame.
           Returns:
               :py:class:`numpy.ndarray`: The Jacobian of the specified group with respect to the reference point.
           Raises:
               Exception: If the group is not a chain.
           )")

      .def("get_jacobian",
           py::overload_cast<const moveit::core::RobotState*, const std::string&, const std::string&,
                             const Eigen::Vector3d&, bool>(&moveit_py::bind_robot_state::get_jacobian),
           py::arg("joint_model_group_name"), py::arg("link_name"), py::arg("reference_point_position"),
           py::arg("use_quaternion_representation") = false, py::return_value_policy::move,
           R"(
           Compute the Jacobian with reference to a particular point on a given link, for a specified group.
           Args:
               joint_model_group_name (str): The name of the joint model group to compute the Jacobian for.
               link_name (str): The name of the link model to compute the Jacobian for.
               reference_point_position (:py:class:`numpy.ndarray`): The position of the reference point in the link frame.
	       use_quaternion_representation (bool): If true, the Jacobian will be represented using a quaternion representation, if false it defaults to euler angle representation.
           Returns:
               :py:class:`numpy.ndarray`: The Jacobian of the specified group with respect to the reference point.
           )")

      // Get state information
      .def_property("state_tree", py::overload_cast<>(&moveit::core::RobotState::getStateTreeString, py::const_),
                    nullptr, py::return_value_policy::move,
                    R"(
                    str: represents the state tree of the robot state.
                    )")

      .def_property_readonly_static(
          "state_info",
          [](const moveit::core::RobotState& s) {
            std::stringstream ss;
            s.printStateInfo(ss);
            return ss.str();
          },
          py::return_value_policy::move,
          R"(
	  str: the state information of the robot state.
	  )")

      // Getting and setting joint model group positions, velocities, accelerations
      .def_property("joint_positions", &moveit_py::bind_robot_state::get_joint_positions,
                    &moveit_py::bind_robot_state::set_joint_positions, py::return_value_policy::copy)

      .def_property("joint_velocities", &moveit_py::bind_robot_state::get_joint_velocities,
                    &moveit_py::bind_robot_state::set_joint_velocities, py::return_value_policy::copy)

      .def_property("joint_accelerations", &moveit_py::bind_robot_state::get_joint_accelerations,
                    &moveit_py::bind_robot_state::set_joint_accelerations, py::return_value_policy::copy)

      .def_property("joint_efforts", &moveit_py::bind_robot_state::get_joint_efforts,
                    &moveit_py::bind_robot_state::set_joint_efforts, py::return_value_policy::copy)

      .def("set_joint_group_positions",
           py::overload_cast<const std::string&, const Eigen::VectorXd&>(
               &moveit::core::RobotState::setJointGroupPositions),
           py::arg("joint_model_group_name"), py::arg("position_values"),
           R"(
           Sets the positions of the joints in the specified joint model group.
           Args:
               joint_model_group_name (str):
               position_values (:py:class:`numpy.ndarray`): The positions of the joints in the joint model group.
       )")

      // peterdavidfagan: I am not sure if additional function names are better than having function parameters for joint setting.
      .def("set_joint_group_active_positions",
           py::overload_cast<const std::string&, const Eigen::VectorXd&>(
               &moveit::core::RobotState::setJointGroupActivePositions),
           py::arg("joint_model_group_name"), py::arg("position_values"),
           R"(
           Sets the active positions of joints in the specified joint model group.

           Args:
               joint_model_group_name (str): The name of the joint model group to set the active positions for.
               position_values (:py:class:`numpy.ndarray`): The positions of the joints in the joint model group.
       )")

      .def("get_joint_group_positions", &moveit_py::bind_robot_state::copy_joint_group_positions,
           py::arg("joint_model_group_name"),
           R"(
           For a given group, get the position values of the variables that make up the group.
           Args:
               joint_model_group_name (str): The name of the joint model group to copy the positions for.
           Returns:
               :py:class:`numpy.ndarray`: The positions of the joints in the joint model group.
           )")

      .def("set_joint_group_velocities",
           py::overload_cast<const std::string&, const Eigen::VectorXd&>(
               &moveit::core::RobotState::setJointGroupVelocities),
           py::arg("joint_model_group_name"), py::arg("velocity_values"),
           R"(
           Sets the velocities of the joints in the specified joint model group.
           Args:
               joint_model_group_name (str): The name of the joint model group to set the velocities for.
               velocity_values (:py:class:`numpy.ndarray`): The velocities of the joints in the joint model group.
           )")

      .def("get_joint_group_velocities", &moveit_py::bind_robot_state::copy_joint_group_velocities,
           py::arg("joint_model_group_name"),
           R"(
           For a given group, get the velocity values of the variables that make up the group.

           Args:
               joint_model_group_name (str): The name of the joint model group to copy the velocities for.
           Returns:
               :py:class:`numpy.ndarray`: The velocities of the joints in the joint model group.
       )")

      .def("set_joint_group_accelerations",
           py::overload_cast<const std::string&, const Eigen::VectorXd&>(
               &moveit::core::RobotState::setJointGroupAccelerations),
           py::arg("joint_model_group_name"), py::arg("acceleration_values"),
           R"(
           Sets the accelerations of the joints in the specified joint model group.

           Args:
               joint_model_group_name (str): The name of the joint model group to set the accelerations for.
               acceleration_values (:py:class:`numpy.ndarray`): The accelerations of the joints in the joint model group.
           )")

      .def("get_joint_group_accelerations", &moveit_py::bind_robot_state::copy_joint_group_accelerations,
           py::arg("joint_model_group_name"),
           R"(
           For a given group, get the acceleration values of the variables that make up the group.
           Args:
               joint_model_group_name (str): The name of the joint model group to copy the accelerations for.
           Returns:
               :py:class:`numpy.ndarray`: The accelerations of the joints in the joint model group.
           )")

      // Forward kinematics
      .def("get_global_link_transform", &moveit_py::bind_robot_state::get_global_link_transform, py::arg("link_name"),
           R"(
       Returns the transform of the specified link in the global frame.
       Args:
           link_name (str): The name of the link to get the transform for.
       Returns:
           :py:class:`numpy.ndarray`: The transform of the specified link in the global frame.
       )")

      // Setting state from inverse kinematics
      .def(
          "set_from_ik",
          [](moveit::core::RobotState* self, const std::string& group, const geometry_msgs::msg::Pose& pose,
             const std::string& tip,
             double timeout) { return self->setFromIK(self->getJointModelGroup(group), pose, tip, timeout); },
          py::arg("joint_model_group_name"), py::arg("geometry_pose"), py::arg("tip_name"), py::arg("timeout") = 0.0,
          R"(
           Sets the state of the robot to the one that results from solving the inverse kinematics for the specified group.
           Args:
               joint_model_group_name (str): The name of the joint model group to set the state for.
               geometry_pose (:py:class: `geometry_msgs.msg.Pose`): The pose of the end-effector to solve the inverse kinematics for.
               tip_name (str): The name of the link that is the tip of the end-effector.
               timeout (float): The amount of time to wait for the IK solution to be found.
           )")

      // Setting entire state values
      .def("set_to_default_values", py::overload_cast<>(&moveit::core::RobotState::setToDefaultValues),
           R"(
           Set all joints to their default positions.
           The default position is 0, or if that is not within bounds then half way between min and max bound.
           )")

      .def("set_to_default_values",
           py::overload_cast<const moveit::core::JointModelGroup*, const std::string&>(
               &moveit::core::RobotState::setToDefaultValues),
           py::arg("joint_model_group"), py::arg("name"),
           R"(
           Set the joints in group to the position name defined in the SRDF.
           Args:
               joint_model_group (:py:class:`moveit_py.core.JointModelGroup`): The joint model group to set the default values for.
               name (str): The name of a predefined state which is defined in the robot model SRDF.
           )")

      .def("set_to_default_values",
           py::overload_cast<moveit::core::RobotState*, const std::string&, const std::string&>(
               &moveit_py::bind_robot_state::set_to_default_values),
           py::arg("joint_model_group_name"), py::arg("name"),
           R"(
           Set the joints in group to the position name defined in the SRDF.
           Args:
               joint_model_group_name (str): The name of the joint model group to set the default values for.
               name (str): The name of a predefined state which is defined in the robot model SRDF.
       )")

      .def("set_to_random_positions", py::overload_cast<>(&moveit::core::RobotState::setToRandomPositions),
           R"(
           Set all joints to random positions within the default bounds.
           )")

      .def("set_to_random_positions",
           py::overload_cast<const moveit::core::JointModelGroup*>(&moveit::core::RobotState::setToRandomPositions),
           py::arg("joint_model_group"),
           R"(
           Set all joints in the joint model group to random positions within the default bounds.
           Args:
               joint_model_group (:py:class:`moveit_py.core.JointModelGroup`): The joint model group to set the random values for.
           )")

      .def("clear_attached_bodies", py::overload_cast<>(&moveit::core::RobotState::clearAttachedBodies),
           R"(
      	   Clear all attached bodies.

      	   We only allow for attaching of objects via the PlanningScene instance. This method allows any attached objects that are associated to this RobotState instance to be removed.
	   )")

      .def("update", &moveit_py::bind_robot_state::update, py::arg("force") = false, py::arg("type") = "all",
           R"(
             Update state transforms.

             Args:
	     	force (bool): when true forces the update of the transforms from scratch.
		category (str): specifies the category to update. All indicates updating all transforms while "links_only" and "collisions_only" ensure that only links or collision transforms are updated. )");
}
}  // namespace bind_robot_state
}  // namespace moveit_py
