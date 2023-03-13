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

#include "planning_scene.h"
#include <moveit_py/moveit_py_utils/ros_msg_typecasters.h>
#include <pybind11/operators.h>

namespace moveit_py
{
namespace bind_planning_scene
{
void apply_collision_object(std::shared_ptr<planning_scene::PlanningScene>& planning_scene,
                            moveit_msgs::msg::CollisionObject& collision_object_msg,
                            std::optional<moveit_msgs::msg::ObjectColor> color_msg)
{
  // apply collision object
  planning_scene->processCollisionObjectMsg(collision_object_msg);

  // check if color message is provided
  if (color_msg.has_value())
  {
    // set object color
    planning_scene->setObjectColor(color_msg.value().id, color_msg.value().color);
  }
}

Eigen::MatrixXd get_frame_transform(std::shared_ptr<planning_scene::PlanningScene>& planning_scene,
                                    const std::string& id)
{
  auto transformation = planning_scene->getFrameTransform(id);
  return transformation.matrix();
}

moveit_msgs::msg::PlanningScene get_planning_scene_msg(std::shared_ptr<planning_scene::PlanningScene>& planning_scene)
{
  moveit_msgs::msg::PlanningScene planning_scene_msg;
  planning_scene->getPlanningSceneMsg(planning_scene_msg);
  return planning_scene_msg;
}

void init_planning_scene(py::module& m)
{
  py::module planning_scene = m.def_submodule("planning_scene");

  py::class_<planning_scene::PlanningScene, std::shared_ptr<planning_scene::PlanningScene>>(planning_scene,
                                                                                            "PlanningScene",
                                                                                            R"(
      		     Representation of the environment as seen by a planning instance. The environment geometry, the robot geometry and state are maintained.
      		     )")

      .def(py::init<const moveit::core::RobotModelConstPtr&, const collision_detection::WorldPtr&>(),
           py::arg("robot_model"), py::arg("world") = std::make_shared<collision_detection::World>())
      // properties
      .def_property("name", &planning_scene::PlanningScene::getName, &planning_scene::PlanningScene::setName,
                    R"(
                    str: The name of the planning scene.
                    )")

      .def_property("robot_model", &planning_scene::PlanningScene::getRobotModel, nullptr,
                    py::return_value_policy::move,
                    R"(
                    :py:class:`moveit_py.core.RobotModel`: The robot model associated to this planning scene.
                    )")

      .def_property("planning_frame", &planning_scene::PlanningScene::getPlanningFrame, nullptr,
                    py::return_value_policy::move,
                    R"(
                    str: The frame in which planning is performed.
                    )")

      .def_property("current_state", &planning_scene::PlanningScene::getCurrentState,
                    py::overload_cast<const moveit::core::RobotState&>(&planning_scene::PlanningScene::setCurrentState),
                    py::return_value_policy::reference_internal,
                    R"(
                    :py:class:`moveit_py.core.RobotState`: The current state of the robot.
                    )")

      .def_property("planning_scene_message", &moveit_py::bind_planning_scene::get_planning_scene_msg, nullptr,
                    py::return_value_policy::move)

      .def_property("transforms", py::overload_cast<>(&planning_scene::PlanningScene::getTransforms), nullptr)

      // methods
      .def("__copy__",
           [](const planning_scene::PlanningScene* self) {
             return planning_scene::PlanningScene::clone(self->shared_from_this());
           })
      .def("__deepcopy__",
           [](const planning_scene::PlanningScene* self, py::dict /* memo */) {
             return planning_scene::PlanningScene::clone(self->shared_from_this());
           })
      .def("knows_frame_transform",
           py::overload_cast<const moveit::core::RobotState&, const std::string&>(
               &planning_scene::PlanningScene::knowsFrameTransform, py::const_),
           py::arg("robot_state"), py::arg("frame_id"),
           R"(
           Check if a transform to the frame id is known.
           This will be known if id is a link name, an attached body id or a collision object.
           Args:
               robot_state (:py:class:`moveit_py.core.RobotState`): The robot state to check.
               frame_id (str): The frame id to check.
           Returns:
               bool: True if the transform is known, false otherwise.
           )")

      .def("knows_frame_transform",
           py::overload_cast<const std::string&>(&planning_scene::PlanningScene::knowsFrameTransform, py::const_),
           py::arg("frame_id"),
           R"(
           Check if a transform to the frame id is known.
           This will be known if id is a link name, an attached body id or a collision object.
           Args:
               frame_id (str): The frame id to check.
           Returns:
               bool: True if the transform is known, false otherwise.
           )")

      .def("get_frame_transform", &moveit_py::bind_planning_scene::get_frame_transform, py::arg("frame_id"),
           R"(
           Get the transform corresponding to the frame id.
           This will be known if id is a link name, an attached body id or a collision object. Return identity when no transform is available.
           Args:
               frame_id (str): The frame id to get the transform for.
           Returns:
               :py:class:`numpy.ndarray`: The transform corresponding to the frame id.
           )")

      // writing to the planning scene
      .def("process_planning_scene_world", &planning_scene::PlanningScene::processPlanningSceneWorldMsg, py::arg("msg"),
           R"(
	   Process a planning scene world message.
	   Args:
	       msg (:py:class:`moveit_msgs.msg.PlanningSceneWorld`): The planning scene world message.
	   )")

      .def("apply_collision_object", &moveit_py::bind_planning_scene::apply_collision_object,
           py::arg("collision_object_msg"), py::arg("color_msg") = nullptr,
           R"(
           Apply a collision object to the planning scene.
           Args:
	   	object (moveit_msgs.msg.CollisionObject): The collision object to apply to the planning scene.
		color (moveit_msgs.msg.ObjectColor, optional): The color of the collision object. Defaults to None if not specified.
           )")

      .def("set_object_color", &planning_scene::PlanningScene::setObjectColor, py::arg("object_id"),
           py::arg("color_msg"), R"(
	   Set the color of a collision object.
	   Args:
	       object_id (str): The id of the collision object to set the color of.
	       color (std_msgs.msg.ObjectColor): The color of the collision object.
	   )")

      .def("process_attached_collision_object", &planning_scene::PlanningScene::processAttachedCollisionObjectMsg,
           py::arg("object"),
           R"(
           Apply an attached collision object to the planning scene.
           Args:
               object (moveit_msgs.msg.AttachedCollisionObject): The attached collision object to apply to the planning scene.
           )")

      .def("process_octomap",
           py::overload_cast<const octomap_msgs::msg::Octomap&>(&planning_scene::PlanningScene::processOctomapMsg),
           py::arg("msg"),
           R"(
           Apply an octomap to the planning scene.
           Args:
               octomap (moveit_msgs.msg.Octomap): The octomap to apply to the planning scene.
           )")

      .def("remove_all_collision_objects", &planning_scene::PlanningScene::removeAllCollisionObjects,
           R"(
           Removes collision objects from the planning scene.
	   This method will remove all collision object from the scene except for attached collision objects.
           )")

      // checking state validity
      .def("is_state_valid",
           py::overload_cast<const moveit::core::RobotState&, const std::string&, bool>(
               &planning_scene::PlanningScene::isStateValid, py::const_),
           py::arg("robot_state"), py::arg("joint_model_group_name"), py::arg("verbose") = false)
      .def("is_state_colliding",
           py::overload_cast<const std::string&, bool>(&planning_scene::PlanningScene::isStateColliding),
           py::arg("joint_model_group_name"), py::arg("verbose") = false,
           R"(
           Check if the robot state is in collision.
           Args:
               joint_model_group_name (str): The name of the group to check collision for.
               verbose (bool): If true, print the link names of the links in collision.
           Returns:
               bool: True if the robot state is in collision, false otherwise.
           )")

      .def("is_state_colliding",
           py::overload_cast<const moveit::core::RobotState&, const std::string&, bool>(
               &planning_scene::PlanningScene::isStateColliding, py::const_),
           py::arg("robot_state"), py::arg("joint_model_group_name"), py::arg("verbose") = false,
           R"(
           Check if the robot state is in collision.
           Args:
               robot_state (:py:class:`moveit_py.core.RobotState`): The robot state to check collision for.
               joint_model_group_name (str): The name of the group to check collision for.
               verbose (bool): If true, print the link names of the links in collision.
           Returns:
               bool: True if the robot state is in collision, false otherwise.
           )")

      .def("is_state_constrained",
           py::overload_cast<const moveit::core::RobotState&, const moveit_msgs::msg::Constraints&, bool>(
               &planning_scene::PlanningScene::isStateConstrained, py::const_),
           py::arg("state"), py::arg("constraints"), py::arg("verbose") = false,
           R"(
           Check if the robot state fulfills the passed constraints
           Args:
               state (moveit_py.core.RobotState): The robot state to check constraints for.
	       constraints (moveit_msgs.msg.Constraints): The constraints to check.
	       verbose (bool):
           Returns:
               bool: true if state is constrained otherwise false.
           )")

      .def("is_path_valid",
           py::overload_cast<const robot_trajectory::RobotTrajectory&, const std::string&, bool,
                             std::vector<std::size_t>*>(&planning_scene::PlanningScene::isPathValid, py::const_),
           py::arg("trajectory"), py::arg("joint_model_group_name"), py::arg("verbose") = false,
           py::arg("invalid_index") = nullptr,
           R"(
           Check if a given path is valid.
           Each state is checked for validity (collision avoidance and feasibility)
           Args:
               trajectory (:py:class:`moveit_py.core.RobotTrajectory`): The trajectory to check.
               joint_model_group_name (str): The joint model group to check the path against.
               verbose (bool):
	       invalid_index (list):
           Returns:
               bool: true if the path is valid otherwise false.
           )")

      // TODO (peterdavidfagan): remove collision result from input parameters and write separate binding code.
      // TODO (peterdavidfagan): consider merging check_collision and check_collision_unpadded into one function with unpadded_param
      .def("check_collision",
           py::overload_cast<const collision_detection::CollisionRequest&, collision_detection::CollisionResult&>(
               &planning_scene::PlanningScene::checkCollision),
           py::arg("collision_request"), py::arg("collision_result"),
           R"(
           Check whether the current state is in collision, and if needed, updates the collision transforms of the current state before the computation.
           Args:
               collision_request (:py:class:`moveit_py.core.CollisionRequest`): The collision request to use.
               collision_result (:py:class:`moveit_py.core.CollisionResult`): The collision result to update
           Returns:
               bool: true if state is in collision otherwise false.
           )")

      .def("check_collision",
           py::overload_cast<const collision_detection::CollisionRequest&, collision_detection::CollisionResult&,
                             moveit::core::RobotState&>(&planning_scene::PlanningScene::checkCollision, py::const_),
           py::arg("collision_request"), py::arg("collision_result"), py::arg("state"),
           R"(
           Check if the robot state is in collision.
           Args:
               collision_request ():
               collision_result ():
               state ():
           Returns:
               bool: true if state is in collision otherwise false.
           )")

      .def("check_collision",
           py::overload_cast<const collision_detection::CollisionRequest&, collision_detection::CollisionResult&,
                             moveit::core::RobotState&, const collision_detection::AllowedCollisionMatrix&>(
               &planning_scene::PlanningScene::checkCollision, py::const_),
           py::arg("collision_request"), py::arg("collision_result"), py::arg("state"), py::arg("acm"),
           R"(
           Check if the robot state is in collision.
           Args:
               collision_request ():
               collision_result ():
               state ():
           acm ():
           Returns:
               bool: true if state is in collision otherwise false.
           )")

      .def("check_collision_unpadded",
           py::overload_cast<const collision_detection::CollisionRequest&, collision_detection::CollisionResult&>(
               &planning_scene::PlanningScene::checkCollisionUnpadded),
           py::arg("req"), py::arg("result"),
           R"(
           Check if the robot state is in collision.
           Args:
               collision_request ():
               collision_result ():
           Returns:
               bool: true if state is in collision otherwise false.
           )")

      .def("check_collision_unpadded",
           py::overload_cast<const collision_detection::CollisionRequest&, collision_detection::CollisionResult&,
                             moveit::core::RobotState&>(&planning_scene::PlanningScene::checkCollisionUnpadded,
                                                        py::const_),
           py::arg("collision_request"), py::arg("collision_result"), py::arg("state"),
           R"(
           Check if the robot state is in collision.
           Args:
               collision_request ():
               collision_result ():
               state ():
           Returns:
               bool: true if state is in collision otherwise false.
           )")

      .def("check_collision_unpadded",
           py::overload_cast<const collision_detection::CollisionRequest&, collision_detection::CollisionResult&,
                             moveit::core::RobotState&, const collision_detection::AllowedCollisionMatrix&>(
               &planning_scene::PlanningScene::checkCollisionUnpadded, py::const_),
           py::arg("collision_request"), py::arg("collision_result"), py::arg("state"), py::arg("acm"),
           R"(
           Check if the robot state is in collision.
           Args:
               collision_request ():
               collision_result ():
               state ():
           acm ():
           Returns:
               bool: true if state is in collision otherwise false.
           )")

      .def("check_self_collision",
           py::overload_cast<const collision_detection::CollisionRequest&, collision_detection::CollisionResult&>(
               &planning_scene::PlanningScene::checkSelfCollision),
           py::arg("collision_request"), py::arg("collision_result"),
           R"(
           Check if the robot state is in collision.
           Args:
               collision_request ():
               collision_result ():
           Returns:
               bool: true if state is in collision otherwise false.
           )")

      .def("check_self_collision",
           py::overload_cast<const collision_detection::CollisionRequest&, collision_detection::CollisionResult&,
                             moveit::core::RobotState&>(&planning_scene::PlanningScene::checkSelfCollision, py::const_),
           py::arg("collision_request"), py::arg("collision_result"), py::arg("state"),
           R"(
           Check if the robot state is in collision.
           Args:
               collision request ():
               collision_result ():
               state ():
           Returns:
               bool: true if state is in self collision otherwise false.
           )")

      .def("check_self_collision",
           py::overload_cast<const collision_detection::CollisionRequest&, collision_detection::CollisionResult&,
                             moveit::core::RobotState&, const collision_detection::AllowedCollisionMatrix&>(
               &planning_scene::PlanningScene::checkSelfCollision, py::const_),
           py::arg("collision_request"), py::arg("collision_result"), py::arg("state"), py::arg("acm"),
           R"(
           Check if the robot state is in collision.
           Args:
               collision request ():
               collision_result ():
               state ():
           acm():
           Returns:
               bool: true if state is in self collision otherwise false.
           )");
}
}  // namespace bind_planning_scene
}  // namespace moveit_py
