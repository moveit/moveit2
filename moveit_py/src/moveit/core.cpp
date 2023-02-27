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

#include "moveit_core/collision_detection/collision_common.h"
#include "moveit_core/collision_detection/collision_matrix.h"
#include "moveit_core/collision_detection/world.h"
#include "moveit_core/controller_manager/controller_manager.h"
#include "moveit_core/kinematic_constraints/utils.h"
#include "moveit_core/planning_interface/planning_response.h"
#include "moveit_core/planning_scene/planning_scene.h"
#include "moveit_core/robot_model/joint_model.h"
#include "moveit_core/robot_model/joint_model_group.h"
#include "moveit_core/robot_model/robot_model.h"
#include "moveit_core/robot_state/robot_state.h"
#include "moveit_core/robot_trajectory/robot_trajectory.h"

PYBIND11_MODULE(core, m)
{
  m.doc() = R"(
            Python bindings for moveit_core functionalities.
            )";

  // Provide custom function signatures
  py::options options;
  options.disable_function_signatures();

  // Construct module classes
  moveit_py::bind_collision_detection::init_collision_request(m);
  moveit_py::bind_collision_detection::init_collision_result(m);
  moveit_py::bind_collision_detection::init_world(m);
  moveit_py::bind_collision_detection::init_acm(m);
  moveit_py::bind_controller_manager::init_execution_status(m);
  moveit_py::bind_kinematic_constraints::init_kinematic_constraints(m);
  moveit_py::bind_planning_scene::init_planning_scene(m);
  moveit_py::bind_planning_interface::init_motion_plan_response(m);
  moveit_py::bind_robot_model::init_joint_model(m);
  moveit_py::bind_robot_model::init_joint_model_group(m);
  moveit_py::bind_robot_model::init_robot_model(m);
  moveit_py::bind_robot_state::init_robot_state(m);
  moveit_py::bind_robot_trajectory::init_robot_trajectory(m);
  // TODO (peterdavidfagan): complete LinkModel bindings
  // LinkModel
  // py::class_<moveit::core::LinkModel>(m, "LinkModel");

  // TODO (peterdavidfagan): complete JointModel bindings
  // JointModel (this is an abstract base class)
  // py::class_<moveit::core::JointModel>(m, "JointModel");
}
