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

#include "collision_common.h"

namespace moveit_py
{
namespace bind_collision_detection
{
void init_collision_request(py::module& m)
{
  py::module collision_detection = m.def_submodule("collision_detection");

  py::class_<collision_detection::CollisionRequest>(collision_detection, "CollisionRequest", R"(
      Representation of a collision checking request.
      )")

      .def(py::init<>())

      .def_readwrite("joint_model_group_name", &collision_detection::CollisionRequest::group_name,
                     R"(
                     str: The group name to check collisions for (optional; if empty, assume the complete robot)
                     )")

      .def_readwrite("distance", &collision_detection::CollisionRequest::distance,
                     R"(
                     bool: If true, compute proximity distance.
                     )")

      .def_readwrite("cost", &collision_detection::CollisionRequest::cost,
                     R"(
                     bool: If true, a collision cost is computed.
                     )")

      .def_readwrite("contacts", &collision_detection::CollisionRequest::contacts,
                     R"(
                     bool: If true, compute contacts.
                     )")

      .def_readwrite("max_contacts", &collision_detection::CollisionRequest::max_contacts,
                     R"(
                     int: Overall maximum number of contacts to compute.
                     )")

      .def_readwrite("max_contacts_per_pair", &collision_detection::CollisionRequest::max_contacts_per_pair,
                     R"(
                     int: Maximum number of contacts to compute per pair of bodies (multiple bodies may be in contact at different configurations).
                     )")

      .def_readwrite("max_cost_sources", &collision_detection::CollisionRequest::max_cost_sources,
                     R"(
                     int: When costs are computed, this value defines how many of the top cost sources should be returned.
                     )")

      // TODO (peterdavidfagan): define is_done as function call.
      //.def_readwrite("is_done", &collision_detection::CollisionRequest::is_done,
      //               R"(
      //               )")

      .def_readwrite("verbose", &collision_detection::CollisionRequest::verbose,
                     R"(
                     bool: Flag indicating whether information about detected collisions should be reported.
                     )");
}

void init_collision_result(py::module& m)
{
  py::module collision_detection = m.def_submodule("collision_detection");

  py::class_<collision_detection::CollisionResult>(collision_detection, "CollisionResult", R"(
      Representation of a collision checking result.
      )")
      .def_readwrite("collision", &collision_detection::CollisionResult::collision,
                     R"(
                     bool: True if collision was found, false otherwise.
                     )")

      .def_readwrite("distance", &collision_detection::CollisionResult::distance,
                     R"(
                     float: Closest distance between two bodies.
                     )")

      .def_readwrite("contact_count", &collision_detection::CollisionResult::contact_count,
                     R"(
                     int: Number of contacts returned.
                     )")

      // TODO (peterdavidfagan): define binding and test for ContactMap.
      .def_readwrite("contacts", &collision_detection::CollisionResult::contacts,
                     R"(
                     dict: A dict returning the pairs of ids of the bodies in contact, plus information about the contacts themselves.
                     )")

      .def_readwrite("cost_sources", &collision_detection::CollisionResult::cost_sources,
                     R"(
                     dict: The individual cost sources from computed costs.
                     )");
}
}  // namespace bind_collision_detection
}  // namespace moveit_py
