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

#include "collision_common.hpp"

namespace moveit_py
{
namespace bind_collision_detection
{
void initBodyType(py::module& m)
{
  py::module collision_detection = m.def_submodule("collision_detection");

  py::enum_<collision_detection::BodyType>(collision_detection, "BodyType", R"(
      The type of a body involved in a collision contact.
      )")
      .value("ROBOT_LINK", collision_detection::BodyTypes::ROBOT_LINK, "A link on the robot.")
      .value("ROBOT_ATTACHED", collision_detection::BodyTypes::ROBOT_ATTACHED, "A body attached to a robot link.")
      .value("WORLD_OBJECT", collision_detection::BodyTypes::WORLD_OBJECT, "A body in the environment.")
      .export_values();
}

void initContact(py::module& m)
{
  py::module collision_detection = m.def_submodule("collision_detection");

  py::class_<collision_detection::Contact>(collision_detection, "Contact", R"(
      A contact point between two bodies.
      )")
      .def(py::init<>())
      .def_readwrite("body_name_1", &collision_detection::Contact::body_name_1, R"(
          str: The name of the first body in contact.
          )")
      .def_readwrite("body_name_2", &collision_detection::Contact::body_name_2, R"(
          str: The name of the second body in contact.
          )")
      .def_readwrite("body_type_1", &collision_detection::Contact::body_type_1, R"(
          BodyType: The type of the first body in contact.
          )")
      .def_readwrite("body_type_2", &collision_detection::Contact::body_type_2, R"(
          BodyType: The type of the second body in contact.
          )")
      .def_readwrite("depth", &collision_detection::Contact::depth, R"(
          float: Penetration depth between the two bodies.
          )")
      .def_readwrite("pos", &collision_detection::Contact::pos, R"(
          numpy.ndarray: Contact position (3D vector).
          )")
      .def_readwrite("normal", &collision_detection::Contact::normal, R"(
          numpy.ndarray: Normal unit vector at the contact point (3D vector).
          )")
      .def_readwrite("percent_interpolation", &collision_detection::Contact::percent_interpolation, R"(
          float: Distance percentage between cast poses until collision (0 = start pose, 1 = end pose).
          )");
}

void initCostSource(py::module& m)
{
  py::module collision_detection = m.def_submodule("collision_detection");

  py::class_<collision_detection::CostSource>(collision_detection, "CostSource", R"(
      A partial cost incurred in a particular volume due to a collision.
      )")
      .def(py::init<>())
      .def_readwrite("aabb_min", &collision_detection::CostSource::aabb_min, R"(
          list[float]: Minimum bound of the AABB defining the volume responsible for this cost.
          )")
      .def_readwrite("aabb_max", &collision_detection::CostSource::aabb_max, R"(
          list[float]: Maximum bound of the AABB defining the volume responsible for this cost.
          )")
      .def_readwrite("cost", &collision_detection::CostSource::cost, R"(
          float: The partial cost (probability of object existence at collision point).
          )")
      .def("get_volume", &collision_detection::CostSource::getVolume, R"(
          float: Volume of the AABB around this cost source.
          )");
}

void initCollisionRequest(py::module& m)
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

void initCollisionResult(py::module& m)
{
  py::module collision_detection = m.def_submodule("collision_detection");

  py::class_<collision_detection::CollisionResult>(collision_detection, "CollisionResult", R"(
      Representation of a collision checking result.
      )")

      .def(py::init<>())

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

      .def_property_readonly(
          "contacts",
          [](const collision_detection::CollisionResult& self) {
            py::dict result;
            for (const auto& [key, contact_vec] : self.contacts)
            {
              py::list py_contacts;
              for (const auto& contact : contact_vec)
                py_contacts.append(contact);
              result[py::make_tuple(key.first, key.second)] = py_contacts;
            }
            return result;
          },
          R"(
          dict[tuple[str, str], list[Contact]]: Pairs of body ids in contact mapped to their contact details.
          )")

      .def_property_readonly(
          "cost_sources",
          [](const collision_detection::CollisionResult& self) {
            py::list result;
            for (const auto& cs : self.cost_sources)
              result.append(cs);
            return result;
          },
          R"(
          list[CostSource]: Individual cost sources from computed costs, ordered highest cost first.
          )");
}
}  // namespace bind_collision_detection
}  // namespace moveit_py
