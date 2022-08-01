/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Larry Lu, Ioan Sucan */

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/py_bindings_tools/py_conversions.h>
#include <moveit/py_bindings_tools/serialize_msg.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

/** @cond IGNORE */

namespace py = pybind11;

namespace moveit
{
namespace planning_interface
{
class PlanningSceneInterfacePython : public PlanningSceneInterface
{
public:
  // ROSInitializer is constructed first, and ensures ros::init() was called, if needed
  PlanningSceneInterfacePython(const std::string& ns = "") : PlanningSceneInterface(ns)
  {
  }

  py::list getKnownObjectNamesPython(bool with_type = false)
  {
    return py_bindings_tools::listFromString(getKnownObjectNames(with_type));
  }

  py::list getKnownObjectNamesInROIPython(double minx, double miny, double minz, double maxx, double maxy, double maxz,
                                          bool with_type = false)
  {
    return py_bindings_tools::listFromString(getKnownObjectNamesInROI(minx, miny, minz, maxx, maxy, maxz, with_type));
  }

  py::dict getObjectPosesPython(const py::list& object_ids)
  {
    std::map<std::string, geometry_msgs::msg::Pose> ops = getObjectPoses(py_bindings_tools::stringFromList(object_ids));
    std::map<std::string, py::bytes> ser_ops;
    for (std::map<std::string, geometry_msgs::msg::Pose>::const_iterator it = ops.begin(); it != ops.end(); ++it)
      ser_ops[py::str(it->first)] = py_bindings_tools::serializeMsg(it->second);

    return py_bindings_tools::dictFromType(ser_ops);
  }

  py::dict getObjectsPython(const py::list& object_ids)
  {
    std::map<std::string, moveit_msgs::msg::CollisionObject> objs =
        getObjects(py_bindings_tools::stringFromList(object_ids));
    std::map<std::string, py::bytes> ser_objs;
    for (std::map<std::string, moveit_msgs::msg::CollisionObject>::const_iterator it = objs.begin(); it != objs.end();
         ++it)
      ser_objs[py::str(it->first)] = py_bindings_tools::serializeMsg(it->second);

    return py_bindings_tools::dictFromType(ser_objs);
  }

  py::dict getAttachedObjectsPython(const py::list& object_ids)
  {
    std::map<std::string, moveit_msgs::msg::AttachedCollisionObject> aobjs =
        getAttachedObjects(py_bindings_tools::stringFromList(object_ids));
    std::map<std::string, py::bytes> ser_aobjs;
    for (std::map<std::string, moveit_msgs::msg::AttachedCollisionObject>::const_iterator it = aobjs.begin();
         it != aobjs.end(); ++it)
      ser_aobjs[py::str(it->first)] = py_bindings_tools::serializeMsg(it->second);

    return py_bindings_tools::dictFromType(ser_aobjs);
  }

  bool applyCollisionObjectPython(const py::bytes& co_str)
  {
    moveit_msgs::msg::CollisionObject co_msg;
    py_bindings_tools::deserializeMsg(co_str, co_msg);
    return applyCollisionObject(co_msg);
  }

  bool applyCollisionObjectWithColorPython(const py::bytes& co_str, const py::bytes& oc_str)
  {
    moveit_msgs::msg::CollisionObject co_msg;
    py_bindings_tools::deserializeMsg(co_str, co_msg);
    std_msgs::msg::ColorRGBA oc_msg;
    py_bindings_tools::deserializeMsg(oc_str, oc_msg);
    return applyCollisionObject(co_msg, oc_msg);
  }

  bool applyCollisionObjectsWithColorPython(const py::list& co_strs, const py::list& oc_strs)
  {
    std::vector<moveit_msgs::msg::CollisionObject> co_msgs;
    int l_co = py::len(co_strs);
    for (int i = 0; i < l_co; ++i)
    {
      const py::bytes co_str = (co_strs[i]).cast<py::bytes>();
      moveit_msgs::msg::CollisionObject co_msg;
      py_bindings_tools::deserializeMsg(co_str, co_msg);
      co_msgs.push_back(co_msg);
    }
    std::vector<moveit_msgs::msg::ObjectColor> oc_msgs;
    int l_oc = py::len(oc_strs);
    for (int i = 0; i < l_oc; ++i)
    {
      const py::bytes oc_str = (oc_strs[i]).cast<py::bytes>();
      moveit_msgs::msg::ObjectColor oc_msg;
      py_bindings_tools::deserializeMsg(oc_str, oc_msg);
      oc_msgs.push_back(oc_msg);
    }
    return applyCollisionObjects(co_msgs, oc_msgs);
  }

  bool applyAttachedCollisionObjectPython(const py::bytes& co_str)
  {
    moveit_msgs::msg::AttachedCollisionObject co_msg;
    py_bindings_tools::deserializeMsg(co_str, co_msg);
    return applyAttachedCollisionObject(co_msg);
  }

  bool applyAttachedCollisionObjectsPython(const py::list& co_strs)
  {
    std::vector<moveit_msgs::msg::AttachedCollisionObject> co_msgs;
    int l_co = py::len(co_strs);
    for (int i = 0; i < l_co; ++i)
    {
      const py::bytes co_str = (co_strs[i]).cast<py::bytes>();
      moveit_msgs::msg::AttachedCollisionObject co_msg;
      py_bindings_tools::deserializeMsg(co_str, co_msg);
      co_msgs.push_back(co_msg);
    }
    return applyAttachedCollisionObjects(co_msgs);
  }

  bool applyPlanningScenePython(const py::bytes& ps_str)
  {
    moveit_msgs::msg::PlanningScene ps_msg;
    py_bindings_tools::deserializeMsg(ps_str, ps_msg);
    return applyPlanningScene(ps_msg);
  }

  void addCollisionObjectsPython(const py::list& co_strs, const py::list& oc_strs)
  {
    std::vector<moveit_msgs::msg::CollisionObject> co_msgs;
    int l_co = py::len(co_strs);
    for (int i = 0; i < l_co; ++i)
    {
      const py::bytes co_str = (co_strs[i]).cast<py::bytes>();
      moveit_msgs::msg::CollisionObject co_msg;
      py_bindings_tools::deserializeMsg(co_str, co_msg);
      co_msgs.push_back(co_msg);
    }
    std::vector<moveit_msgs::msg::ObjectColor> oc_msgs;
    int l_oc = py::len(oc_strs);
    for (int i = 0; i < l_oc; ++i)
    {
      const py::bytes oc_str = (oc_strs[i]).cast<py::bytes>();
      moveit_msgs::msg::ObjectColor oc_msg;
      py_bindings_tools::deserializeMsg(oc_str, oc_msg);
      oc_msgs.push_back(oc_msg);
    }
    addCollisionObjects(co_msgs, oc_msgs);
  }

  void removeCollisionObjectsPython(const py::list& object_ids)
  {
    removeCollisionObjects(py_bindings_tools::stringFromList(object_ids));
  }
};

}  // namespace planning_interface
}  // namespace moveit

using namespace moveit::planning_interface;

PYBIND11_MODULE(planning_scene_interface, m)
{
  m.doc() = "MOVEIT2 planning_scene interface.";
  py::class_<PlanningSceneInterfacePython>(m, "PlanningSceneInterface")
      .def(py::init<std::string>(), py::arg("ns") = "")
      .def("get_known_object_names", &PlanningSceneInterfacePython::getKnownObjectNamesPython,
           py::arg("with_type") = false)
      .def("get_known_object_names_in_roi", &PlanningSceneInterfacePython::getKnownObjectNamesInROIPython)
      .def("get_object_poses", &PlanningSceneInterfacePython::getObjectPosesPython)
      .def("get_objects", &PlanningSceneInterfacePython::getObjectsPython)
      .def("get_attached_objects", &PlanningSceneInterfacePython::getAttachedObjectsPython)
      .def("apply_collision_object", &PlanningSceneInterfacePython::applyCollisionObjectPython)
      .def("apply_collision_object", &PlanningSceneInterfacePython::applyCollisionObjectWithColorPython)
      .def("apply_collision_objects", &PlanningSceneInterfacePython::applyCollisionObjectsWithColorPython)
      .def("apply_attached_collision_object", &PlanningSceneInterfacePython::applyAttachedCollisionObjectPython)
      .def("apply_attached_collision_objects", &PlanningSceneInterfacePython::applyAttachedCollisionObjectsPython)
      .def("apply_planning_scene", &PlanningSceneInterfacePython::applyPlanningScenePython)
      .def("add_collision_objects", &PlanningSceneInterfacePython::addCollisionObjectsPython)
      .def("remove_collision_objects", &PlanningSceneInterfacePython::removeCollisionObjectsPython);
}

/** @endcond */
