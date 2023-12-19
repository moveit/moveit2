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

#include <list>
#include <moveit_py/moveit_py_utils/copy_ros_msg.h>

namespace moveit_py
{
namespace moveit_py_utils
{
// Ros Message Copy Definitions (Note: copying faster than serialize/deserialize)

geometry_msgs::msg::PoseStamped poseStampedToCpp(const py::object& pose_stamped)
{
  // recreate instance in C++ using python object data
  geometry_msgs::msg::PoseStamped pose_stamped_cpp;
  pose_stamped_cpp.header.frame_id = pose_stamped.attr("header").attr("frame_id").cast<std::string>();
  pose_stamped_cpp.pose = poseToCpp(pose_stamped.attr("pose"));
  return pose_stamped_cpp;
}

geometry_msgs::msg::Pose poseToCpp(const py::object& pose)
{
  // recreate instance in C++ using python object data
  geometry_msgs::msg::Pose pose_cpp;
  pose_cpp.orientation.w = pose.attr("orientation").attr("w").cast<double>();
  pose_cpp.orientation.x = pose.attr("orientation").attr("x").cast<double>();
  pose_cpp.orientation.y = pose.attr("orientation").attr("y").cast<double>();
  pose_cpp.orientation.z = pose.attr("orientation").attr("z").cast<double>();
  pose_cpp.position.x = pose.attr("position").attr("x").cast<double>();
  pose_cpp.position.y = pose.attr("position").attr("y").cast<double>();
  pose_cpp.position.z = pose.attr("position").attr("z").cast<double>();

  return pose_cpp;
}

py::object poseToPy(geometry_msgs::msg::Pose pose)
{
  // recreate instance in Python using C++ object data
  py::object pose_py = py::module_::import("geometry_msgs.msg").attr("Pose")();

  pose_py.attr("orientation").attr("w") = pose.orientation.w;
  pose_py.attr("orientation").attr("x") = pose.orientation.x;
  pose_py.attr("orientation").attr("y") = pose.orientation.y;
  pose_py.attr("orientation").attr("z") = pose.orientation.z;
  pose_py.attr("position").attr("x") = pose.position.x;
  pose_py.attr("position").attr("y") = pose.position.y;
  pose_py.attr("position").attr("z") = pose.position.z;

  return pose_py;
}

geometry_msgs::msg::Point pointToCpp(const py::object& point)
{
  // recreate instance in C++ using python object data
  geometry_msgs::msg::Point point_cpp;
  point_cpp.x = point.attr("x").cast<double>();
  point_cpp.y = point.attr("y").cast<double>();
  point_cpp.z = point.attr("z").cast<double>();

  return point_cpp;
}

geometry_msgs::msg::Vector3 vector3ToCpp(const py::object& vector3)
{
  // recreate instance in C++ using python object data
  geometry_msgs::msg::Vector3 vector3_cpp;
  vector3_cpp.x = vector3.attr("x").cast<double>();
  vector3_cpp.y = vector3.attr("y").cast<double>();
  vector3_cpp.z = vector3.attr("z").cast<double>();

  return vector3_cpp;
}

geometry_msgs::msg::Quaternion quaternionToCpp(const py::object& quaternion)
{
  // recreate instance in C++ using python object data
  geometry_msgs::msg::Quaternion quaternion_cpp;
  quaternion_cpp.w = quaternion.attr("w").cast<double>();
  quaternion_cpp.x = quaternion.attr("x").cast<double>();
  quaternion_cpp.y = quaternion.attr("y").cast<double>();
  quaternion_cpp.z = quaternion.attr("z").cast<double>();

  return quaternion_cpp;
}

shape_msgs::msg::SolidPrimitive solidPrimitiveToCpp(const py::object& primitive)
{
  // recreate instance in C++ using python object data
  shape_msgs::msg::SolidPrimitive primitive_cpp;
  primitive_cpp.type = primitive.attr("type").cast<int>();
  for (auto& dimension : primitive.attr("dimensions"))
  {
    primitive_cpp.dimensions.push_back(py::reinterpret_borrow<py::object>(dimension).cast<double>());
  }

  return primitive_cpp;
}

shape_msgs::msg::MeshTriangle meshTriangleToCpp(const py::object& mesh_triangle)
{
  // recreate instance in C++ using python object data
  shape_msgs::msg::MeshTriangle mesh_triangle_cpp;
  mesh_triangle_cpp.vertex_indices[0] = mesh_triangle.attr("vertex_indices").attr("__getitem__")(0).cast<int>();
  mesh_triangle_cpp.vertex_indices[1] = mesh_triangle.attr("vertex_indices").attr("__getitem__")(1).cast<int>();
  mesh_triangle_cpp.vertex_indices[2] = mesh_triangle.attr("vertex_indices").attr("__getitem__")(2).cast<int>();

  return mesh_triangle_cpp;
}

shape_msgs::msg::Mesh meshToCpp(const py::object& mesh)
{
  // recreate instance in C++ using python object data
  shape_msgs::msg::Mesh mesh_cpp;
  mesh_cpp.vertices.resize(mesh.attr("vertices").attr("__len__")().cast<int>());
  for (const auto& vertex : mesh.attr("vertices"))
  {
    mesh_cpp.vertices.push_back(pointToCpp(py::reinterpret_borrow<py::object>(vertex)));
  }
  mesh_cpp.triangles.resize(mesh.attr("triangles").attr("__len__")().cast<int>());
  for (const auto& triangle : mesh.attr("triangles"))
  {
    mesh_cpp.triangles.push_back(meshTriangleToCpp(py::reinterpret_borrow<py::object>(triangle)));
  }

  return mesh_cpp;
}

moveit_msgs::msg::BoundingVolume boundingVolumeToCpp(const py::object& bounding_volume)
{
  // recreate instance in C++ using python object data
  moveit_msgs::msg::BoundingVolume bounding_volume_cpp;

  // primitives
  for (const auto& primitive : bounding_volume.attr("primitives"))
  {
    bounding_volume_cpp.primitives.push_back(solidPrimitiveToCpp(py::reinterpret_borrow<py::object>(primitive)));
  }

  // primitive poses
  for (const auto& primitive_pose : bounding_volume.attr("primitive_poses"))
  {
    bounding_volume_cpp.primitive_poses.push_back(poseToCpp(py::reinterpret_borrow<py::object>(primitive_pose)));
  }

  // meshes
  for (const auto& mesh : bounding_volume.attr("meshes"))
  {
    bounding_volume_cpp.meshes.push_back(meshToCpp(py::reinterpret_borrow<py::object>(mesh)));
  }

  // mesh poses
  for (const auto& mesh_poses : bounding_volume.attr("mesh_poses"))
  {
    bounding_volume_cpp.mesh_poses.push_back(poseToCpp(py::reinterpret_borrow<py::object>(mesh_poses)));
  }

  return bounding_volume_cpp;
}

moveit_msgs::msg::JointConstraint jointConstraintToCpp(const py::object& joint_constraint)
{
  // recreate instance in C++ using python object data
  moveit_msgs::msg::JointConstraint joint_constraint_cpp;
  joint_constraint_cpp.joint_name = joint_constraint.attr("joint_name").cast<std::string>();
  joint_constraint_cpp.position = joint_constraint.attr("position").cast<double>();
  joint_constraint_cpp.tolerance_above = joint_constraint.attr("tolerance_above").cast<double>();
  joint_constraint_cpp.tolerance_below = joint_constraint.attr("tolerance_below").cast<double>();
  joint_constraint_cpp.weight = joint_constraint.attr("weight").cast<double>();

  return joint_constraint_cpp;
}

moveit_msgs::msg::PositionConstraint positionConstraintToCpp(const py::object& position_constraint)
{
  // recreate instance in C++ using python object data
  moveit_msgs::msg::PositionConstraint position_constraint_cpp;
  position_constraint_cpp.header.frame_id = position_constraint.attr("header").attr("frame_id").cast<std::string>();
  position_constraint_cpp.link_name = position_constraint.attr("link_name").cast<std::string>();
  position_constraint_cpp.target_point_offset = vector3ToCpp(position_constraint.attr("target_point_offset"));
  position_constraint_cpp.constraint_region = boundingVolumeToCpp(position_constraint.attr("constraint_region"));
  position_constraint_cpp.weight = position_constraint.attr("weight").cast<double>();

  return position_constraint_cpp;
}

moveit_msgs::msg::OrientationConstraint orientationConstraintToCpp(const py::object& orientation_constraint)
{
  // recreate instance in C++ using python object data
  moveit_msgs::msg::OrientationConstraint orientation_constraint_cpp;
  orientation_constraint_cpp.header.frame_id =
      orientation_constraint.attr("header").attr("frame_id").cast<std::string>();
  orientation_constraint_cpp.link_name = orientation_constraint.attr("link_name").cast<std::string>();
  orientation_constraint_cpp.orientation = quaternionToCpp(orientation_constraint.attr("target_quaternion"));
  orientation_constraint_cpp.absolute_x_axis_tolerance =
      orientation_constraint.attr("absolute_x_axis_tolerance").cast<double>();
  orientation_constraint_cpp.absolute_y_axis_tolerance =
      orientation_constraint.attr("absolute_y_axis_tolerance").cast<double>();
  orientation_constraint_cpp.absolute_z_axis_tolerance =
      orientation_constraint.attr("absolute_z_axis_tolerance").cast<double>();
  orientation_constraint_cpp.parameterization = orientation_constraint.attr("parameterization").cast<int>();
  orientation_constraint_cpp.weight = orientation_constraint.attr("weight").cast<double>();

  return orientation_constraint_cpp;
}

moveit_msgs::msg::VisibilityConstraint visibilityConstraintToCpp(const py::object& visibility_constraint)
{
  // recreate instance in C++ using python object data
  moveit_msgs::msg::VisibilityConstraint visibility_constraint_cpp;
  visibility_constraint_cpp.target_radius = visibility_constraint.attr("target_radius").cast<double>();
  visibility_constraint_cpp.target_pose = poseStampedToCpp(visibility_constraint.attr("target_pose"));
  visibility_constraint_cpp.cone_sides = visibility_constraint.attr("cone_sides").cast<int>();
  visibility_constraint_cpp.sensor_pose = poseStampedToCpp(visibility_constraint.attr("sensor_pose"));
  visibility_constraint_cpp.max_view_angle = visibility_constraint.attr("max_view_angle").cast<double>();
  visibility_constraint_cpp.max_range_angle = visibility_constraint.attr("max_range_angle").cast<double>();
  visibility_constraint_cpp.sensor_view_direction = visibility_constraint.attr("sensor_view_direction").cast<int>();
  visibility_constraint_cpp.weight = visibility_constraint.attr("weight").cast<double>();

  return visibility_constraint_cpp;
}

moveit_msgs::msg::CollisionObject collisionObjectToCpp(const py::object& collision_object)
{
  //  recreate instance in C++ using python object data
  moveit_msgs::msg::CollisionObject collision_object_cpp;

  // header
  collision_object_cpp.header.frame_id = collision_object.attr("header").attr("frame_id").cast<std::string>();

  // object pose
  collision_object_cpp.pose = poseToCpp(collision_object.attr("pose"));

  // object id
  collision_object_cpp.id = collision_object.attr("id").cast<std::string>();

  // object type
  collision_object_cpp.type.key = collision_object.attr("type").attr("key").cast<std::string>();
  collision_object_cpp.type.db = collision_object.attr("type").attr("db").cast<std::string>();

  // iterate through python list creating C++ vector of primitives
  for (const auto& primitive : collision_object.attr("primitives"))
  {
    auto primitive_cpp = solidPrimitiveToCpp(py::reinterpret_borrow<py::object>(primitive));
    collision_object_cpp.primitives.push_back(primitive_cpp);
  }

  // iterate through python list creating C++ vector of primitive poses
  for (const auto& primitive_pose : collision_object.attr("primitive_poses"))
  {
    auto primitive_pose_cpp = poseToCpp(py::reinterpret_borrow<py::object>(primitive_pose));
    collision_object_cpp.primitive_poses.push_back(primitive_pose_cpp);
  }

  // iterate through python list creating C++ vector of meshes
  for (const auto& mesh : collision_object.attr("meshes"))
  {
    // TODO (peterdavidfagan):  implement mesh conversion
    auto mesh_cpp = meshToCpp(py::reinterpret_borrow<py::object>(mesh));
    collision_object_cpp.meshes.push_back(mesh_cpp);
  }

  // iterate through python list creating C++ vector of mesh poses
  for (const auto& mesh_pose : collision_object.attr("mesh_poses"))
  {
    auto mesh_pose_cpp = poseToCpp(py::reinterpret_borrow<py::object>(mesh_pose));
    collision_object_cpp.mesh_poses.push_back(mesh_pose_cpp);
  }

  // operation
  collision_object_cpp.operation = collision_object.attr("operation").cast<char>();

  return collision_object_cpp;
}

moveit_msgs::msg::Constraints constraintsToCpp(const py::object& constraints)
{
  // recreate instance in C++ using python object data
  moveit_msgs::msg::Constraints constraints_cpp;

  // iterate through python list creating C++ vector of joint constraints
  for (const auto& joint_constraint : constraints.attr("joint_constraints"))
  {
    auto joint_constraint_cpp = jointConstraintToCpp(py::reinterpret_borrow<py::object>(joint_constraint));
    constraints_cpp.joint_constraints.push_back(joint_constraint_cpp);
  }

  // iterate through python list creating C++ vector of position constraints
  for (const auto& position_constraint : constraints.attr("position_constraints"))
  {
    auto position_constraint_cpp = positionConstraintToCpp(py::reinterpret_borrow<py::object>(position_constraint));
    constraints_cpp.position_constraints.push_back(position_constraint_cpp);
  }

  // iterate through python list creating C++ vector of orientation constraints
  for (const auto& orientation_constraint : constraints.attr("orientation_constraints"))
  {
    auto orientation_constraint_cpp =
        orientationConstraintToCpp(py::reinterpret_borrow<py::object>(orientation_constraint));
    constraints_cpp.orientation_constraints.push_back(orientation_constraint_cpp);
  }

  // iterate through python list creating C++ vector of visibility constraints
  for (const auto& visibility_constraint : constraints.attr("visibility_constraints"))
  {
    auto visibility_constraint_cpp =
        visibilityConstraintToCpp(py::reinterpret_borrow<py::object>(visibility_constraint));
    constraints_cpp.visibility_constraints.push_back(visibility_constraint_cpp);
  }

  return constraints_cpp;
}
}  // namespace moveit_py_utils
}  // namespace moveit_py
