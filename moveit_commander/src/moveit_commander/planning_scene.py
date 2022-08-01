# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Larry Lu, Ioan Sucan, Felix Messmer

import moveit_commander.conversions as conversions

from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from moveit_ros_planning_interface import planning_scene_interface
from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
from moveit_commander.exception import MoveItCommanderException

try:
    from pyassimp import pyassimp
except:
    # support pyassimp > 3.0
    try:
        import pyassimp
    except:
        pyassimp = False
        print(
            "Failed to import pyassimp, see https://github.com/ros-planning/moveit/issues/86 for more info"
        )


class PlanningSceneInterface(object):
    """
    Python interface for a C++ PlanningSceneInterface.
    Uses both C++ wrapped methods and scene manipulation topics
    to manipulate the PlanningScene managed by the PlanningSceneMonitor.
    See python_planning_scene_interface.cpp for the wrapped methods.
    """

    def __init__(self, ns="", synchronous=False):
        """Create a planning scene interface; it uses both C++ wrapped methods and scene manipulation topics."""
        self._psi = planning_scene_interface.PlanningSceneInterface(
            ns, synchronous)
        self.__synchronous = synchronous
        

    def __submit(self, collision_object, attach=False):
        co_str = conversions.serialize(collision_object)
        if attach:
            self._psi.apply_attached_collision_object(
                co_str, self.__synchronous)
        else:
            self._psi.apply_collision_object(co_str, self.__synchronous)


    def add_object(self, collision_object):
        """Add an object to the planning scene"""
        self.__submit(collision_object, attach=False)

    def add_sphere(self, name, pose, radius=1):
        """Add a sphere to the planning scene"""
        co = self.__make_sphere(name, pose, radius)
        self.__submit(co, attach=False)

    def add_cylinder(self, name, pose, height, radius):
        """Add a cylinder to the planning scene"""
        co = self.__make_cylinder(name, pose, height, radius)
        self.__submit(co, attach=False)

    def add_mesh(self, name, pose, filename, size=(1, 1, 1)):
        """Add a mesh to the planning scene"""
        co = self.__make_mesh(name, pose, filename, size)
        self.__submit(co, attach=False)

    def add_box(self, name, pose, size=(1, 1, 1)):
        """Add a box to the planning scene"""
        co = self.__make_box(name, pose, size)
        self.__submit(co, attach=False)

    def add_plane(self, name, pose, normal=(0.0, 0.0, 1.0), offset=0.0):
        """Add a plane to the planning scene"""
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header
        p = Plane()
        coef = list(normal)
        coef.append(offset)
        p.coef = coef
        co.planes = [p]
        co.plane_poses = [pose.pose]
        self.__submit(co, attach=False)

    def attach_object(self, attached_collision_object):
        """Attach an object in the planning scene"""
        self.__submit(attached_collision_object, attach=True)

    def attach_mesh(
        self, link, name, pose=None, filename="", size=(1, 1, 1), touch_links=[]
    ):
        aco = AttachedCollisionObject()
        if (pose is not None) and filename:
            aco.object = self.__make_mesh(name, pose, filename, size)
        else:
            aco.object = self.__make_existing(name)
        aco.link_name = link
        aco.touch_links = [link]
        if len(touch_links) > 0:
            aco.touch_links = touch_links
        self.__submit(aco, attach=True)

    def attach_box(self, link, name, pose=None, size=(1, 1, 1), touch_links=[]):
        aco = AttachedCollisionObject()
        if pose is not None:
            aco.object = self.__make_box(name, pose, size)
        else:
            aco.object = self.__make_existing(name)
        aco.link_name = link
        if len(touch_links) > 0:
            aco.touch_links = touch_links
        else:
            aco.touch_links = [link]
        self.__submit(aco, attach=True)

    def clear(self):
        """Remove all objects from the planning scene"""
        self.remove_attached_object()
        self.remove_world_object()

    def remove_world_object(self, name=None):
        """
        Remove an object from planning scene, or all if no name is provided
        """
        co = CollisionObject()
        co.operation = CollisionObject.REMOVE
        if name is not None:
            co.id = name
        self.__submit(co, attach=False)

    def remove_attached_object(self, link=None, name=None):
        """
        Remove an attached object from the robot, or all objects attached to the link if no name is provided,
        or all attached objects in the scene if neither link nor name are provided.

        Removed attached objects remain in the scene as world objects.
        Call remove_world_object afterwards to remove them from the scene.
        """
        aco = AttachedCollisionObject()
        aco.object.operation = CollisionObject.REMOVE
        if link is not None:
            aco.link_name = link
        if name is not None:
            aco.object.id = name
        self.__submit(aco, attach=True)

    def get_known_object_names(self, with_type=False):
        """
        Get the names of all known objects in the world. If with_type is set to true, only return objects that have a known type.
        """
        return self._psi.get_known_object_names(with_type)

    def get_known_object_names_in_roi(
        self, minx, miny, minz, maxx, maxy, maxz, with_type=False
    ):
        """
        Get the names of known objects in the world that are located within a bounding region (specified in the frame reported by
        get_planning_frame()). If with_type is set to true, only return objects that have a known type.
        """
        return self._psi.get_known_object_names_in_roi(
            minx, miny, minz, maxx, maxy, maxz, with_type
        )

    def get_object_poses(self, object_ids):
        """
        Get the poses from the objects identified by the given object ids list.
        """
        ser_ops = self._psi.get_object_poses(object_ids)
        ops = dict()
        for key in ser_ops:
            msg = conversions.deserialize(ser_ops[key], Pose)
            ops[key] = msg
        return ops

    def get_objects(self, object_ids=[]):
        """
        Get the objects identified by the given object ids list. If no ids are provided, return all the known objects.
        """
        ser_objs = self._psi.get_objects(object_ids)
        objs = dict()
        for key in ser_objs:
            msg = conversions.deserialize(ser_objs[key], CollisionObject)
            objs[key] = msg
        return objs

    def get_attached_objects(self, object_ids=[]):
        """
        Get the attached objects identified by the given object ids list. If no ids are provided, return all the attached objects.
        """
        ser_aobjs = self._psi.get_attached_objects(object_ids)
        aobjs = dict()
        for key in ser_aobjs:
            msg = conversions.deserialize(
                ser_aobjs[key], AttachedCollisionObject)
            aobjs[key] = msg
        return aobjs

    def apply_planning_scene(self, planning_scene_message):
        """
        Applies the planning scene message.
        """
        return self._psi.apply_planning_scene(
            conversions.serialize(planning_scene_message)
        )

    @staticmethod
    def __make_existing(name):
        """
        Create an empty Collision Object. Used when the object already exists
        """
        co = CollisionObject()
        co.id = name
        return co

    @staticmethod
    def __make_box(name, pose, size):
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header
        co.pose = pose.pose
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(size)
        co.primitives = [box]
        return co

    @staticmethod
    def __make_mesh(name, pose, filename, scale=(1, 1, 1)):
        co = CollisionObject()
        if pyassimp is False:
            raise MoveItCommanderException(
                "Pyassimp needs patch https://launchpadlibrarian.net/319496602/patchPyassim.txt"
            )
        scene = pyassimp.load(filename)
        if not scene.meshes or len(scene.meshes) == 0:
            raise MoveItCommanderException("There are no meshes in the file")
        if len(scene.meshes[0].faces) == 0:
            raise MoveItCommanderException("There are no faces in the mesh")
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header
        co.pose = pose.pose

        mesh = Mesh()
        first_face = scene.meshes[0].faces[0]
        if hasattr(first_face, "__len__"):
            for face in scene.meshes[0].faces:
                if len(face) == 3:
                    triangle = MeshTriangle()
                    triangle.vertex_indices = [face[0], face[1], face[2]]
                    mesh.triangles.append(triangle)
        elif hasattr(first_face, "indices"):
            for face in scene.meshes[0].faces:
                if len(face.indices) == 3:
                    triangle = MeshTriangle()
                    triangle.vertex_indices = [
                        face.indices[0],
                        face.indices[1],
                        face.indices[2],
                    ]
                    mesh.triangles.append(triangle)
        else:
            raise MoveItCommanderException(
                "Unable to build triangles from mesh due to mesh object structure"
            )
        for vertex in scene.meshes[0].vertices:
            point = Point()
            point.x = vertex[0] * scale[0]
            point.y = vertex[1] * scale[1]
            point.z = vertex[2] * scale[2]
            mesh.vertices.append(point)
        co.meshes = [mesh]
        pyassimp.release(scene)
        return co

    @staticmethod
    def __make_sphere(name, pose, radius):
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header
        co.pose = pose.pose
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [radius]
        co.primitives = [sphere]
        return co

    @staticmethod
    def __make_cylinder(name, pose, height, radius):
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header
        co.pose = pose.pose
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = [height, radius]
        co.primitives = [cylinder]
        return co
