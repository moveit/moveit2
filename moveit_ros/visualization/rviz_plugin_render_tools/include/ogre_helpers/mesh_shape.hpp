/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <rviz_rendering/objects/shape.hpp>

namespace Ogre
{
class ManualObject;
}

// TODO(JafarAbdi): This's taken from https://github.com/ros2/rviz for MoveIt 2 beta release -- remove when it's ported
namespace rviz_rendering
{
/** \brief This class allows constructing Ogre shapes manually, from triangle lists.

    For example:
    Assuming we have a set of mesh triangles represented like this:
    \verbatim
    struct Triangle
    {
      unsigned v1, v2, v3; // index for the 3 vertices that make up a triangle
    };
    std::vector<Triangle> triangles;
    std::vector<Ogre::Vector3> vertices;
    std::vector<Ogre::Vector3> normals; // normal at every vertex
    \endverbatim

    we can use this class to render the mesh as follows:
    \verbatim
    rviz::MeshShape *shape = new MeshShape(scene_manager);
    mesh->estimateVertexCount(vertices.size());
    mesh->beginTriangles();
    for (std::size_t i = 0 ; i < vertices.size() ; ++i)
      mesh->addVertex(vertices[i], normals[i]);
    for (std::size_t i = 0 ; i < triangles.size() ; ++i)
      mesh->addTriangle(triangles[i].v1, triangles[i].v2, triangles[i].v3);
    mesh->endTriangles();
    \endverbatim
 */
class MeshShape : public Shape
{
public:
  /**
   * \brief Constructor
   *
   * @param scene_manager The scene manager this object is associated with
   * @param parent_node A scene node to use as the parent of this object.  If nullptr, uses the root scene node.
   */
  MeshShape(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node = nullptr);
  ~MeshShape() override;

  /* \brief Estimate the number of vertices ahead of time. */
  void estimateVertexCount(size_t vcount);

  /** \brief Start adding triangles to the mesh */
  void beginTriangles();

  /** \brief Add a vertex to the mesh (no normal defined). If using
      this function only (not using addTriangle()) it is assumed that
      triangles are added by specifying the 3 vertices in order (3
      consecutive calls to this function). This means there must be
      3*n calls to this function to add n triangles. If addTriangle()
      is used, indexing in the defined vertices is done. */
  void addVertex(const Ogre::Vector3& position);

  /** \brief Add a vertex to the mesh with a normal defined. If using
      this function only (not using addTriangle()) it is assumed that
      triangles are added by specifying the 3 vertices in order (3
      consecutive calls to this function). This means there must be
      3*n calls to this function to add n triangles.If addTriangle()
      is used, indexing in the defined vertices is done.  */
  void addVertex(const Ogre::Vector3& position, const Ogre::Vector3& normal);

  /** \brief Add a vertex to the mesh with normal and color defined. If using
      this function only (not using addTriangle()) it is assumed that
      triangles are added by specifying the 3 vertices in order (3
      consecutive calls to this function). This means there must be
      3*n calls to this function to add n triangles.If addTriangle()
      is used, indexing in the defined vertices is done. */
  void addVertex(const Ogre::Vector3& position, const Ogre::Vector3& normal, const Ogre::ColourValue& color);

  /** \brief Add normal for a vertex */
  void addNormal(const Ogre::Vector3& normal);

  /** \brief Add color for a vertex */
  void addColor(const Ogre::ColourValue& color);

  /** \brief Add a triangle by indexing in the defined vertices. */
  void addTriangle(unsigned int p1, unsigned int p2, unsigned int p3);

  /** \brief Notify that the set of triangles to add is complete. No more triangles can be added, beginTriangles() can
   * no longer be called unless clear() was called. */
  void endTriangles();

  /** \brief Clear the mesh */
  void clear();

  /** \brief Get the manual object created for the mesh */
  Ogre::ManualObject* getManualObject()
  {
    return manual_object_;
  }

private:
  // true in between calls to beginTriangles() and endTriangles()
  bool started_;
  Ogre::ManualObject* manual_object_;
};

}  // namespace rviz_rendering
