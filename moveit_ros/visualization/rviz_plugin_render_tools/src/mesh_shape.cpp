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

#include <ogre_helpers/mesh_shape.hpp>

#include <OgreMesh.h>
#include <OgreMeshManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreEntity.h>
#include <OgreMaterialManager.h>
#include <OgreManualObject.h>

#include <rviz_common/logging.hpp>
#include <string>

namespace rviz_rendering
{
MeshShape::MeshShape(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
  : Shape(Shape::Mesh, scene_manager, parent_node), started_(false)
{
  static uint32_t count = 0;
  manual_object_ = scene_manager->createManualObject("MeshShape_ManualObject" + std::to_string(count++));
  material_->setCullingMode(Ogre::CULL_NONE);
}

MeshShape::~MeshShape()
{
  clear();
  scene_manager_->destroyManualObject(manual_object_);
}

void MeshShape::estimateVertexCount(size_t vcount)
{
  if (entity_ == nullptr && !started_)
    manual_object_->estimateVertexCount(vcount);
}

void MeshShape::beginTriangles()
{
  if (!started_ && entity_)
  {
    RVIZ_COMMON_LOG_WARNING("Cannot modify mesh once construction is complete");
    return;
  }

  if (!started_)
  {
    started_ = true;
    manual_object_->begin(material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST, "rviz_rendering");
  }
}

void MeshShape::addVertex(const Ogre::Vector3& position)
{
  beginTriangles();
  manual_object_->position(position);
}

void MeshShape::addVertex(const Ogre::Vector3& position, const Ogre::Vector3& normal)
{
  beginTriangles();
  manual_object_->position(position);
  manual_object_->normal(normal);
}

void MeshShape::addVertex(const Ogre::Vector3& position, const Ogre::Vector3& normal, const Ogre::ColourValue& color)
{
  beginTriangles();
  manual_object_->position(position);
  manual_object_->normal(normal);
  manual_object_->colour(color);
}

void MeshShape::addNormal(const Ogre::Vector3& normal)
{
  manual_object_->normal(normal);
}

void MeshShape::addColor(const Ogre::ColourValue& color)
{
  manual_object_->colour(color);
}

void MeshShape::addTriangle(unsigned int v1, unsigned int v2, unsigned int v3)
{
  manual_object_->triangle(v1, v2, v3);
}

void MeshShape::endTriangles()
{
  if (started_)
  {
    started_ = false;
    manual_object_->end();
    static uint32_t count = 0;
    std::string name = "ConvertedMeshShape@" + std::to_string(count++);
    manual_object_->convertToMesh(name);
    entity_ = scene_manager_->createEntity(name);
    if (entity_)
    {
      entity_->setMaterialName(material_name_, "rviz_rendering");
      offset_node_->attachObject(entity_);
    }
    else
      RVIZ_COMMON_LOG_ERROR("Unable to construct triangle mesh");
  }
  else
    RVIZ_COMMON_LOG_ERROR("No triangles added");
}

void MeshShape::clear()
{
  if (entity_)
  {
    entity_->detachFromParent();
    Ogre::MeshManager::getSingleton().remove(entity_->getMesh()->getName());
    scene_manager_->destroyEntity(entity_);
    entity_ = nullptr;
  }
  manual_object_->clear();
  started_ = false;
}

}  // namespace rviz_rendering
