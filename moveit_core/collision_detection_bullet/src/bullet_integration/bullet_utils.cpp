/*********************************************************************
 * Software License Agreement (BSD-2-Clause)
 *
 * Copyright (c) 2017, Southwest Research Institute
 * Copyright (c) 2013, John Schulman
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: John Schulman, Levi Armstrong */

#include "moveit/collision_detection_bullet/bullet_integration/bullet_utils.h"

#include <BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <geometric_shapes/shapes.h>
#include <memory>
#include <octomap/octomap.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

static const rclcpp::Logger BULLET_LOGGER = rclcpp::get_logger("collision_detection.bullet");

namespace collision_detection_bullet
{
bool acmCheck(const std::string& body_1, const std::string& body_2,
              const collision_detection::AllowedCollisionMatrix* acm)
{
  collision_detection::AllowedCollision::Type allowed_type;

  if (acm != nullptr)
  {
    if (acm->getAllowedCollision(body_1, body_2, allowed_type))
    {
      if (allowed_type == collision_detection::AllowedCollision::Type::NEVER)
      {
        RCLCPP_DEBUG_STREAM(BULLET_LOGGER,
                            "Not allowed entry in ACM found, collision check between " << body_1 << " and " << body_2);
        return false;
      }
      else
      {
        RCLCPP_DEBUG_STREAM(BULLET_LOGGER,
                            "Entry in ACM found, skipping collision check as allowed " << body_1 << " and " << body_2);
        return true;
      }
    }
    else
    {
      RCLCPP_DEBUG_STREAM(BULLET_LOGGER,
                          "No entry in ACM found, collision check between " << body_1 << " and " << body_2);
      return false;
    }
  }
  else
  {
    RCLCPP_DEBUG_STREAM(BULLET_LOGGER, "No ACM, collision check between " << body_1 << " and " << body_2);
    return false;
  }
}

btCollisionShape* createShapePrimitive(const shapes::Box* geom, const CollisionObjectType& collision_object_type)
{
  (void)(collision_object_type);
  assert(collision_object_type == CollisionObjectType::USE_SHAPE_TYPE);
  const double* size = geom->size;
  btScalar a = static_cast<btScalar>(size[0] / 2);
  btScalar b = static_cast<btScalar>(size[1] / 2);
  btScalar c = static_cast<btScalar>(size[2] / 2);

  return (new btBoxShape(btVector3(a, b, c)));
}

btCollisionShape* createShapePrimitive(const shapes::Sphere* geom, const CollisionObjectType& collision_object_type)
{
  (void)(collision_object_type);
  assert(collision_object_type == CollisionObjectType::USE_SHAPE_TYPE);
  return (new btSphereShape(static_cast<btScalar>(geom->radius)));
}

btCollisionShape* createShapePrimitive(const shapes::Cylinder* geom, const CollisionObjectType& collision_object_type)
{
  (void)(collision_object_type);
  assert(collision_object_type == CollisionObjectType::USE_SHAPE_TYPE);
  btScalar r = static_cast<btScalar>(geom->radius);
  btScalar l = static_cast<btScalar>(geom->length / 2);
  return (new btCylinderShapeZ(btVector3(r, r, l)));
}

btCollisionShape* createShapePrimitive(const shapes::Cone* geom, const CollisionObjectType& collision_object_type)
{
  (void)(collision_object_type);
  assert(collision_object_type == CollisionObjectType::USE_SHAPE_TYPE);
  btScalar r = static_cast<btScalar>(geom->radius);
  btScalar l = static_cast<btScalar>(geom->length);
  return (new btConeShapeZ(r, l));
}

btCollisionShape* createShapePrimitive(const shapes::Mesh* geom, const CollisionObjectType& collision_object_type,
                                       CollisionObjectWrapper* cow)
{
  assert(collision_object_type == CollisionObjectType::USE_SHAPE_TYPE ||
         collision_object_type == CollisionObjectType::CONVEX_HULL ||
         collision_object_type == CollisionObjectType::SDF);

  if (geom->vertex_count > 0 && geom->triangle_count > 0)
  {
    // convert the mesh to the assigned collision object type
    switch (collision_object_type)
    {
      case CollisionObjectType::CONVEX_HULL:
      {
        // Create a convex hull shape to approximate Trimesh
        collision_detection_bullet::AlignedVector<Eigen::Vector3d> input;
        collision_detection_bullet::AlignedVector<Eigen::Vector3d> vertices;
        std::vector<int> faces;

        input.reserve(geom->vertex_count);
        for (unsigned int i = 0; i < geom->vertex_count; ++i)
          input.push_back(Eigen::Vector3d(geom->vertices[3 * i], geom->vertices[3 * i + 1], geom->vertices[3 * i + 2]));

        if (collision_detection_bullet::createConvexHull(vertices, faces, input) < 0)
          return nullptr;

        btConvexHullShape* subshape = new btConvexHullShape();
        for (const Eigen::Vector3d& v : vertices)
          subshape->addPoint(
              btVector3(static_cast<btScalar>(v[0]), static_cast<btScalar>(v[1]), static_cast<btScalar>(v[2])));

        return subshape;
      }
      case CollisionObjectType::USE_SHAPE_TYPE:
      {
        btCompoundShape* compound =
            new btCompoundShape(BULLET_COMPOUND_USE_DYNAMIC_AABB, static_cast<int>(geom->triangle_count));
        compound->setMargin(
            BULLET_MARGIN);  // margin: compound seems to have no effect when positive but has an effect when negative

        for (unsigned i = 0; i < geom->triangle_count; ++i)
        {
          btVector3 v[3];
          for (unsigned x = 0; x < 3; ++x)
          {
            unsigned idx = geom->triangles[3 * i + x];
            for (unsigned y = 0; y < 3; ++y)
            {
              v[x][y] = static_cast<btScalar>(geom->vertices[3 * idx + y]);
            }
          }

          btCollisionShape* subshape = new btTriangleShapeEx(v[0], v[1], v[2]);
          if (subshape != nullptr)
          {
            cow->manage(subshape);
            subshape->setMargin(BULLET_MARGIN);
            btTransform geom_trans;
            geom_trans.setIdentity();
            compound->addChildShape(geom_trans, subshape);
          }
        }

        return compound;
      }
      default:
      {
        RCLCPP_ERROR(BULLET_LOGGER, "This bullet shape type (%d) is not supported for geometry meshs",
                     static_cast<int>(collision_object_type));
        return nullptr;
      }
    }
  }
  RCLCPP_ERROR(BULLET_LOGGER, "The mesh is empty!");
  return nullptr;
}

btCollisionShape* createShapePrimitive(const shapes::OcTree* geom, const CollisionObjectType& collision_object_type,
                                       CollisionObjectWrapper* cow)
{
  assert(collision_object_type == CollisionObjectType::USE_SHAPE_TYPE ||
         collision_object_type == CollisionObjectType::CONVEX_HULL ||
         collision_object_type == CollisionObjectType::SDF ||
         collision_object_type == CollisionObjectType::MULTI_SPHERE);

  btCompoundShape* subshape =
      new btCompoundShape(BULLET_COMPOUND_USE_DYNAMIC_AABB, static_cast<int>(geom->octree->size()));
  double occupancy_threshold = geom->octree->getOccupancyThres();

  // convert the mesh to the assigned collision object type
  switch (collision_object_type)
  {
    case CollisionObjectType::USE_SHAPE_TYPE:
    {
      for (auto it = geom->octree->begin(static_cast<unsigned char>(geom->octree->getTreeDepth())),
                end = geom->octree->end();
           it != end; ++it)
      {
        if (it->getOccupancy() >= occupancy_threshold)
        {
          double size = it.getSize();
          btTransform geom_trans;
          geom_trans.setIdentity();
          geom_trans.setOrigin(btVector3(static_cast<btScalar>(it.getX()), static_cast<btScalar>(it.getY()),
                                         static_cast<btScalar>(it.getZ())));
          btScalar l = static_cast<btScalar>(size / 2);
          btBoxShape* childshape = new btBoxShape(btVector3(l, l, l));
          childshape->setMargin(BULLET_MARGIN);
          cow->manage(childshape);

          subshape->addChildShape(geom_trans, childshape);
        }
      }
      return subshape;
    }
    case CollisionObjectType::MULTI_SPHERE:
    {
      for (auto it = geom->octree->begin(static_cast<unsigned char>(geom->octree->getTreeDepth())),
                end = geom->octree->end();
           it != end; ++it)
      {
        if (it->getOccupancy() >= occupancy_threshold)
        {
          double size = it.getSize();
          btTransform geom_trans;
          geom_trans.setIdentity();
          geom_trans.setOrigin(btVector3(static_cast<btScalar>(it.getX()), static_cast<btScalar>(it.getY()),
                                         static_cast<btScalar>(it.getZ())));
          btSphereShape* childshape =
              new btSphereShape(static_cast<btScalar>(std::sqrt(2 * ((size / 2) * (size / 2)))));
          childshape->setMargin(BULLET_MARGIN);
          cow->manage(childshape);

          subshape->addChildShape(geom_trans, childshape);
        }
      }
      return subshape;
    }
    default:
    {
      RCLCPP_ERROR(BULLET_LOGGER, "This bullet shape type (%d) is not supported for geometry octree",
                   static_cast<int>(collision_object_type));
      return nullptr;
    }
  }
}

void updateCollisionObjectFilters(const std::vector<std::string>& active, CollisionObjectWrapper& cow)
{
  // if not active make cow part of static
  if (!isLinkActive(active, cow.getName()))
  {
    cow.m_collisionFilterGroup = btBroadphaseProxy::StaticFilter;
    cow.m_collisionFilterMask = btBroadphaseProxy::KinematicFilter;
  }
  else
  {
    cow.m_collisionFilterGroup = btBroadphaseProxy::KinematicFilter;
    cow.m_collisionFilterMask = btBroadphaseProxy::KinematicFilter | btBroadphaseProxy::StaticFilter;
  }

  if (cow.getBroadphaseHandle())
  {
    cow.getBroadphaseHandle()->m_collisionFilterGroup = cow.m_collisionFilterGroup;
    cow.getBroadphaseHandle()->m_collisionFilterMask = cow.m_collisionFilterMask;
  }
  RCLCPP_DEBUG_STREAM(BULLET_LOGGER, "COW " << cow.getName() << " group " << cow.m_collisionFilterGroup << " mask "
                                            << cow.m_collisionFilterMask);
}

CollisionObjectWrapperPtr makeCastCollisionObject(const CollisionObjectWrapperPtr& cow)
{
  CollisionObjectWrapperPtr new_cow = cow->clone();

  btTransform tf;
  tf.setIdentity();

  if (btBroadphaseProxy::isConvex(new_cow->getCollisionShape()->getShapeType()))
  {
    assert(dynamic_cast<btConvexShape*>(new_cow->getCollisionShape()) != nullptr);
    btConvexShape* convex = static_cast<btConvexShape*>(new_cow->getCollisionShape());

    // This checks if the collision object is already a cast collision object
    assert(convex->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE);

    CastHullShape* shape = new CastHullShape(convex, tf);

    new_cow->manage(shape);
    new_cow->setCollisionShape(shape);
  }
  else if (btBroadphaseProxy::isCompound(new_cow->getCollisionShape()->getShapeType()))
  {
    btCompoundShape* compound = static_cast<btCompoundShape*>(new_cow->getCollisionShape());
    btCompoundShape* new_compound =
        new btCompoundShape(BULLET_COMPOUND_USE_DYNAMIC_AABB, compound->getNumChildShapes());

    for (int i = 0; i < compound->getNumChildShapes(); ++i)
    {
      if (btBroadphaseProxy::isConvex(compound->getChildShape(i)->getShapeType()))
      {
        btConvexShape* convex = static_cast<btConvexShape*>(compound->getChildShape(i));
        assert(convex->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE);  // This checks if already a cast collision object

        btTransform geom_trans = compound->getChildTransform(i);

        btCollisionShape* subshape = new CastHullShape(convex, tf);

        new_cow->manage(subshape);
        subshape->setMargin(BULLET_MARGIN);
        new_compound->addChildShape(geom_trans, subshape);
      }
      else if (btBroadphaseProxy::isCompound(compound->getChildShape(i)->getShapeType()))
      {
        btCompoundShape* second_compound = static_cast<btCompoundShape*>(compound->getChildShape(i));
        btCompoundShape* new_second_compound =
            new btCompoundShape(BULLET_COMPOUND_USE_DYNAMIC_AABB, second_compound->getNumChildShapes());
        for (int j = 0; j < second_compound->getNumChildShapes(); ++j)
        {
          assert(!btBroadphaseProxy::isCompound(second_compound->getChildShape(j)->getShapeType()));

          btConvexShape* convex = static_cast<btConvexShape*>(second_compound->getChildShape(j));
          assert(convex->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE);  // This checks if already a cast collision object

          btTransform geom_trans = second_compound->getChildTransform(j);

          btCollisionShape* subshape = new CastHullShape(convex, tf);

          new_cow->manage(subshape);
          subshape->setMargin(BULLET_MARGIN);
          new_second_compound->addChildShape(geom_trans, subshape);
        }

        btTransform geom_trans = compound->getChildTransform(i);

        new_cow->manage(new_second_compound);

        // margin on compound seems to have no effect when positive but has an effect when negative
        new_second_compound->setMargin(BULLET_MARGIN);
        new_compound->addChildShape(geom_trans, new_second_compound);
      }
      else
      {
        RCLCPP_ERROR_STREAM(BULLET_LOGGER,
                            "I can only collision check convex shapes and compound shapes made of convex shapes");
        throw std::runtime_error("I can only collision check convex shapes and compound shapes made of convex shapes");
      }
    }

    // margin on compound seems to have no effect when positive but has an effect when negative
    new_compound->setMargin(BULLET_MARGIN);
    new_cow->manage(new_compound);
    new_cow->setCollisionShape(new_compound);
    new_cow->setWorldTransform(cow->getWorldTransform());
  }
  else
  {
    RCLCPP_ERROR_STREAM(BULLET_LOGGER,
                        "I can only collision check convex shapes and compound shapes made of convex shapes");
    throw std::runtime_error("I can only collision check convex shapes and compound shapes made of convex shapes");
  }

  return new_cow;
}

void addCollisionObjectToBroadphase(const CollisionObjectWrapperPtr& cow,
                                    const std::unique_ptr<btBroadphaseInterface>& broadphase,
                                    const std::unique_ptr<btCollisionDispatcher>& dispatcher)
{
  RCLCPP_DEBUG_STREAM(BULLET_LOGGER, "Added " << cow->getName() << " to broadphase");
  btVector3 aabb_min, aabb_max;
  cow->getAABB(aabb_min, aabb_max);

  int type = cow->getCollisionShape()->getShapeType();
  cow->setBroadphaseHandle(broadphase->createProxy(aabb_min, aabb_max, type, cow.get(), cow->m_collisionFilterGroup,
                                                   cow->m_collisionFilterMask, dispatcher.get()));
}

btCollisionShape* createShapePrimitive(const shapes::ShapeConstPtr& geom,
                                       const CollisionObjectType& collision_object_type, CollisionObjectWrapper* cow)
{
  switch (geom->type)
  {
    case shapes::BOX:
    {
      return createShapePrimitive(static_cast<const shapes::Box*>(geom.get()), collision_object_type);
    }
    case shapes::SPHERE:
    {
      return createShapePrimitive(static_cast<const shapes::Sphere*>(geom.get()), collision_object_type);
    }
    case shapes::CYLINDER:
    {
      return createShapePrimitive(static_cast<const shapes::Cylinder*>(geom.get()), collision_object_type);
    }
    case shapes::CONE:
    {
      return createShapePrimitive(static_cast<const shapes::Cone*>(geom.get()), collision_object_type);
    }
    case shapes::MESH:
    {
      return createShapePrimitive(static_cast<const shapes::Mesh*>(geom.get()), collision_object_type, cow);
    }
    case shapes::OCTREE:
    {
      return createShapePrimitive(static_cast<const shapes::OcTree*>(geom.get()), collision_object_type, cow);
    }
    default:
    {
      RCLCPP_ERROR(BULLET_LOGGER, "This geometric shape type (%d) is not supported using BULLET yet",
                   static_cast<int>(geom->type));
      return nullptr;
    }
  }
}

bool BroadphaseFilterCallback::needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const
{
  bool cull = !(proxy0->m_collisionFilterMask & proxy1->m_collisionFilterGroup);
  cull = cull || !(proxy1->m_collisionFilterMask & proxy0->m_collisionFilterGroup);

  if (cull)
    return false;

  const CollisionObjectWrapper* cow0 = static_cast<const CollisionObjectWrapper*>(proxy0->m_clientObject);
  const CollisionObjectWrapper* cow1 = static_cast<const CollisionObjectWrapper*>(proxy1->m_clientObject);

  if (!cow0->m_enabled)
    return false;

  if (!cow1->m_enabled)
    return false;

  if (cow0->getTypeID() == collision_detection::BodyType::ROBOT_ATTACHED &&
      cow1->getTypeID() == collision_detection::BodyType::ROBOT_LINK)
    if (cow0->m_touch_links.find(cow1->getName()) != cow0->m_touch_links.end())
      return false;

  if (cow1->getTypeID() == collision_detection::BodyType::ROBOT_ATTACHED &&
      cow0->getTypeID() == collision_detection::BodyType::ROBOT_LINK)
    if (cow1->m_touch_links.find(cow0->getName()) != cow1->m_touch_links.end())
      return false;

  if (cow0->getTypeID() == collision_detection::BodyType::ROBOT_ATTACHED &&
      cow1->getTypeID() == collision_detection::BodyType::ROBOT_ATTACHED)
    if (cow0->m_touch_links == cow1->m_touch_links)
      return false;

  RCLCPP_DEBUG_STREAM(BULLET_LOGGER, "Broadphase pass " << cow0->getName() << " vs " << cow1->getName());
  return true;
}

btScalar BroadphaseContactResultCallback::addSingleResult(btManifoldPoint& cp,
                                                          const btCollisionObjectWrapper* colObj0Wrap, int /*partId0*/,
                                                          int index0, const btCollisionObjectWrapper* colObj1Wrap,
                                                          int /*partId1*/, int index1)
{
  if (cp.m_distance1 > static_cast<btScalar>(contact_distance_))
  {
    RCLCPP_DEBUG_STREAM(BULLET_LOGGER, "Not close enough for collision with " << cp.m_distance1);
    return 0;
  }

  if (cast_)
  {
    return addCastSingleResult(cp, colObj0Wrap, index0, colObj1Wrap, index1, collisions_);
  }
  else
  {
    return addDiscreteSingleResult(cp, colObj0Wrap, colObj1Wrap, collisions_);
  }
}

bool TesseractCollisionPairCallback::processOverlap(btBroadphasePair& pair)
{
  results_callback_.collisions_.pair_done = false;

  if (results_callback_.collisions_.done)
  {
    return false;
  }

  const CollisionObjectWrapper* cow0 = static_cast<const CollisionObjectWrapper*>(pair.m_pProxy0->m_clientObject);
  const CollisionObjectWrapper* cow1 = static_cast<const CollisionObjectWrapper*>(pair.m_pProxy1->m_clientObject);

  std::pair<std::string, std::string> pair_names{ cow0->getName(), cow1->getName() };
  if (results_callback_.needsCollision(cow0, cow1))
  {
    RCLCPP_DEBUG_STREAM(BULLET_LOGGER, "Processing " << cow0->getName() << " vs " << cow1->getName());
    btCollisionObjectWrapper obj0_wrap(nullptr, cow0->getCollisionShape(), cow0, cow0->getWorldTransform(), -1, -1);
    btCollisionObjectWrapper obj1_wrap(nullptr, cow1->getCollisionShape(), cow1, cow1->getWorldTransform(), -1, -1);

    // dispatcher will keep algorithms persistent in the collision pair
    if (!pair.m_algorithm)
    {
      pair.m_algorithm = dispatcher_->findAlgorithm(&obj0_wrap, &obj1_wrap, nullptr, BT_CLOSEST_POINT_ALGORITHMS);
    }

    if (pair.m_algorithm)
    {
      TesseractBroadphaseBridgedManifoldResult contact_point_result(&obj0_wrap, &obj1_wrap, results_callback_);
      contact_point_result.m_closestPointDistanceThreshold = static_cast<btScalar>(results_callback_.contact_distance_);

      // discrete collision detection query
      pair.m_algorithm->processCollision(&obj0_wrap, &obj1_wrap, dispatch_info_, &contact_point_result);
    }
  }
  else
  {
    RCLCPP_DEBUG_STREAM(BULLET_LOGGER, "Not processing " << cow0->getName() << " vs " << cow1->getName());
  }
  return false;
}

CollisionObjectWrapper::CollisionObjectWrapper(const std::string& name, const collision_detection::BodyType& type_id,
                                               const std::vector<shapes::ShapeConstPtr>& shapes,
                                               const AlignedVector<Eigen::Isometry3d>& shape_poses,
                                               const std::vector<CollisionObjectType>& collision_object_types,
                                               bool active)
  : m_name(name)
  , m_type_id(type_id)
  , m_shapes(shapes)
  , m_shape_poses(shape_poses)
  , m_collision_object_types(collision_object_types)
{
  if (shapes.empty() || shape_poses.empty() ||
      (shapes.size() != shape_poses.size() || collision_object_types.empty() ||
       shapes.size() != collision_object_types.size()))
  {
    throw std::exception();
  }

  this->setContactProcessingThreshold(BULLET_DEFAULT_CONTACT_DISTANCE);
  assert(!name.empty());

  if (active)
  {
    m_collisionFilterGroup = btBroadphaseProxy::KinematicFilter;
    m_collisionFilterMask = btBroadphaseProxy::KinematicFilter | btBroadphaseProxy::StaticFilter;
  }
  else
  {
    m_collisionFilterGroup = btBroadphaseProxy::StaticFilter;
    m_collisionFilterMask = btBroadphaseProxy::KinematicFilter;
  }

  if (shapes.size() == 1)
  {
    btCollisionShape* shape = createShapePrimitive(m_shapes[0], collision_object_types[0], this);
    shape->setMargin(BULLET_MARGIN);
    manage(shape);
    setCollisionShape(shape);
    setWorldTransform(convertEigenToBt(m_shape_poses[0]));
  }
  else
  {
    btCompoundShape* compound =
        new btCompoundShape(BULLET_COMPOUND_USE_DYNAMIC_AABB, static_cast<int>(m_shapes.size()));
    manage(compound);
    // margin on compound seems to have no effect when positive but has an effect when negative
    compound->setMargin(BULLET_MARGIN);
    setCollisionShape(compound);

    setWorldTransform(convertEigenToBt(m_shape_poses[0]));
    Eigen::Isometry3d inv_world = m_shape_poses[0].inverse();

    for (std::size_t j = 0; j < m_shapes.size(); ++j)
    {
      btCollisionShape* subshape = createShapePrimitive(m_shapes[j], collision_object_types[j], this);
      if (subshape != nullptr)
      {
        manage(subshape);
        subshape->setMargin(BULLET_MARGIN);
        btTransform geom_trans = convertEigenToBt(inv_world * m_shape_poses[j]);
        compound->addChildShape(geom_trans, subshape);
      }
    }
  }
}

CollisionObjectWrapper::CollisionObjectWrapper(const std::string& name, const collision_detection::BodyType& type_id,
                                               const std::vector<shapes::ShapeConstPtr>& shapes,
                                               const AlignedVector<Eigen::Isometry3d>& shape_poses,
                                               const std::vector<CollisionObjectType>& collision_object_types,
                                               const std::set<std::string>& touch_links)
  : CollisionObjectWrapper(name, type_id, shapes, shape_poses, collision_object_types, true)
{
  m_touch_links = touch_links;
}

CollisionObjectWrapper::CollisionObjectWrapper(const std::string& name, const collision_detection::BodyType& type_id,
                                               const std::vector<shapes::ShapeConstPtr>& shapes,
                                               const AlignedVector<Eigen::Isometry3d>& shape_poses,
                                               const std::vector<CollisionObjectType>& collision_object_types,
                                               const std::vector<std::shared_ptr<void>>& data)
  : m_name(name)
  , m_type_id(type_id)
  , m_shapes(shapes)
  , m_shape_poses(shape_poses)
  , m_collision_object_types(collision_object_types)
  , m_data(data)
{
}
}  // namespace collision_detection_bullet
