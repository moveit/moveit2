/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Ioan Sucan, Jia Pan */

#include <moveit/collision_detection_hpp_fcl/collision_common_hpp_fcl.h>
#include <geometric_shapes/shapes.h>

#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/octree.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/distance.h>

#include <memory>
#include <type_traits>
#include <mutex>

namespace collision_detection
{
// Logger
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_collision_detection_hpp_fcl.collision_common");

/** \brief Cache for an arbitrary type of shape. It is assigned during the execution of \e createCollisionGeometry().
 *
 *  Only a single cache per thread and object type is created as it is a quasi-singleton instance. */
struct HPPFCLShapeCache
{
  using ShapeKey = shapes::ShapeConstWeakPtr;
  using ShapeMap = std::map<ShapeKey, HPPFCLGeometryConstPtr, std::owner_less<ShapeKey>>;

  HPPFCLShapeCache() : clean_count_(0)
  {
  }

  void bumpUseCount(bool force = false)
  {
    clean_count_++;

    // clean-up for cache (we don't want to keep infinitely large number of weak ptrs stored)
    if (clean_count_ > MAX_CLEAN_COUNT || force)
    {
      clean_count_ = 0;
      for (auto it = map_.begin(); it != map_.end();)
      {
        auto nit = it;
        ++nit;
        if (it->first.expired())
          map_.erase(it);
        it = nit;
      }
      //      RCLCPP_DEBUG(LOGGER, "Cleaning up cache for FCL objects that correspond to static
      //      shapes. Cache size
      //      reduced from %u
      //      to %u", from, (unsigned int)map_.size());
    }
  }

  static const unsigned int MAX_CLEAN_COUNT = 100;  // every this many uses of the cache, a cleaning operation is
                                                    // executed (this is only removal of expired entries)
  /** \brief Map of weak pointers to the FCLGeometry. */
  ShapeMap map_;

  /** \brief Counts cache usage and triggers clearing of cache when \m MAX_CLEAN_COUNT is exceeded. */
  unsigned int clean_count_;
};

/* Templated function to get a different cache for each of the template arguments combinations.
 *
 * The returned cache is a quasi-singleton for each thread as it is created \e thread_local. */
template <typename BV, typename T>
HPPFCLShapeCache& GetShapeCache()
{
  /* The cache is created thread_local, that is each thread calling
   * this quasi-singleton function will get its own instance. Once
   * the thread joins/exits, the cache gets deleted.
   * Reasoning is that multi-threaded planners (eg OMPL) or user-code
   * will often need to do collision checks with the same object
   * simultaneously (especially true for attached objects). Having only
   * one global cache leads to many cache misses. Also as the cache can
   * only be accessed by one thread we don't need any locking.
   */
  static thread_local HPPFCLShapeCache cache;
  return cache;
}

/** \brief Templated helper function creating new collision geometry out of general object using an arbitrary bounding
 *  volume (BV).
 *
 *  It assigns a thread-local cache for each type of shape and minimizes memory usage and copying through utilizing the
 *  cache. */
template <typename BV, typename T>
HPPFCLGeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr& shape, const T* data, int shape_index)
{
  using ShapeKey = shapes::ShapeConstWeakPtr;
  using ShapeMap = std::map<ShapeKey, HPPFCLGeometryConstPtr, std::owner_less<ShapeKey>>;

  HPPFCLShapeCache& cache = GetShapeCache<BV, T>();

  shapes::ShapeConstWeakPtr wptr(shape);
  {
    ShapeMap::const_iterator cache_it = cache.map_.find(wptr);
    if (cache_it != cache.map_.end())
    {
      if (cache_it->second->collision_geometry_data_->ptr.raw == data)
      {
        //        RCLCPP_DEBUG(LOGGER, "Collision data structures for object %s retrieved from
        //        cache.",
        //        cache_it->second->collision_geometry_data_->getID().c_str());
        return cache_it->second;
      }
      else if (cache_it->second.unique())
      {
        const_cast<HPPFCLGeometry*>(cache_it->second.get())->updateCollisionGeometryData(data, shape_index, false);
        //          RCLCPP_DEBUG(LOGGER, "Collision data structures for object %s retrieved from
        //          cache after updating
        //          the source
        //          object.", cache_it->second->collision_geometry_data_->getID().c_str());
        return cache_it->second;
      }
    }
  }

  // attached objects could have previously been World::Object; we try to move them
  // from their old cache to the new one, if possible. the code is not pretty, but should help
  // when we attach/detach objects that are in the world
  if (std::is_same<T, moveit::core::AttachedBody>::value)
  {
    // get the cache that corresponds to objects; maybe this attached object used to be a world object
    HPPFCLShapeCache& othercache = GetShapeCache<BV, World::Object>();

    // attached bodies could be just moved from the environment.
    auto cache_it = othercache.map_.find(wptr);
    if (cache_it != othercache.map_.end())
    {
      if (cache_it->second.unique())
      {
        // remove from old cache
        HPPFCLGeometryConstPtr obj_cache = cache_it->second;
        othercache.map_.erase(cache_it);
        // update the CollisionGeometryData; nobody has a pointer to this, so we can safely modify it
        const_cast<HPPFCLGeometry*>(obj_cache.get())->updateCollisionGeometryData(data, shape_index, true);

        //        RCLCPP_DEBUG(LOGGER, "Collision data structures for attached body %s retrieved
        //        from the cache for
        //        world objects.",
        //        obj_cache->collision_geometry_data_->getID().c_str());

        // add to the new cache
        cache.map_[wptr] = obj_cache;
        cache.bumpUseCount();
        return obj_cache;
      }
    }
  }
  else
      // world objects could have previously been attached objects; we try to move them
      // from their old cache to the new one, if possible. the code is not pretty, but should help
      // when we attach/detach objects that are in the world
      if (std::is_same<T, World::Object>::value)
  {
    // get the cache that corresponds to objects; maybe this attached object used to be a world object
    HPPFCLShapeCache& othercache = GetShapeCache<BV, moveit::core::AttachedBody>();

    // attached bodies could be just moved from the environment.
    auto cache_it = othercache.map_.find(wptr);
    if (cache_it != othercache.map_.end())
    {
      if (cache_it->second.unique())
      {
        // remove from old cache
        HPPFCLGeometryConstPtr obj_cache = cache_it->second;
        othercache.map_.erase(cache_it);

        // update the CollisionGeometryData; nobody has a pointer to this, so we can safely modify it
        const_cast<HPPFCLGeometry*>(obj_cache.get())->updateCollisionGeometryData(data, shape_index, true);

        //          RCLCPP_DEBUG(LOGGER, "Collision data structures for world object %s retrieved
        //          from the cache for
        //          attached
        //          bodies.",
        //                   obj_cache->collision_geometry_data_->getID().c_str());

        // add to the new cache
        cache.map_[wptr] = obj_cache;
        cache.bumpUseCount();
        return obj_cache;
      }
    }
  }

  hpp::fcl::CollisionGeometry* cg_g = nullptr;
  // handle cases individually
  switch (shape->type)
  {
    case shapes::PLANE:
    {
      const shapes::Plane* p = static_cast<const shapes::Plane*>(shape.get());
      cg_g = new hpp::fcl::Plane(p->a, p->b, p->c, p->d);
    }
    break;
    case shapes::SPHERE:
    {
      const shapes::Sphere* s = static_cast<const shapes::Sphere*>(shape.get());
      cg_g = new hpp::fcl::Sphere(s->radius);
    }
    break;
    case shapes::BOX:
    {
      const shapes::Box* s = static_cast<const shapes::Box*>(shape.get());
      const double* size = s->size;
      cg_g = new hpp::fcl::Box(size[0], size[1], size[2]);
    }
    break;
    case shapes::CYLINDER:
    {
      const shapes::Cylinder* s = static_cast<const shapes::Cylinder*>(shape.get());
      cg_g = new hpp::fcl::Cylinder(s->radius, s->length);
    }
    break;
    case shapes::CONE:
    {
      const shapes::Cone* s = static_cast<const shapes::Cone*>(shape.get());
      cg_g = new hpp::fcl::Cone(s->radius, s->length);
    }
    break;
    case shapes::MESH:
    {
      auto g = new hpp::fcl::BVHModel<BV>();
      const shapes::Mesh* mesh = static_cast<const shapes::Mesh*>(shape.get());
      if (mesh->vertex_count > 0 && mesh->triangle_count > 0)
      {
        std::vector<hpp::fcl::Triangle> tri_indices(mesh->triangle_count);
        for (unsigned int i = 0; i < mesh->triangle_count; ++i)
          tri_indices[i] =
              hpp::fcl::Triangle(mesh->triangles[3 * i], mesh->triangles[3 * i + 1], mesh->triangles[3 * i + 2]);

        std::vector<hpp::fcl::Vec3f> points(mesh->vertex_count);
        for (unsigned int i = 0; i < mesh->vertex_count; ++i)
          points[i] = hpp::fcl::Vec3f(mesh->vertices[3 * i], mesh->vertices[3 * i + 1], mesh->vertices[3 * i + 2]);

        g->beginModel();
        g->addSubModel(points, tri_indices);
        g->endModel();
      }
      cg_g = g;
    }
    break;
    case shapes::OCTREE:
    {
      const shapes::OcTree* g = static_cast<const shapes::OcTree*>(shape.get());
      cg_g = new hpp::fcl::OcTree(g->octree);
    }
    break;
    default:
      RCLCPP_ERROR(LOGGER, "This shape type (%d) is not supported using HPP FCL yet", static_cast<int>(shape->type));
      cg_g = nullptr;
  }

  if (cg_g)
  {
    cg_g->computeLocalAABB();
    HPPFCLGeometryConstPtr res = std::make_shared<const HPPFCLGeometry>(cg_g, data, shape_index);
    cache.map_[wptr] = res;
    cache.bumpUseCount();
    return res;
  }
  return HPPFCLGeometryConstPtr();
}

HPPFCLGeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr& shape, const moveit::core::LinkModel* link,
                                               int shape_index)
{
  return createCollisionGeometry<hpp::fcl::OBBRSS, moveit::core::LinkModel>(shape, link, shape_index);
}

HPPFCLGeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr& shape, const moveit::core::AttachedBody* ab,
                                               int shape_index)
{
  return createCollisionGeometry<hpp::fcl::OBBRSS, moveit::core::AttachedBody>(shape, ab, shape_index);
}

HPPFCLGeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr& shape, const World::Object* obj)
{
  return createCollisionGeometry<hpp::fcl::OBBRSS, World::Object>(shape, obj, 0);
}

/** \brief Templated helper function creating new collision geometry out of general object using an arbitrary bounding
 *  volume (BV). This can include padding and scaling. */
template <typename BV, typename T>
HPPFCLGeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr& shape, double scale, double padding,
                                               const T* data, int shape_index)
{
  if (fabs(scale - 1.0) <= std::numeric_limits<double>::epsilon() &&
      fabs(padding) <= std::numeric_limits<double>::epsilon())
    return createCollisionGeometry<BV, T>(shape, data, shape_index);
  else
  {
    shapes::ShapePtr scaled_shape(shape->clone());
    scaled_shape->scaleAndPadd(scale, padding);
    return createCollisionGeometry<BV, T>(scaled_shape, data, shape_index);
  }
}

HPPFCLGeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr& shape, double scale, double padding,
                                               const moveit::core::LinkModel* link, int shape_index)
{
  return createCollisionGeometry<hpp::fcl::OBBRSS, moveit::core::LinkModel>(shape, scale, padding, link, shape_index);
}

HPPFCLGeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr& shape, double scale, double padding,
                                               const moveit::core::AttachedBody* ab, int shape_index)
{
  return createCollisionGeometry<hpp::fcl::OBBRSS, moveit::core::AttachedBody>(shape, scale, padding, ab, shape_index);
}

HPPFCLGeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr& shape, double scale, double padding,
                                               const World::Object* obj)
{
  return createCollisionGeometry<hpp::fcl::OBBRSS, World::Object>(shape, scale, padding, obj, 0);
}

void cleanCollisionGeometryCache()
{
  HPPFCLShapeCache& cache1 = GetShapeCache<hpp::fcl::OBBRSS, World::Object>();
  {
    cache1.bumpUseCount(true);
  }
  HPPFCLShapeCache& cache2 = GetShapeCache<hpp::fcl::OBBRSS, moveit::core::AttachedBody>();
  {
    cache2.bumpUseCount(true);
  }
}

void CollisionData::enableGroup(const moveit::core::RobotModelConstPtr& robot_model)
{
  if (robot_model->hasJointModelGroup(req_->group_name))
    active_components_only_ = &robot_model->getJointModelGroup(req_->group_name)->getUpdatedLinkModelsSet();
  else
    active_components_only_ = nullptr;
}

void HPPFCLObject::registerTo(hpp::fcl::BroadPhaseCollisionManager* manager)
{
  std::vector<hpp::fcl::CollisionObject*> collision_objects(collision_objects_.size());
  for (std::size_t i = 0; i < collision_objects_.size(); ++i)
    collision_objects[i] = collision_objects_[i].get();
  if (!collision_objects.empty())
    manager->registerObjects(collision_objects);
}

void HPPFCLObject::unregisterFrom(hpp::fcl::BroadPhaseCollisionManager* manager)
{
  for (auto& collision_object : collision_objects_)
    manager->unregisterObject(collision_object.get());
}

void HPPFCLObject::clear()
{
  collision_objects_.clear();
  collision_geometry_.clear();
}
}  // namespace collision_detection
