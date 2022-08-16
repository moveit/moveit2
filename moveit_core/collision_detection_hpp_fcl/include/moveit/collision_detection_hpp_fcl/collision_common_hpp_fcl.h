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

/* Author: Ioan Sucan */

#pragma once

#include <moveit/collision_detection/world.h>
#include <moveit/collision_detection/collision_env.h>
#include <moveit/macros/class_forward.h>
#include <geometric_shapes/check_isometry.h>

#include <hpp/fcl/broadphase/broadphase_collision_manager.h>
#include <hpp/fcl/narrowphase/narrowphase.h>

#include <memory>
#include <set>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/octree.h>

namespace collision_detection
{
MOVEIT_STRUCT_FORWARD(CollisionGeometryData);

/** \brief Wrapper around world, link and attached objects' geometry data. */
struct CollisionGeometryData
{
  /** \brief Constructor for a robot link collision geometry object. */
  CollisionGeometryData(const moveit::core::LinkModel* link, int index)
    : type(BodyTypes::ROBOT_LINK), shape_index(index)
  {
    ptr.link = link;
  }

  /** \brief Constructor for a new collision geometry object which is attached to the robot. */
  CollisionGeometryData(const moveit::core::AttachedBody* ab, int index)
    : type(BodyTypes::ROBOT_ATTACHED), shape_index(index)
  {
    ptr.ab = ab;
  }

  /** \brief Constructor for a new world collision geometry. */
  CollisionGeometryData(const World::Object* obj, int index) : type(BodyTypes::WORLD_OBJECT), shape_index(index)
  {
    ptr.obj = obj;
  }

  /** \brief Returns the name which is saved in the member pointed to in \e ptr. */
  const std::string& getID() const
  {
    switch (type)
    {
      case BodyTypes::ROBOT_LINK:
        return ptr.link->getName();
      case BodyTypes::ROBOT_ATTACHED:
        return ptr.ab->getName();
      default:
        break;
    }
    return ptr.obj->id_;
  }

  /** \brief Returns a string of the corresponding \e type. */
  std::string getTypeString() const
  {
    switch (type)
    {
      case BodyTypes::ROBOT_LINK:
        return "Robot link";
      case BodyTypes::ROBOT_ATTACHED:
        return "Robot attached";
      default:
        break;
    }
    return "Object";
  }

  /** \brief Check if two CollisionGeometryData objects point to the same source object. */
  bool sameObject(const CollisionGeometryData& other) const
  {
    return type == other.type && ptr.raw == other.ptr.raw;
  }

  /** \brief Indicates the body type of the object. */
  BodyType type;

  /** \brief Multiple \e CollisionGeometryData objects construct a collision object. The collision object refers to an
   *  array of coordinate transformations at a certain start index. The index of the transformation of a child \e
   *  CollisionGeometryData object is then given by adding the parent collision object index and the \e shape_index of a
   *  geometry data object. */
  int shape_index;

  /** \brief Points to the type of body which contains the geometry. */
  union
  {
    const moveit::core::LinkModel* link;
    const moveit::core::AttachedBody* ab;
    const World::Object* obj;
    const void* raw;
  } ptr;
};

/** \brief Data structure which is passed to the collision callback function of the collision manager. */
struct CollisionData
{
  CollisionData() : req_(nullptr), active_components_only_(nullptr), res_(nullptr), acm_(nullptr), done_(false)
  {
  }

  CollisionData(const CollisionRequest* req, CollisionResult* res, const AllowedCollisionMatrix* acm)
    : req_(req), active_components_only_(nullptr), res_(res), acm_(acm), done_(false)
  {
  }

  ~CollisionData()
  {
  }

  /** \brief Compute \e active_components_only_ based on the joint group specified in \e req_ */
  void enableGroup(const moveit::core::RobotModelConstPtr& robot_model);

  /** \brief The collision request passed by the user */
  const CollisionRequest* req_;

  /** \brief  If the collision request includes a group name, this set contains the pointers to the link models that
   *  are considered for collision.
   *
   *  If the pointer is nullptr, all collisions are considered. */
  const std::set<const moveit::core::LinkModel*>* active_components_only_;

  /** \brief The user-specified response location. */
  CollisionResult* res_;

  /** \brief The user-specified collision matrix (may be nullptr). */
  const AllowedCollisionMatrix* acm_;

  /** \brief Flag indicating whether collision checking is complete. */
  bool done_;
};

/** \brief Data structure which is passed to the distance callback function of the collision manager. */
struct DistanceData
{
  DistanceData(const DistanceRequest* req, DistanceResult* res) : req(req), res(res), done(false)
  {
  }
  ~DistanceData()
  {
  }

  /** \brief Distance query request information. */
  const DistanceRequest* req;

  /** \brief Distance query results information. */
  DistanceResult* res;

  /** \brief Indicates if distance query is finished. */
  bool done;
};

MOVEIT_STRUCT_FORWARD(HPPFCLGeometry);

/** \brief Bundles the \e CollisionGeometryData and FCL collision geometry representation into a single class. */
struct HPPFCLGeometry
{
  HPPFCLGeometry()
  {
  }

  /** \brief Constructor for a robot link. */
  HPPFCLGeometry(hpp::fcl::CollisionGeometry* collision_geometry, const moveit::core::LinkModel* link, int shape_index)
    : collision_geometry_(collision_geometry)
    , collision_geometry_data_(std::make_shared<CollisionGeometryData>(link, shape_index))
  {
    collision_geometry_->setUserData(collision_geometry_data_.get());
  }

  /** \brief Constructor for an attached body. */
  HPPFCLGeometry(hpp::fcl::CollisionGeometry* collision_geometry, const moveit::core::AttachedBody* ab, int shape_index)
    : collision_geometry_(collision_geometry)
    , collision_geometry_data_(std::make_shared<CollisionGeometryData>(ab, shape_index))
  {
    collision_geometry_->setUserData(collision_geometry_data_.get());
  }

  /** \brief Constructor for a world object. */
  HPPFCLGeometry(hpp::fcl::CollisionGeometry* collision_geometry, const World::Object* obj, int shape_index)
    : collision_geometry_(collision_geometry)
    , collision_geometry_data_(std::make_shared<CollisionGeometryData>(obj, shape_index))
  {
    collision_geometry_->setUserData(collision_geometry_data_.get());
  }

  /** \brief Updates the \e collision_geometry_data_ with new data while also setting the \e collision_geometry_ to the
   *   new data. */
  template <typename T>
  void updateCollisionGeometryData(const T* data, int shape_index, bool newType)
  {
    if (!newType && collision_geometry_data_)
      if (collision_geometry_data_->ptr.raw == reinterpret_cast<const void*>(data))
        return;
    collision_geometry_data_ = std::make_shared<CollisionGeometryData>(data, shape_index);
    collision_geometry_->setUserData(collision_geometry_data_.get());
  }

  /** \brief Pointer to FCL collision geometry. */
  std::shared_ptr<hpp::fcl::CollisionGeometry> collision_geometry_;

  /** \brief Pointer to the user-defined geometry data. */
  CollisionGeometryDataPtr collision_geometry_data_;
};

typedef std::shared_ptr<hpp::fcl::CollisionObject> HPPFCLCollisionObjectPtr;
typedef std::shared_ptr<const hpp::fcl::CollisionObject> HPPFCLCollisionObjectConstPtr;

/** \brief A general high-level object which consists of multiple \e FCLCollisionObjects. It is the top level data
 *  structure which is used in the collision checking process. */
struct HPPFCLObject
{
  void registerTo(hpp::fcl::BroadPhaseCollisionManager* manager);
  void unregisterFrom(hpp::fcl::BroadPhaseCollisionManager* manager);
  void clear();

  std::vector<HPPFCLCollisionObjectPtr> collision_objects_;

  /** \brief Geometry data corresponding to \e collision_objects_. */
  std::vector<HPPFCLGeometryConstPtr> collision_geometry_;
};

/** \brief Bundles an \e FCLObject and a broadphase FCL collision manager. */
struct HPPFCLManager
{
  HPPFCLObject object_;
  std::shared_ptr<hpp::fcl::BroadPhaseCollisionManager> manager_;
};

/** \brief Callback function used by the FCLManager used for each pair of collision objects to
 *   calculate object contact information.
 *
 *   \param o1 First FCL collision object
 *   \param o2 Second FCL collision object
 *   \data General pointer to arbitrary data which is used during the callback
 *   \return True terminates the collision check, false continues it to the next pair of objects */
struct CollisionCallback : hpp::fcl::CollisionCallBackBase
{
  CollisionData* data;

  virtual bool collide(hpp::fcl::CollisionObject* o1, hpp::fcl::CollisionObject* o2) override
  {
    CollisionData* cdata = reinterpret_cast<CollisionData*>(data);
    if (cdata->done_)
      return true;
    const CollisionGeometryData* cd1 =
        static_cast<const CollisionGeometryData*>(o1->collisionGeometry()->getUserData());
    const CollisionGeometryData* cd2 =
        static_cast<const CollisionGeometryData*>(o2->collisionGeometry()->getUserData());

    // do not collision check geoms part of the same object / link / attached body
    if (cd1->sameObject(*cd2))
      return false;

    // If active components are specified
    if (cdata->active_components_only_)
    {
      const moveit::core::LinkModel* l1 =
          cd1->type == BodyTypes::ROBOT_LINK ?
              cd1->ptr.link :
              (cd1->type == BodyTypes::ROBOT_ATTACHED ? cd1->ptr.ab->getAttachedLink() : nullptr);
      const moveit::core::LinkModel* l2 =
          cd2->type == BodyTypes::ROBOT_LINK ?
              cd2->ptr.link :
              (cd2->type == BodyTypes::ROBOT_ATTACHED ? cd2->ptr.ab->getAttachedLink() : nullptr);

      // If neither of the involved components is active
      if ((!l1 || cdata->active_components_only_->find(l1) == cdata->active_components_only_->end()) &&
          (!l2 || cdata->active_components_only_->find(l2) == cdata->active_components_only_->end()))
        return false;
    }

    // use the collision matrix (if any) to avoid certain collision checks
    DecideContactFn dcf;
    bool always_allow_collision = false;
    if (cdata->acm_)
    {
      AllowedCollision::Type type;
      bool found = cdata->acm_->getAllowedCollision(cd1->getID(), cd2->getID(), type);
      if (found)
      {
        // if we have an entry in the collision matrix, we read it
        if (type == AllowedCollision::ALWAYS)
        {
          always_allow_collision = true;
          if (cdata->req_->verbose)
            RCLCPP_DEBUG(LOGGER,
                         "Collision between '%s' (type '%s') and '%s' (type '%s') is always allowed. "
                         "No contacts are computed.",
                         cd1->getID().c_str(), cd1->getTypeString().c_str(), cd2->getID().c_str(),
                         cd2->getTypeString().c_str());
        }
        else if (type == AllowedCollision::CONDITIONAL)
        {
          cdata->acm_->getAllowedCollision(cd1->getID(), cd2->getID(), dcf);
          if (cdata->req_->verbose)
            RCLCPP_DEBUG(LOGGER, "Collision between '%s' and '%s' is conditionally allowed", cd1->getID().c_str(),
                         cd2->getID().c_str());
        }
      }
    }

    // check if a link is touching an attached object
    if (cd1->type == BodyTypes::ROBOT_LINK && cd2->type == BodyTypes::ROBOT_ATTACHED)
    {
      const std::set<std::string>& tl = cd2->ptr.ab->getTouchLinks();
      if (tl.find(cd1->getID()) != tl.end())
      {
        always_allow_collision = true;
        if (cdata->req_->verbose)
          RCLCPP_DEBUG(LOGGER, "Robot link '%s' is allowed to touch attached object '%s'. No contacts are computed.",
                       cd1->getID().c_str(), cd2->getID().c_str());
      }
    }
    else if (cd2->type == BodyTypes::ROBOT_LINK && cd1->type == BodyTypes::ROBOT_ATTACHED)
    {
      const std::set<std::string>& tl = cd1->ptr.ab->getTouchLinks();
      if (tl.find(cd2->getID()) != tl.end())
      {
        always_allow_collision = true;
        if (cdata->req_->verbose)
          RCLCPP_DEBUG(LOGGER, "Robot link '%s' is allowed to touch attached object '%s'. No contacts are computed.",
                       cd2->getID().c_str(), cd1->getID().c_str());
      }
    }
    // bodies attached to the same link should not collide
    if (cd1->type == BodyTypes::ROBOT_ATTACHED && cd2->type == BodyTypes::ROBOT_ATTACHED)
    {
      if (cd1->ptr.ab->getAttachedLink() == cd2->ptr.ab->getAttachedLink())
        always_allow_collision = true;
    }

    // if collisions are always allowed, we are done
    if (always_allow_collision)
      return false;

    if (cdata->req_->verbose)
      RCLCPP_DEBUG(LOGGER, "Actually checking collisions between %s and %s", cd1->getID().c_str(), cd2->getID().c_str());

    // see if we need to compute a contact
    std::size_t want_contact_count = 0;
    if (cdata->req_->contacts)
      if (cdata->res_->contact_count < cdata->req_->max_contacts)
      {
        std::size_t have;
        if (cd1->getID() < cd2->getID())
        {
          std::pair<std::string, std::string> cp(cd1->getID(), cd2->getID());
          have = cdata->res_->contacts.find(cp) != cdata->res_->contacts.end() ? cdata->res_->contacts[cp].size() : 0;
        }
        else
        {
          std::pair<std::string, std::string> cp(cd2->getID(), cd1->getID());
          have = cdata->res_->contacts.find(cp) != cdata->res_->contacts.end() ? cdata->res_->contacts[cp].size() : 0;
        }
        if (have < cdata->req_->max_contacts_per_pair)
          want_contact_count = std::min(cdata->req_->max_contacts_per_pair - have,
                                        cdata->req_->max_contacts - cdata->res_->contact_count);
      }

    if (dcf)
    {
      // if we have a decider for allowed contacts, we need to look at all the contacts
      bool enable_cost = cdata->req_->cost;
      std::size_t num_max_cost_sources = cdata->req_->max_cost_sources;
      hpp::fcl::CollisionResult col_result;
      int num_contacts = hpp::fcl::collide(o1, o2,
                                           hpp::fcl::CollisionRequest(hpp::fcl::CollisionRequestFlag::CONTACT,
                                                                      std::numeric_limits<size_t>::max()),
                                           col_result);
      if (num_contacts > 0)
      {
        if (cdata->req_->verbose)
          RCLCPP_INFO(LOGGER,
                      "Found %d contacts between '%s' and '%s'. "
                      "These contacts will be evaluated to check if they are accepted or not",
                      num_contacts, cd1->getID().c_str(), cd2->getID().c_str());
        Contact c;
        const std::pair<std::string, std::string>& pc = cd1->getID() < cd2->getID() ?
                                                            std::make_pair(cd1->getID(), cd2->getID()) :
                                                            std::make_pair(cd2->getID(), cd1->getID());
        for (int i = 0; i < num_contacts; ++i)
        {
          hppfcl2contact(col_result.getContact(i), c);
          // if the contact is  not allowed, we have a collision
          if (!dcf(c))
          {
            // store the contact, if it is needed
            if (want_contact_count > 0)
            {
              --want_contact_count;
              cdata->res_->contacts[pc].push_back(c);
              cdata->res_->contact_count++;
              if (cdata->req_->verbose)
                RCLCPP_INFO(LOGGER, "Found unacceptable contact between '%s' and '%s'. Contact was stored.",
                            cd1->getID().c_str(), cd2->getID().c_str());
            }
            else if (cdata->req_->verbose)
              RCLCPP_INFO(LOGGER,
                          "Found unacceptable contact between '%s' (type '%s') and '%s' "
                          "(type '%s'). Contact was stored.",
                          cd1->getID().c_str(), cd1->getTypeString().c_str(), cd2->getID().c_str(),
                          cd2->getTypeString().c_str());
            cdata->res_->collision = true;
            if (want_contact_count == 0)
              break;
          }
        }
      }

      if (enable_cost)
      {
        std::vector<hpp::fcl::Contact> contacts;
        col_result.getContacts(contacts);
        CostSource cs;
        for (const auto& contact : contacts)
        {
          hppfcl2costsource(*contact.o1, cs);
          cdata->res_->cost_sources.insert(cs);
          hppfcl2costsource(*contact.o2, cs);
          cdata->res_->cost_sources.insert(cs);
          while (cdata->res_->cost_sources.size() > cdata->req_->max_cost_sources)
            cdata->res_->cost_sources.erase(--cdata->res_->cost_sources.end());
        }
      }
    }
    else
    {
      if (want_contact_count > 0)
      {
        // otherwise, we need to compute more things
        bool enable_cost = cdata->req_->cost;
        std::size_t num_max_cost_sources = cdata->req_->max_cost_sources;
        bool enable_contact = true;

        hpp::fcl::CollisionResult col_result;
        int num_contacts =
            hpp::fcl::collide(o1, o2,
                              hpp::fcl::CollisionRequest(hpp::fcl::CollisionRequestFlag::CONTACT, want_contact_count),
                              col_result);
        if (num_contacts > 0)
        {
          int num_contacts_initial = num_contacts;

          // make sure we don't get more contacts than we want
          if (want_contact_count >= (std::size_t)num_contacts)
            want_contact_count -= num_contacts;
          else
          {
            num_contacts = want_contact_count;
            want_contact_count = 0;
          }

          if (cdata->req_->verbose)
            RCLCPP_INFO(LOGGER,
                        "Found %d contacts between '%s' (type '%s') and '%s' (type '%s'), "
                        "which constitute a collision. %d contacts will be stored",
                        num_contacts_initial, cd1->getID().c_str(), cd1->getTypeString().c_str(), cd2->getID().c_str(),
                        cd2->getTypeString().c_str(), num_contacts);

          const std::pair<std::string, std::string>& pc = cd1->getID() < cd2->getID() ?
                                                              std::make_pair(cd1->getID(), cd2->getID()) :
                                                              std::make_pair(cd2->getID(), cd1->getID());
          cdata->res_->collision = true;
          for (int i = 0; i < num_contacts; ++i)
          {
            Contact c;
            hppfcl2contact(col_result.getContact(i), c);
            cdata->res_->contacts[pc].push_back(c);
            cdata->res_->contact_count++;
          }
        }

        if (enable_cost)
        {
          std::vector<hpp::fcl::Contact> contacts;
          col_result.getContacts(contacts);
          CostSource cs;
          for (const auto& contact : contacts)
          {
            hppfcl2costsource(*contact.o1, cs);
            cdata->res_->cost_sources.insert(cs);
            hppfcl2costsource(*contact.o2, cs);
            cdata->res_->cost_sources.insert(cs);
            while (cdata->res_->cost_sources.size() > cdata->req_->max_cost_sources)
              cdata->res_->cost_sources.erase(--cdata->res_->cost_sources.end());
          }
        }
      }
      else
      {
        bool enable_cost = cdata->req_->cost;
        std::size_t num_max_cost_sources = cdata->req_->max_cost_sources;
        bool enable_contact = false;
        hpp::fcl::CollisionResult col_result;
        // TODO (vatanaksoytezer): Are we asking distance_lower_bound here?
        int num_contacts = hpp::fcl::collide(
            o1, o2, hpp::fcl::CollisionRequest(hpp::fcl::CollisionRequestFlag::DISTANCE_LOWER_BOUND, 1), col_result);
        if (num_contacts > 0)
        {
          cdata->res_->collision = true;
          if (cdata->req_->verbose)
            RCLCPP_INFO(LOGGER,
                        "Found a contact between '%s' (type '%s') and '%s' (type '%s'), "
                        "which constitutes a collision. "
                        "Contact information is not stored.",
                        cd1->getID().c_str(), cd1->getTypeString().c_str(), cd2->getID().c_str(),
                        cd2->getTypeString().c_str());
        }

        if (enable_cost)
        {
          std::vector<hpp::fcl::Contact> contacts;
          col_result.getContacts(contacts);
          CostSource cs;
          for (const auto& contact : contacts)
          {
            hppfcl2costsource(*contact.o1, cs);
            cdata->res_->cost_sources.insert(cs);
            hppfcl2costsource(*contact.o2, cs);
            cdata->res_->cost_sources.insert(cs);
            while (cdata->res_->cost_sources.size() > cdata->req_->max_cost_sources)
              cdata->res_->cost_sources.erase(--cdata->res_->cost_sources.end());
          }
        }
      }
    }

    if (cdata->res_->collision)
      if (!cdata->req_->contacts || cdata->res_->contact_count >= cdata->req_->max_contacts)
      {
        if (!cdata->req_->cost)
          cdata->done_ = true;
        if (cdata->req_->verbose)
          RCLCPP_INFO(LOGGER,
                      "Collision checking is considered complete (collision was found and %u contacts are stored)",
                      (unsigned int)cdata->res_->contact_count);
      }

    if (!cdata->done_ && cdata->req_->is_done)
    {
      cdata->done_ = cdata->req_->is_done(*cdata->res_);
      if (cdata->done_ && cdata->req_->verbose)
        RCLCPP_INFO(LOGGER,
                    "Collision checking is considered complete due to external callback. "
                    "%s was found. %u contacts are stored.",
                    cdata->res_->collision ? "Collision" : "No collision", (unsigned int)cdata->res_->contact_count);
    }

    return cdata->done_;
  }
};

/** \brief Callback function used by the FCLManager used for each pair of collision objects to
 *   calculate collisions and distances.
 *
 *   \param o1 First FCL collision object
 *   \param o2 Second FCL collision object
 *   \data General pointer to arbitrary data which is used during the callback
 *   \return True terminates the distance check, false continues it to the next pair of objects */
// bool distanceCallback(hpp::fcl::CollisionObject* o1, hpp::fcl::CollisionObject* o2, void* data, double& min_dist);
struct DistanceCallback : public hpp::fcl::DistanceCallBackBase
{
  DistanceData* data;

  virtual bool distance(hpp::fcl::CollisionObject* o1, hpp::fcl::CollisionObject* o2, hpp::fcl::FCL_REAL& dist) override
  {
    DistanceData* cdata = data;

    const CollisionGeometryData* cd1 =
        static_cast<const CollisionGeometryData*>(o1->collisionGeometry()->getUserData());
    const CollisionGeometryData* cd2 =
        static_cast<const CollisionGeometryData*>(o2->collisionGeometry()->getUserData());

    // do not distance check for geoms part of the same object / link / attached body
    if (cd1->sameObject(*cd2))
      return false;

    // If active components are specified
    if (cdata->req->active_components_only)
    {
      const moveit::core::LinkModel* l1 =
          cd1->type == BodyTypes::ROBOT_LINK ?
              cd1->ptr.link :
              (cd1->type == BodyTypes::ROBOT_ATTACHED ? cd1->ptr.ab->getAttachedLink() : nullptr);
      const moveit::core::LinkModel* l2 =
          cd2->type == BodyTypes::ROBOT_LINK ?
              cd2->ptr.link :
              (cd2->type == BodyTypes::ROBOT_ATTACHED ? cd2->ptr.ab->getAttachedLink() : nullptr);

      // If neither of the involved components is active
      if ((!l1 || cdata->req->active_components_only->find(l1) == cdata->req->active_components_only->end()) &&
          (!l2 || cdata->req->active_components_only->find(l2) == cdata->req->active_components_only->end()))
      {
        return false;
      }
    }

    // use the collision matrix (if any) to avoid certain distance checks
    bool always_allow_collision = false;
    if (cdata->req->acm)
    {
      AllowedCollision::Type type;
      bool found = cdata->req->acm->getAllowedCollision(cd1->getID(), cd2->getID(), type);
      if (found)
      {
        // if we have an entry in the collision matrix, we read it
        if (type == AllowedCollision::ALWAYS)
        {
          always_allow_collision = true;
          if (cdata->req->verbose)
            RCLCPP_DEBUG(LOGGER, "Collision between '%s' and '%s' is always allowed. No distances are computed.",
                         cd1->getID().c_str(), cd2->getID().c_str());
        }
      }
    }

    // check if a link is touching an attached object
    if (cd1->type == BodyTypes::ROBOT_LINK && cd2->type == BodyTypes::ROBOT_ATTACHED)
    {
      const std::set<std::string>& tl = cd2->ptr.ab->getTouchLinks();
      if (tl.find(cd1->getID()) != tl.end())
      {
        always_allow_collision = true;
        if (cdata->req->verbose)
          RCLCPP_DEBUG(LOGGER, "Robot link '%s' is allowed to touch attached object '%s'. No distances are computed.",
                       cd1->getID().c_str(), cd2->getID().c_str());
      }
    }
    else if (cd2->type == BodyTypes::ROBOT_LINK && cd1->type == BodyTypes::ROBOT_ATTACHED)
    {
      const std::set<std::string>& tl = cd1->ptr.ab->getTouchLinks();
      if (tl.find(cd2->getID()) != tl.end())
      {
        always_allow_collision = true;
        if (cdata->req->verbose)
          RCLCPP_DEBUG(LOGGER, "Robot link '%s' is allowed to touch attached object '%s'. No distances are computed.",
                       cd2->getID().c_str(), cd1->getID().c_str());
      }
    }

    if (always_allow_collision)
    {
      return false;
    }
    if (cdata->req->verbose)
      RCLCPP_DEBUG(LOGGER, "Actually checking collisions between %s and %s", cd1->getID().c_str(), cd2->getID().c_str());

    double dist_threshold = cdata->req->distance_threshold;

    const std::pair<const std::string&, const std::string&> pc =
        cd1->getID() < cd2->getID() ? std::make_pair(std::cref(cd1->getID()), std::cref(cd2->getID())) :
                                      std::make_pair(std::cref(cd2->getID()), std::cref(cd1->getID()));

    DistanceMap::iterator it = cdata->res->distances.find(pc);

    // GLOBAL search: for efficiency, distance_threshold starts at the smallest distance between any pairs found so far
    if (cdata->req->type == DistanceRequestType::GLOBAL)
    {
      dist_threshold = cdata->res->minimum_distance.distance;
    }
    // Check if a distance between this pair has been found yet. Decrease threshold_distance if so, to narrow the search
    else if (it != cdata->res->distances.end())
    {
      if (cdata->req->type == DistanceRequestType::LIMITED)
      {
        // If at the limit for a given pair just return
        if (it->second.size() >= cdata->req->max_contacts_per_body)
        {
          return cdata->done;
        }
      }
      else if (cdata->req->type == DistanceRequestType::SINGLE)
      {
        dist_threshold = it->second[0].distance;
      }
    }

    hpp::fcl::DistanceResult fcl_result;
    fcl_result.min_distance = dist_threshold;
    // fcl::distance segfaults when given an octree with a null root pointer (using FCL 0.6.1)
    if ((o1->getObjectType() == hpp::fcl::OT_OCTREE &&
         !std::static_pointer_cast<const hpp::fcl::OcTree>(o1->collisionGeometry())->getRoot()) ||
        (o2->getObjectType() == hpp::fcl::OT_OCTREE &&
         !std::static_pointer_cast<const hpp::fcl::OcTree>(o2->collisionGeometry())->getRoot()))
    {
      return false;
    }
    hpp::fcl::DistanceRequest fcl_req;
    fcl_req = hpp::fcl::DistanceRequest(cdata->req->enable_nearest_points, 0, 0);
    double distance = hpp::fcl::distance(o1, o2, fcl_req, fcl_result);

    // Check if either object is already in the map. If not add it or if present
    // check to see if the new distance is closer. If closer remove the existing
    // one and add the new distance information.
    if (distance < dist_threshold)
    {
      // thread_local storage makes this variable persistent. We do not clear it at every iteration because all members
      // get overwritten.
      thread_local DistanceResultsData dist_result;
      dist_result.distance = fcl_result.min_distance;

      // Careful here: Get the collision geometry data again, since FCL might
      // swap o1 and o2 in the result.
      const CollisionGeometryData* res_cd1 = static_cast<const CollisionGeometryData*>(fcl_result.o1->getUserData());
      const CollisionGeometryData* res_cd2 = static_cast<const CollisionGeometryData*>(fcl_result.o2->getUserData());

      dist_result.nearest_points[0] = fcl_result.nearest_points[0];
      dist_result.nearest_points[1] = fcl_result.nearest_points[1];
      dist_result.link_names[0] = res_cd1->getID();
      dist_result.link_names[1] = res_cd2->getID();
      dist_result.body_types[0] = res_cd1->type;
      dist_result.body_types[1] = res_cd2->type;
      if (cdata->req->enable_nearest_points)
      {
        dist_result.normal = (dist_result.nearest_points[1] - dist_result.nearest_points[0]).normalized();
      }

      if (distance <= 0 && cdata->req->enable_signed_distance)
      {
        dist_result.nearest_points[0].setZero();
        dist_result.nearest_points[1].setZero();
        dist_result.normal.setZero();

        hpp::fcl::CollisionRequest coll_req;
        thread_local hpp::fcl::CollisionResult coll_res;
        coll_res.clear();  // thread_local storage makes the variable persistent. Ensure that it is cleared!
        coll_req.enable_contact = true;
        coll_req.num_max_contacts = 200;
        std::size_t contacts = hpp::fcl::collide(o1, o2, coll_req, coll_res);
        if (contacts > 0)
        {
          double max_dist = 0;
          int max_index = 0;
          for (std::size_t i = 0; i < contacts; ++i)
          {
            const hpp::fcl::Contact& contact = coll_res.getContact(i);
            if (contact.penetration_depth > max_dist)
            {
              max_dist = contact.penetration_depth;
              max_index = i;
            }
          }

          const hpp::fcl::Contact& contact = coll_res.getContact(max_index);
          dist_result.distance = -contact.penetration_depth;
          dist_result.nearest_points[0] = contact.pos;
          dist_result.nearest_points[1] = contact.pos;

          if (cdata->req->enable_nearest_points)
          {
            Eigen::Vector3d normal(contact.normal);

            // Check order of o1/o2 again, we might need to flip the normal
            if (contact.o1 == o1->collisionGeometry().get())
              dist_result.normal = normal;
            else
              dist_result.normal = -normal;
          }
        }
      }

      if (dist_result.distance < cdata->res->minimum_distance.distance)
      {
        cdata->res->minimum_distance = dist_result;
      }

      if (dist_result.distance <= 0)
      {
        cdata->res->collision = true;
      }

      if (cdata->req->type != DistanceRequestType::GLOBAL)
      {
        if (it == cdata->res->distances.end())
        {
          std::vector<DistanceResultsData> data;
          data.reserve(cdata->req->type == DistanceRequestType::SINGLE ? 1 : cdata->req->max_contacts_per_body);
          data.push_back(dist_result);
          cdata->res->distances.insert(std::make_pair(pc, data));
        }
        else
        {
          if (cdata->req->type == DistanceRequestType::ALL)
          {
            it->second.push_back(dist_result);
          }
          else if (cdata->req->type == DistanceRequestType::SINGLE)
          {
            if (dist_result.distance < it->second[0].distance)
              it->second[0] = dist_result;
          }
          else if (cdata->req->type == DistanceRequestType::LIMITED)
          {
            assert(it->second.size() < cdata->req->max_contacts_per_body);
            it->second.push_back(dist_result);
          }
        }
      }

      if (!cdata->req->enable_signed_distance && cdata->res->collision)
      {
        cdata->done = true;
      }
    }

    return cdata->done;
  }
};

/** \brief Create new FCLGeometry object out of robot link model. */
HPPFCLGeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr& shape, const moveit::core::LinkModel* link,
                                               int shape_index);

/** \brief Create new FCLGeometry object out of attached body. */
HPPFCLGeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr& shape, const moveit::core::AttachedBody* ab,
                                               int shape_index);

/** \brief Create new FCLGeometry object out of a world object.
 *
 *  A world object always consists only of a single shape, therefore we don't need the \e shape_index. */
HPPFCLGeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr& shape, const World::Object* obj);

/** \brief Create new scaled and / or padded FCLGeometry object out of robot link model. */
HPPFCLGeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr& shape, double scale, double padding,
                                               const moveit::core::LinkModel* link, int shape_index);

/** \brief Create new scaled and / or padded FCLGeometry object out of an attached body. */
HPPFCLGeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr& shape, double scale, double padding,
                                               const moveit::core::AttachedBody* ab, int shape_index);

/** \brief Create new scaled and / or padded FCLGeometry object out of an world object. */
HPPFCLGeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr& shape, double scale, double padding,
                                               const World::Object* obj);

/** \brief Increases the counter of the caches which can trigger the cleaning of expired entries from them. */
void cleanCollisionGeometryCache();

/** \brief Transforms an Eigen Isometry3d to FCL coordinate transformation */
inline void transform2hppfcl(const Eigen::Isometry3d& b, hpp::fcl::Transform3f& f)
{
  ASSERT_ISOMETRY(b);
  Eigen::Quaterniond q(b.linear());
  f.setTranslation(hpp::fcl::Vec3f(b.translation().x(), b.translation().y(), b.translation().z()));
  f.setQuatRotation(hpp::fcl::Quaternion3f(q.w(), q.x(), q.y(), q.z()));
}

/** \brief Transforms an Eigen Isometry3d to FCL coordinate transformation */
inline hpp::fcl::Transform3f transform2hppfcl(const Eigen::Isometry3d& b)
{
  hpp::fcl::Transform3f t;
  transform2hppfcl(b, t);
  return t;
}

/** \brief Transforms an FCL contact into a MoveIt contact point. */
inline void hppfcl2contact(const hpp::fcl::Contact& fc, Contact& c)
{
  c.pos = Eigen::Vector3d(fc.pos[0], fc.pos[1], fc.pos[2]);
  c.normal = Eigen::Vector3d(fc.normal[0], fc.normal[1], fc.normal[2]);
  c.depth = fc.penetration_depth;
  const CollisionGeometryData* cgd1 = static_cast<const CollisionGeometryData*>(fc.o1->getUserData());
  c.body_name_1 = cgd1->getID();
  c.body_type_1 = cgd1->type;
  const CollisionGeometryData* cgd2 = static_cast<const CollisionGeometryData*>(fc.o2->getUserData());
  c.body_name_2 = cgd2->getID();
  c.body_type_2 = cgd2->type;
}

/** \brief Transforms the FCL internal representation from a Collision Geometry to the MoveIt \e CostSource data structure. */
inline void hppfcl2costsource(const hpp::fcl::CollisionGeometry& fcs, CostSource& cs)
{
  cs.aabb_min[0] = fcs.aabb_local.min_[0];
  cs.aabb_min[1] = fcs.aabb_local.min_[1];
  cs.aabb_min[2] = fcs.aabb_local.min_[2];

  cs.aabb_max[0] = fcs.aabb_local.max_[0];
  cs.aabb_max[1] = fcs.aabb_local.max_[1];
  cs.aabb_max[2] = fcs.aabb_local.max_[2];

  cs.cost = fcs.cost_density;
}

}  // namespace collision_detection
