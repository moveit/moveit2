/*********************************************************************
 * Software License Agreement (BSD-2-Clause)
 *
 * Copyright (c) 2021, Southwest Research Institute
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

/* Author: Jorge Nicho
 * Author: Levi Armstrong
 */

#include <moveit/collision_detection_bullet/bullet_integration/contact_checker_common.h>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

static const rclcpp::Logger BULLET_LOGGER = rclcpp::get_logger("collision_detection.bullet");

namespace collision_detection_bullet
{
collision_detection::Contact* processResult(ContactTestData& cdata, collision_detection::Contact& contact,
                                            const std::pair<std::string, std::string>& key, bool found)
{
  // add deepest penetration / smallest distance to result
  if (cdata.req.distance)
  {
    if (contact.depth < cdata.res.distance)
    {
      cdata.res.distance = contact.depth;
    }
  }

  RCLCPP_DEBUG_STREAM(BULLET_LOGGER, "Contact btw " << key.first << " and " << key.second << " dist: " << contact.depth);
  // case if pair hasn't a contact yet
  if (!found)
  {
    if (contact.depth <= 0)
    {
      cdata.res.collision = true;
    }

    std::vector<collision_detection::Contact> data;

    // if we don't want contacts we are done here
    if (!cdata.req.contacts)
    {
      if (!cdata.req.distance)
      {
        cdata.done = true;
      }
      return nullptr;
    }
    else
    {
      data.reserve(cdata.req.max_contacts_per_pair);
      data.emplace_back(contact);
      cdata.res.contact_count++;
    }

    if (cdata.res.contact_count >= cdata.req.max_contacts)
    {
      if (!cdata.req.distance)
      {
        cdata.done = true;
      }
    }

    if (cdata.req.max_contacts_per_pair == 1u)
    {
      cdata.pair_done = true;
    }

    return &(cdata.res.contacts.insert(std::make_pair(key, data)).first->second.back());
  }
  else
  {
    std::vector<collision_detection::Contact>& dr = cdata.res.contacts[key];
    dr.emplace_back(contact);
    cdata.res.contact_count++;

    if (dr.size() >= cdata.req.max_contacts_per_pair)
    {
      cdata.pair_done = true;
    }

    if (cdata.res.contact_count >= cdata.req.max_contacts)
    {
      if (!cdata.req.distance)
      {
        cdata.done = true;
      }
    }

    return &(dr.back());
  }

  return nullptr;
}

int createConvexHull(AlignedVector<Eigen::Vector3d>& vertices, std::vector<int>& faces,
                     const AlignedVector<Eigen::Vector3d>& input, double shrink, double shrinkClamp)
{
  vertices.clear();
  faces.clear();

  btConvexHullComputer conv;
  btAlignedObjectArray<btVector3> points;
  points.reserve(static_cast<int>(input.size()));
  for (const Eigen::Vector3d& v : input)
  {
    points.push_back(btVector3(static_cast<btScalar>(v[0]), static_cast<btScalar>(v[1]), static_cast<btScalar>(v[2])));
  }

  btScalar val = conv.compute(&points[0].getX(), sizeof(btVector3), points.size(), static_cast<btScalar>(shrink),
                              static_cast<btScalar>(shrinkClamp));
  if (val < 0)
  {
    RCLCPP_ERROR(BULLET_LOGGER, "Failed to create convex hull");
    return -1;
  }

  int num_verts = conv.vertices.size();
  vertices.reserve(static_cast<size_t>(num_verts));
  for (int i = 0; i < num_verts; ++i)
  {
    btVector3& v = conv.vertices[i];
    vertices.push_back(Eigen::Vector3d(v.getX(), v.getY(), v.getZ()));
  }

  int num_faces = conv.faces.size();
  faces.reserve(static_cast<size_t>(3 * num_faces));
  for (int i = 0; i < num_faces; ++i)
  {
    std::vector<int> face;
    face.reserve(3);

    const btConvexHullComputer::Edge* source_edge = &(conv.edges[conv.faces[i]]);
    int a = source_edge->getSourceVertex();
    face.push_back(a);

    int b = source_edge->getTargetVertex();
    face.push_back(b);

    const btConvexHullComputer::Edge* edge = source_edge->getNextEdgeOfFace();
    int c = edge->getTargetVertex();
    face.push_back(c);

    edge = edge->getNextEdgeOfFace();
    c = edge->getTargetVertex();
    while (c != a)
    {
      face.push_back(c);

      edge = edge->getNextEdgeOfFace();
      c = edge->getTargetVertex();
    }
    faces.push_back(static_cast<int>(face.size()));
    faces.insert(faces.end(), face.begin(), face.end());
  }

  return num_faces;
}

}  // namespace collision_detection_bullet
