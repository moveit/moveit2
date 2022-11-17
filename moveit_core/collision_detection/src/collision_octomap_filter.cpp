/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Adam Leeper */

#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_octomap_filter.h>
#include <octomap/math/Vector3.h>
#include <octomap/math/Utils.h>
#include <octomap/octomap.h>
#include <geometric_shapes/shapes.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <memory>

// Logger
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_collision_detection.collision_octomap_filter");

// forward declarations
bool getMetaballSurfaceProperties(const octomap::point3d_list& cloud, const double& spacing, const double& iso_value,
                                  const double& r_multiple, const octomath::Vector3& contact_point,
                                  octomath::Vector3& normal, double& depth, bool estimate_depth);

bool findSurface(const octomap::point3d_list& cloud, const double& spacing, const double& iso_value,
                 const double& r_multiple, const octomath::Vector3& seed, octomath::Vector3& surface_point,
                 octomath::Vector3& normal);

bool sampleCloud(const octomap::point3d_list& cloud, const double& spacing, const double& r_multiple,
                 const octomath::Vector3& position, double& intensity, octomath::Vector3& gradient);

int collision_detection::refineContactNormals(const World::ObjectConstPtr& object, CollisionResult& res,
                                              const double cell_bbx_search_distance,
                                              const double allowed_angle_divergence, const bool estimate_depth,
                                              const double iso_value, const double metaball_radius_multiple)
{
  if (!object)
  {
    RCLCPP_ERROR(LOGGER, "No valid Object passed in, cannot refine Normals!");
    return 0;
  }
  if (res.contact_count < 1)
  {
    RCLCPP_WARN(LOGGER, "There do not appear to be any contacts, so there is nothing to refine!");
    return 0;
  }

  int modified = 0;

  // iterate through contacts
  for (auto& contact : res.contacts)
  {
    const std::string contact1 = contact.first.first;
    const std::string contact2 = contact.first.second;
    std::string octomap_name = "";
    std::vector<collision_detection::Contact>& contact_vector = contact.second;

    if (contact1.find("octomap") != std::string::npos)
      octomap_name = contact1;
    else if (contact2.find("octomap") != std::string::npos)
      octomap_name = contact2;
    else
    {
      continue;
    }

    double cell_size = 0;
    if (!object->shapes_.empty())
    {
      const shapes::ShapeConstPtr& shape = object->shapes_[0];
      const std::shared_ptr<const shapes::OcTree> shape_octree = std::dynamic_pointer_cast<const shapes::OcTree>(shape);
      if (shape_octree)
      {
        std::shared_ptr<const octomap::OcTree> octree = shape_octree->octree;
        cell_size = octree->getResolution();
        for (auto& contact_info : contact_vector)
        {
          const Eigen::Vector3d& point = contact_info.pos;
          const Eigen::Vector3d& normal = contact_info.normal;

          const octomath::Vector3 contact_point(point[0], point[1], point[2]);
          const octomath::Vector3 contact_normal(normal[0], normal[1], normal[2]);
          const octomath::Vector3 diagonal = octomath::Vector3(1, 1, 1);
          const octomath::Vector3 bbx_min = contact_point - diagonal * cell_size * cell_bbx_search_distance;
          const octomath::Vector3 bbx_max = contact_point + diagonal * cell_size * cell_bbx_search_distance;
          octomap::point3d_list node_centers;
          octomap::OcTreeBaseImpl<octomap::OcTreeNode, octomap::AbstractOccupancyOcTree>::leaf_bbx_iterator it =
              octree->begin_leafs_bbx(bbx_min, bbx_max);
          octomap::OcTreeBaseImpl<octomap::OcTreeNode, octomap::AbstractOccupancyOcTree>::leaf_bbx_iterator leafs_end =
              octree->end_leafs_bbx();
          int count = 0;
          for (; it != leafs_end; ++it)
          {
            const octomap::point3d pt = it.getCoordinate();
            // double prob = it->getOccupancy();
            if (octree->isNodeOccupied(*it))  // magic number!
            {
              count++;
              node_centers.push_back(pt);
              // RCLCPP_INFO(LOGGER, "Adding point %d with prob %.3f at [%.3f, %.3f, %.3f]",
              //                          count, prob, pt.x(), pt.y(), pt.z());
            }
          }
          // RCLCPP_INFO(LOGGER, "Contact point at [%.3f, %.3f, %.3f], cell size %.3f, occupied cells
          // %d",
          //                          contact_point.x(), contact_point.y(), contact_point.z(), cell_size, count);

          // octree->getOccupiedLeafsBBX(node_centers, bbx_min, bbx_max);
          // RCLCPP_ERROR(LOGGER, "bad stuff in collision_octomap_filter.cpp; need to port octomap
          // call for groovy");

          octomath::Vector3 n;
          double depth;
          if (getMetaballSurfaceProperties(node_centers, cell_size, iso_value, metaball_radius_multiple, contact_point,
                                           n, depth, estimate_depth))
          {
            // only modify normal if the refinement predicts a "very different" result.
            const double divergence = contact_normal.angleTo(n);
            if (divergence > allowed_angle_divergence)
            {
              modified++;
              // RCLCPP_INFO(LOGGER, "Normals differ by %.3f, changing: [%.3f, %.3f, %.3f] -> [%.3f,
              // %.3f, %.3f]",
              //                          divergence, contact_normal.x(), contact_normal.y(), contact_normal.z(),
              //                          n.x(), n.y(), n.z());
              contact_info.normal = Eigen::Vector3d(n.x(), n.y(), n.z());
            }

            if (estimate_depth)
              contact_info.depth = depth;
          }
        }
      }
    }
  }
  return modified;
}

bool getMetaballSurfaceProperties(const octomap::point3d_list& cloud, const double& spacing, const double& iso_value,
                                  const double& r_multiple, const octomath::Vector3& contact_point,
                                  octomath::Vector3& normal, double& depth, const bool estimate_depth)
{
  double intensity;
  if (estimate_depth)
  {
    octomath::Vector3 surface_point;
    if (findSurface(cloud, spacing, iso_value, r_multiple, contact_point, surface_point, normal))
    {
      depth = normal.dot(surface_point - contact_point);  // do we prefer this, or magnitude of surface - contact?
      return true;
    }
    else
    {
      return false;
    }
  }
  else  // just get normals, no depth
  {
    octomath::Vector3 gradient;
    if (sampleCloud(cloud, spacing, r_multiple, contact_point, intensity, gradient))
    {
      normal = gradient.normalized();
      return true;
    }
    else
    {
      return false;
    }
  }
}

// --------------------------------------------------------------------------
// This algorithm is from Salisbury & Tarr's 1997 paper.  It will find the
// closest point on the surface starting from a seed point that is close by
// following the direction of the field gradient.
bool findSurface(const octomap::point3d_list& cloud, const double& spacing, const double& iso_value,
                 const double& r_multiple, const octomath::Vector3& seed, octomath::Vector3& surface_point,
                 octomath::Vector3& normal)
{
  const double epsilon = 1e-10;
  const int iterations = 10;
  double intensity = 0;

  octomath::Vector3 p = seed, dp, gs;
  for (int i = 0; i < iterations; ++i)
  {
    if (!sampleCloud(cloud, spacing, r_multiple, p, intensity, gs))
      return false;
    const double s = iso_value - intensity;
    dp = (gs * -s) * (1.0 / std::max(gs.dot(gs), epsilon));
    p = p + dp;
    if (dp.dot(dp) < epsilon)
    {
      surface_point = p;
      normal = gs.normalized();
      return true;
    }
  }
  return false;
  //    return p;
}

bool sampleCloud(const octomap::point3d_list& cloud, const double& spacing, const double& r_multiple,
                 const octomath::Vector3& position, double& intensity, octomath::Vector3& gradient)
{
  intensity = 0.f;
  gradient = octomath::Vector3(0, 0, 0);

  const double r = r_multiple * spacing;  // TODO magic number!
  // double T = 0.5; // TODO magic number!

  const int nn = cloud.size();
  if (nn == 0)
  {
    return false;
  }

  // variables for Wyvill
  double a = 0, b = 0, c = 0, r2 = 0, r4 = 0, r6 = 0, a1 = 0, b1 = 0, c1 = 0, a2 = 0, b2 = 0, c2 = 0;
  const bool wyvill = true;

  for (const octomath::Vector3& v : cloud)
  {
    if (wyvill)
    {
      r2 = r * r;
      r4 = r2 * r2;
      r6 = r4 * r2;
      a = -4.0 / 9.0;
      b = 17.0 / 9.0;
      c = -22.0 / 9.0;
      a1 = a / r6;
      b1 = b / r4;
      c1 = c / r2;
      a2 = 6 * a1;
      b2 = 4 * b1;
      c2 = 2 * c1;
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "This should not be called!");
    }

    double f_val = 0;
    octomath::Vector3 f_grad(0, 0, 0);

    octomath::Vector3 pos = position - v;
    const double r = pos.norm();
    pos = pos * (1.0 / r);
    if (r > r)  // must skip points outside valid bounds.
    {
      continue;
    }
    const double r2 = r * r;
    const double r3 = r * r2;
    const double r4 = r2 * r2;
    const double r5 = r3 * r2;
    const double r6 = r3 * r3;

    if (wyvill)
    {
      f_val = (a1 * r6 + b1 * r4 + c1 * r2 + 1);
      f_grad = pos * (a2 * r5 + b2 * r3 + c2 * r);
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "This should not be called!");
      const double r_scaled = r / r;
      // TODO still need to address the scaling...
      f_val = pow((1 - r_scaled), 4) * (4 * r_scaled + 1);
      f_grad = pos * (-4.0 / r * pow(1.0 - r_scaled, 3) * (4.0 * r_scaled + 1.0) + 4.0 / r * pow(1 - r_scaled, 4));
    }

    // TODO:  The whole library should be overhauled to follow the "gradient points out"
    //        convention of implicit functions.
    intensity += f_val;
    gradient += f_grad;
  }
  // implicit surface gradient convention points out, so we flip it.
  gradient *= -1.0;
  return true;  // it worked
}
