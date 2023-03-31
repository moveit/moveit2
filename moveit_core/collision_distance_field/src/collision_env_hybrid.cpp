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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author: E. Gil Jones, Jens Petit */

#include <moveit/collision_distance_field/collision_detector_allocator_hybrid.h>
#include <moveit/collision_distance_field/collision_env_hybrid.h>

namespace collision_detection
{
const std::string collision_detection::CollisionDetectorAllocatorHybrid::NAME("HYBRID");

CollisionEnvHybrid::CollisionEnvHybrid(
    const moveit::core::RobotModelConstPtr& robot_model,
    const std::map<std::string, std::vector<CollisionSphere>>& link_body_decompositions, double size_x, double size_y,
    double size_z, const Eigen::Vector3d& origin, bool use_signed_distance_field, double resolution,
    double collision_tolerance, double max_propogation_distance, double padding, double scale)
  : CollisionEnvFCL(robot_model)
  , cenv_distance_(std::make_shared<collision_detection::CollisionEnvDistanceField>(
        robot_model, getWorld(), link_body_decompositions, size_x, size_y, size_z, origin, use_signed_distance_field,
        resolution, collision_tolerance, max_propogation_distance, padding, scale))
{
}

CollisionEnvHybrid::CollisionEnvHybrid(
    const moveit::core::RobotModelConstPtr& robot_model, const WorldPtr& world,
    const std::map<std::string, std::vector<CollisionSphere>>& link_body_decompositions, double size_x, double size_y,
    double size_z, const Eigen::Vector3d& origin, bool use_signed_distance_field, double resolution,
    double collision_tolerance, double max_propogation_distance, double padding, double scale)
  : CollisionEnvFCL(robot_model, world, padding, scale)
  , cenv_distance_(std::make_shared<collision_detection::CollisionEnvDistanceField>(
        robot_model, getWorld(), link_body_decompositions, size_x, size_y, size_z, origin, use_signed_distance_field,
        resolution, collision_tolerance, max_propogation_distance, padding, scale))
{
}

CollisionEnvHybrid::CollisionEnvHybrid(const CollisionEnvHybrid& other, const WorldPtr& world)
  : CollisionEnvFCL(other, world)
  , cenv_distance_(std::make_shared<collision_detection::CollisionEnvDistanceField>(
        *other.getCollisionWorldDistanceField(), world))
{
}

void CollisionEnvHybrid::checkSelfCollisionDistanceField(const collision_detection::CollisionRequest& req,
                                                         collision_detection::CollisionResult& res,
                                                         const moveit::core::RobotState& state) const
{
  cenv_distance_->checkSelfCollision(req, res, state);
}

void CollisionEnvHybrid::checkSelfCollisionDistanceField(const collision_detection::CollisionRequest& req,
                                                         collision_detection::CollisionResult& res,
                                                         const moveit::core::RobotState& state,
                                                         GroupStateRepresentationPtr& gsr) const
{
  cenv_distance_->checkSelfCollision(req, res, state, gsr);
}

void CollisionEnvHybrid::checkSelfCollisionDistanceField(const collision_detection::CollisionRequest& req,
                                                         collision_detection::CollisionResult& res,
                                                         const moveit::core::RobotState& state,
                                                         const collision_detection::AllowedCollisionMatrix& acm) const
{
  cenv_distance_->checkSelfCollision(req, res, state, acm);
}
    
bool CollisionEnvHybrid::checkSelfCollisionDistanceField(const collision_detection::CollisionRequest& req,
                                                         collision_detection::CollisionResult& res,
                                                         const moveit::core::RobotState& state) const
{
  ROS_DEBUG_STREAM_NAMED("collision_detection.distance_field", "Starting self collision check using distance field");

  const std::vector<const collision_detection::DistanceField*>& dfs = getAllDistanceFields();
  std::vector<std::pair<int, int>> self_collisions;

  for (std::size_t i = 0; i < dfs.size(); ++i)
  {
    if (dfs[i]->getGroupMask() & req.group_mask)
    {
      std::vector<std::pair<int, int>> contacts;
      dfs[i]->checkSelfCollision(req, contacts);

      if (!contacts.empty())
      {
        for (std::size_t j = 0; j < contacts.size(); ++j)
        {
          int id1 = dfs[i]->getLinkModel(contacts[j].first)->getParentLinkModel()->getLinkIndex();
          int id2 = dfs[i]->getLinkModel(contacts[j].second)->getParentLinkModel()->getLinkIndex();
          self_collisions.push_back(std::make_pair(std::max(id1, id2), std::min(id1, id2)));
        }
      }
    }
  }

  // Remove duplicates
  std::sort(self_collisions.begin(), self_collisions.end());
  std::vector<std::pair<int, int>>::iterator end = std::unique(self_collisions.begin(), self_collisions.end());

  for (std::vector<std::pair<int, int>>::iterator it = self_collisions.begin(); it != end; ++it)
  {
    collision_detection::CollisionResult::Contact con;
    con.body_name_1 = state.getRobotModel()->getLinkModel(it->first)->getName();
    con.body_name_2 = state.getRobotModel()->getLinkModel(it->second)->getName();
    res.contacts.push_back(con);
  }

  return res.collision;
}

