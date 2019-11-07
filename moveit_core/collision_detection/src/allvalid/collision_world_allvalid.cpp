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

#include <moveit/collision_detection/allvalid/collision_world_allvalid.h>
#include "rclcpp/rclcpp.hpp"

namespace collision_detection
{
// Logger
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_collision_detection.collision_world_allvalid");

CollisionWorldAllValid::CollisionWorldAllValid() : CollisionWorld()
{
}

CollisionWorldAllValid::CollisionWorldAllValid(const WorldPtr& world) : CollisionWorld(world)
{
}

CollisionWorldAllValid::CollisionWorldAllValid(const CollisionWorld& other, const WorldPtr& world)
  : CollisionWorld(other, world)
{
}

void CollisionWorldAllValid::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                                 const CollisionRobot& robot,
                                                 const robot_state::RobotState& state) const
{
  res.collision = false;
  if (req.verbose)
    RCLCPP_INFO(LOGGER, "Using AllValid collision detection. No collision checking is performed.");
}

void CollisionWorldAllValid::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                                 const CollisionRobot& robot, const robot_state::RobotState& state,
                                                 const AllowedCollisionMatrix& acm) const
{
  res.collision = false;
  if (req.verbose)
    RCLCPP_INFO(LOGGER, "Using AllValid collision detection. No collision checking is performed.");
}

void CollisionWorldAllValid::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                                 const CollisionRobot& robot, const robot_state::RobotState& state1,
                                                 const robot_state::RobotState& state2) const
{
  res.collision = false;
  if (req.verbose)
    RCLCPP_INFO(LOGGER, "Using AllValid collision detection. No collision checking is performed.");
}

void CollisionWorldAllValid::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                                 const CollisionRobot& robot, const robot_state::RobotState& state1,
                                                 const robot_state::RobotState& state2,
                                                 const AllowedCollisionMatrix& acm) const
{
  res.collision = false;
  if (req.verbose)
    RCLCPP_INFO(LOGGER, "Using AllValid collision detection. No collision checking is performed.");
}

void CollisionWorldAllValid::checkWorldCollision(const CollisionRequest& req, CollisionResult& res,
                                                 const CollisionWorld& other_world) const
{
  res.collision = false;
  if (req.verbose)
    RCLCPP_INFO(LOGGER, "Using AllValid collision detection. No collision checking is performed.");
}

void CollisionWorldAllValid::checkWorldCollision(const CollisionRequest& req, CollisionResult& res,
                                                 const CollisionWorld& other_world,
                                                 const AllowedCollisionMatrix& acm) const
{
  res.collision = false;
  if (req.verbose)
    RCLCPP_INFO(LOGGER, "Using AllValid collision detection. No collision checking is performed.");
}

double CollisionWorldAllValid::distanceRobot(const CollisionRobot& robot, const robot_state::RobotState& state) const
{
  return 0.0;
}

double CollisionWorldAllValid::distanceRobot(const CollisionRobot& robot, const robot_state::RobotState& state,
                                             const AllowedCollisionMatrix& acm) const
{
  return 0.0;
}

void CollisionWorldAllValid::distanceRobot(const DistanceRequest& req, DistanceResult& res, const CollisionRobot& robot,
                                           const moveit::core::RobotState& state) const
{
  res.collision = false;
}

double CollisionWorldAllValid::distanceWorld(const CollisionWorld& world) const
{
  return 0.0;
}

double CollisionWorldAllValid::distanceWorld(const CollisionWorld& world, const AllowedCollisionMatrix& acm) const
{
  return 0.0;
}

void CollisionWorldAllValid::distanceWorld(const DistanceRequest& req, DistanceResult& res,
                                           const CollisionWorld& world) const
{
  res.collision = false;
}
}  // namespace collision_detection

#include <moveit/collision_detection/allvalid/collision_detector_allocator_allvalid.h>
const std::string collision_detection::CollisionDetectorAllocatorAllValid::NAME("ALL_VALID");
