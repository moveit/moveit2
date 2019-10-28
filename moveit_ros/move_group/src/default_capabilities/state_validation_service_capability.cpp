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

/* Author: Ioan Sucan */

#include "state_validation_service_capability.h"
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/move_group/capability_names.h>

move_group::MoveGroupStateValidationService::MoveGroupStateValidationService()
  : MoveGroupCapability("StateValidationService")
{
}

void move_group::MoveGroupStateValidationService::initialize(std::shared_ptr<rclcpp::Node>& node)
{
  this->node_ = node;
  validity_service_ = node_->create_service<moveit_msgs::srv::GetStateValidity>(
        STATE_VALIDITY_SERVICE_NAME, std::bind(&MoveGroupStateValidationService::computePlanService, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3) );
}

void move_group::MoveGroupStateValidationService::computePlanService(const std::shared_ptr<rmw_request_id_t> request_header,
     const std::shared_ptr<moveit_msgs::srv::GetStateValidity::Request> request,
     const std::shared_ptr<moveit_msgs::srv::GetStateValidity::Response> response)
{
  planning_scene_monitor::LockedPlanningSceneRO ls(context_->planning_scene_monitor_);
  robot_state::RobotState rs = ls->getCurrentState();
  robot_state::robotStateMsgToRobotState(request->robot_state, rs);

  response->valid = true;

  // configure collision request
  collision_detection::CollisionRequest creq;
  creq.group_name = request->group_name;
  creq.cost = true;
  creq.contacts = true;
  creq.max_contacts = ls->getWorld()->size();
  creq.max_cost_sources = creq.max_contacts + ls->getRobotModel()->getLinkModelsWithCollisionGeometry().size();
  creq.max_contacts *= creq.max_contacts;
  collision_detection::CollisionResult cres;

  // check collision
  ls->checkCollision(creq, cres, rs);

  // copy contacts if any
  if (cres.collision)
  {
    auto time_now = rclcpp::Clock().now();
    response->contacts.reserve(cres.contact_count);
    response->valid = false;
    for (collision_detection::CollisionResult::ContactMap::const_iterator it = cres.contacts.begin();
         it != cres.contacts.end(); ++it)
      for (std::size_t k = 0; k < it->second.size(); ++k)
      {
        response->contacts.resize(response->contacts.size() + 1);
        collision_detection::contactToMsg(it->second[k], response->contacts.back());
        response->contacts.back().header.frame_id = ls->getPlanningFrame();
        response->contacts.back().header.stamp = time_now;
      }
  }

  // copy cost sources
  response->cost_sources.reserve(cres.cost_sources.size());
  for (std::set<collision_detection::CostSource>::const_iterator it = cres.cost_sources.begin();
       it != cres.cost_sources.end(); ++it)
  {
    response->cost_sources.resize(response->cost_sources.size() + 1);
    collision_detection::costSourceToMsg(*it, response->cost_sources.back());
  }

  // evaluate constraints
  if (!kinematic_constraints::isEmpty(request->constraints))
  {
    kinematic_constraints::KinematicConstraintSet kset(ls->getRobotModel());
    kset.add(request->constraints, ls->getTransforms());
    std::vector<kinematic_constraints::ConstraintEvaluationResult> kres;
    kinematic_constraints::ConstraintEvaluationResult total_result = kset.decide(rs, kres);
    if (!total_result.satisfied)
      response->valid = false;

    // copy constraint results
    response->constraint_result.resize(kres.size());
    for (std::size_t k = 0; k < kres.size(); ++k)
    {
      response->constraint_result[k].result = kres[k].satisfied;
      response->constraint_result[k].distance = kres[k].distance;
    }
  }

  return;
}

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupStateValidationService, move_group::MoveGroupCapability)
