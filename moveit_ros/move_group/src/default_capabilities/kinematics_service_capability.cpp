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

#include "kinematics_service_capability.h"
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit/move_group/capability_names.h>

move_group::MoveGroupKinematicsService::MoveGroupKinematicsService() : MoveGroupCapability("KinematicsService")
{
}

void move_group::MoveGroupKinematicsService::initialize(std::shared_ptr<rclcpp::Node>& node)
{
  this->node_ = node;
  fk_service_ = node_->create_service<moveit_msgs::srv::GetPositionFK>(
        FK_SERVICE_NAME, std::bind(&MoveGroupKinematicsService::computeFKService, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3) );
  ik_service_ = node_->create_service<moveit_msgs::srv::GetPositionIK>(
        IK_SERVICE_NAME, std::bind(&MoveGroupKinematicsService::computeIKService, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3) );
}

namespace
{
bool isIKSolutionValid(const planning_scene::PlanningScene* planning_scene,
                       const kinematic_constraints::KinematicConstraintSet* constraint_set,
                       robot_state::RobotState* state, const robot_model::JointModelGroup* jmg,
                       const double* ik_solution)
{
  state->setJointGroupPositions(jmg, ik_solution);
  state->update();
  return (!planning_scene || !planning_scene->isStateColliding(*state, jmg->getName())) &&
         (!constraint_set || constraint_set->decide(*state).satisfied);
}
}  // namespace

void move_group::MoveGroupKinematicsService::computeIK(
    moveit_msgs::msg::PositionIKRequest& req, moveit_msgs::msg::RobotState& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
    robot_state::RobotState& rs, const robot_state::GroupStateValidityCallbackFn& constraint) const
{
  const robot_state::JointModelGroup* jmg = rs.getJointModelGroup(req.group_name);
  if (jmg)
  {
    robot_state::robotStateMsgToRobotState(req.robot_state, rs);
    const std::string& default_frame = context_->planning_scene_monitor_->getRobotModel()->getModelFrame();

    if (req.pose_stamped_vector.empty() || req.pose_stamped_vector.size() == 1)
    {
      geometry_msgs::msg::PoseStamped req_pose =
          req.pose_stamped_vector.empty() ? req.pose_stamped : req.pose_stamped_vector[0];
      std::string ik_link = (!req.pose_stamped_vector.empty()) ?
                                (req.ik_link_names.empty() ? "" : req.ik_link_names[0]) :
                                req.ik_link_name;

      if (performTransform(req_pose, default_frame))
      {
        bool result_ik = false;
        double timeout = ((double)req.timeout.sec) + ((double)RCUTILS_NS_TO_S(req.timeout.nanosec));
        if (ik_link.empty())
          result_ik = rs.setFromIK(jmg, req_pose.pose, timeout, constraint);
        else
          result_ik = rs.setFromIK(jmg, req_pose.pose, ik_link, timeout, constraint);

        if (result_ik)
        {
          robot_state::robotStateToRobotStateMsg(rs, solution, false);
          error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
        }
        else
          error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
      }
      else
        error_code.val = moveit_msgs::msg::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE;
    }
    else
    {
      if (req.pose_stamped_vector.size() != req.ik_link_names.size())
        error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_LINK_NAME;
      else
      {
        bool ok = true;
        EigenSTL::vector_Isometry3d req_poses(req.pose_stamped_vector.size());
        for (std::size_t k = 0; k < req.pose_stamped_vector.size(); ++k)
        {
          geometry_msgs::msg::PoseStamped msg = req.pose_stamped_vector[k];
          if (performTransform(msg, default_frame))
            tf2::fromMsg(msg.pose, req_poses[k]);
          else
          {
            error_code.val = moveit_msgs::msg::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE;
            ok = false;
            break;
          }
        }
        if (ok)
        {
          double timeout = ((double)req.timeout.sec) + ((double)RCUTILS_NS_TO_S(req.timeout.nanosec));
          if (rs.setFromIK(jmg, req_poses, req.ik_link_names, timeout, constraint))
          {
            robot_state::robotStateToRobotStateMsg(rs, solution, false);
            error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
          }
          else
            error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
        }
      }
    }
  }
  else
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
}

void move_group::MoveGroupKinematicsService::computeIKService(const std::shared_ptr<rmw_request_id_t> request_header,
     const std::shared_ptr<moveit_msgs::srv::GetPositionIK::Request> request,
     const std::shared_ptr<moveit_msgs::srv::GetPositionIK::Response> response)
{
  context_->planning_scene_monitor_->updateFrameTransforms();

  // check if the planning scene needs to be kept locked; if so, call computeIK() in the scope of the lock
  if (request->ik_request.avoid_collisions || !kinematic_constraints::isEmpty(request->ik_request.constraints))
  {
    planning_scene_monitor::LockedPlanningSceneRO ls(context_->planning_scene_monitor_);
    kinematic_constraints::KinematicConstraintSet kset(ls->getRobotModel());
    robot_state::RobotState rs = ls->getCurrentState();
    kset.add(request->ik_request.constraints, ls->getTransforms());
    computeIK(request->ik_request, response->solution, response->error_code, rs,
              boost::bind(&isIKSolutionValid, request->ik_request.avoid_collisions ?
                                                  static_cast<const planning_scene::PlanningSceneConstPtr&>(ls).get() :
                                                  nullptr,
                          kset.empty() ? nullptr : &kset, _1, _2, _3));
  }
  else
  {
    // compute unconstrained IK, no lock to planning scene maintained
    robot_state::RobotState rs =
        planning_scene_monitor::LockedPlanningSceneRO(context_->planning_scene_monitor_)->getCurrentState();
    computeIK(request->ik_request, response->solution, response->error_code, rs);
  }
}

void move_group::MoveGroupKinematicsService::computeFKService(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<moveit_msgs::srv::GetPositionFK::Request> request,
    const std::shared_ptr<moveit_msgs::srv::GetPositionFK::Response> response)
{
  if (request->fk_link_names.empty())
  {
    RCLCPP_ERROR(rclcpp::get_logger("MoveGroupKinematicsService"),"No links specified for FK request");
    response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_LINK_NAME;
    return;
  }

  context_->planning_scene_monitor_->updateFrameTransforms();

  const std::string& default_frame = context_->planning_scene_monitor_->getRobotModel()->getModelFrame();
  bool do_transform = !request->header.frame_id.empty() &&
                      !robot_state::Transforms::sameFrame(request->header.frame_id, default_frame) &&
                      context_->planning_scene_monitor_->getTFClient();
  bool tf_problem = false;

  robot_state::RobotState rs =
      planning_scene_monitor::LockedPlanningSceneRO(context_->planning_scene_monitor_)->getCurrentState();
  robot_state::robotStateMsgToRobotState(request->robot_state, rs);
  for (std::size_t i = 0; i < request->fk_link_names.size(); ++i)
    if (rs.getRobotModel()->hasLinkModel(request->fk_link_names[i]))
    {
      response->pose_stamped.resize(response->pose_stamped.size() + 1);
      response->pose_stamped.back().pose = tf2::toMsg(rs.getGlobalLinkTransform(request->fk_link_names[i]));
      response->pose_stamped.back().header.frame_id = default_frame;
      response->pose_stamped.back().header.stamp = rclcpp::Clock().now();
      if (do_transform)
        if (!performTransform(response->pose_stamped.back(), request->header.frame_id))
          tf_problem = true;
      response->fk_link_names.push_back(request->fk_link_names[i]);
    }
  if (tf_problem)
    response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE;
  else if (response->fk_link_names.size() == request->fk_link_names.size())
    response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  else
    response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_LINK_NAME;
  return;
}

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupKinematicsService, move_group::MoveGroupCapability)
