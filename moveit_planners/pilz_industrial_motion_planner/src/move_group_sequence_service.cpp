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

// Modified by Pilz GmbH & Co. KG

#include <pilz_industrial_motion_planner/move_group_sequence_service.h>

#include <pilz_industrial_motion_planner/capability_names.h>
#include <pilz_industrial_motion_planner/command_list_manager.h>
#include <pilz_industrial_motion_planner/trajectory_generation_exceptions.h>

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/utils/logger.hpp>

namespace pilz_industrial_motion_planner
{
namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.planners.pilz.move_group_sequence_service");
}
}  // namespace
MoveGroupSequenceService::MoveGroupSequenceService() : MoveGroupCapability("SequenceService")
{
}

MoveGroupSequenceService::~MoveGroupSequenceService()
{
}

void MoveGroupSequenceService::initialize()
{
  command_list_manager_ = std::make_unique<pilz_industrial_motion_planner::CommandListManager>(
      context_->moveit_cpp_->getNode(), context_->planning_scene_monitor_->getRobotModel());

  sequence_service_ = context_->moveit_cpp_->getNode()->create_service<moveit_msgs::srv::GetMotionSequence>(
      SEQUENCE_SERVICE_NAME,
      [this](const moveit_msgs::srv::GetMotionSequence::Request::SharedPtr& req,
             const moveit_msgs::srv::GetMotionSequence::Response::SharedPtr& res) { return plan(req, res); });
}

bool MoveGroupSequenceService::plan(const moveit_msgs::srv::GetMotionSequence::Request::SharedPtr& req,
                                    const moveit_msgs::srv::GetMotionSequence::Response::SharedPtr& res)
{
  // Handle empty requests
  if (req->request.items.empty())
  {
    RCLCPP_WARN(getLogger(), "Received empty request. That's ok but maybe not what you intended.");
    res->response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    return true;
  }

  // TODO: Do we lock on the correct scene? Does the lock belong to the scene
  // used for planning?
  planning_scene_monitor::LockedPlanningSceneRO ps(context_->planning_scene_monitor_);

  rclcpp::Time planning_start = context_->moveit_cpp_->getNode()->now();
  RobotTrajCont traj_vec;
  try
  {
    // Select planning_pipeline to handle request
    // All motions in the SequenceRequest need to use the same planning pipeline (but can use different planners)
    const planning_pipeline::PlanningPipelinePtr planning_pipeline =
        resolvePlanningPipeline(req->request.items[0].req.pipeline_id);
    if (!planning_pipeline)
    {
      RCLCPP_ERROR_STREAM(getLogger(), "Could not load planning pipeline " << req->request.items[0].req.pipeline_id);
      res->response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      return false;
    }

    traj_vec = command_list_manager_->solve(ps, context_->planning_pipeline_, req->request);
  }
  catch (const MoveItErrorCodeException& ex)
  {
    RCLCPP_ERROR_STREAM(getLogger(),
                        "Planner threw an exception (error code: " << ex.getErrorCode() << "): " << ex.what());
    res->response.error_code.val = ex.getErrorCode();
    return true;
  }
  // LCOV_EXCL_START // Keep moveit up even if lower parts throw
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR_STREAM(getLogger(), "Planner threw an exception: " << ex.what());
    // If 'FALSE' then no response will be sent to the caller.
    return false;
  }
  // LCOV_EXCL_STOP

  res->response.planned_trajectories.resize(traj_vec.size());
  for (RobotTrajCont::size_type i = 0; i < traj_vec.size(); ++i)
  {
    move_group::MoveGroupCapability::convertToMsg(traj_vec.at(i), res->response.sequence_start,
                                                  res->response.planned_trajectories.at(i));
  }
  res->response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  res->response.planning_time = (context_->moveit_cpp_->getNode()->now() - planning_start).seconds();
  return true;
}

}  // namespace pilz_industrial_motion_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pilz_industrial_motion_planner::MoveGroupSequenceService, move_group::MoveGroupCapability)
