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

#include "cartesian_path_service_capability.h"
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/utils/message_checks.h>
#include <moveit/collision_detection/collision_tools.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit/move_group/capability_names.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

namespace
{
bool isStateValid(const planning_scene::PlanningScene* planning_scene,
                  const kinematic_constraints::KinematicConstraintSet* constraint_set, moveit::core::RobotState* state,
                  const moveit::core::JointModelGroup* group, const double* ik_solution)
{
  state->setJointGroupPositions(group, ik_solution);
  state->update();
  return (!planning_scene || !planning_scene->isStateColliding(*state, group->getName())) &&
         (!constraint_set || constraint_set->decide(*state).satisfied);
}
}  // namespace

namespace move_group
{
static const rclcpp::Logger LOGGER =
    rclcpp::get_logger("moveit_move_group_default_capabilities.cartersian_path_service_capability");

MoveGroupCartesianPathService::MoveGroupCartesianPathService()
  : MoveGroupCapability("CartesianPathService"), display_computed_paths_(true)
{
}

void MoveGroupCartesianPathService::initialize()
{
  display_path_ = context_->moveit_cpp_->getNode()->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      planning_pipeline::PlanningPipeline::DISPLAY_PATH_TOPIC, 10);

  cartesian_path_service_ = context_->moveit_cpp_->getNode()->create_service<moveit_msgs::srv::GetCartesianPath>(

      CARTESIAN_PATH_SERVICE_NAME,
      [this](const std::shared_ptr<rmw_request_id_t>& req_id,
             const std::shared_ptr<moveit_msgs::srv::GetCartesianPath::Request>& req,
             const std::shared_ptr<moveit_msgs::srv::GetCartesianPath::Response>& res) -> bool {
        return computeService(req_id, req, res);
      });
}

bool MoveGroupCartesianPathService::computeService(
    const std::shared_ptr<rmw_request_id_t>& /* unused */,
    const std::shared_ptr<moveit_msgs::srv::GetCartesianPath::Request>& req,
    const std::shared_ptr<moveit_msgs::srv::GetCartesianPath::Response>& res)
{
  RCLCPP_INFO(LOGGER, "Received request to compute Cartesian path");
  context_->planning_scene_monitor_->updateFrameTransforms();

  moveit::core::RobotState start_state =
      planning_scene_monitor::LockedPlanningSceneRO(context_->planning_scene_monitor_)->getCurrentState();
  moveit::core::robotStateMsgToRobotState(req->start_state, start_state);
  if (const moveit::core::JointModelGroup* jmg = start_state.getJointModelGroup(req->group_name))
  {
    std::string link_name = req->link_name;
    if (link_name.empty() && !jmg->getLinkModelNames().empty())
      link_name = jmg->getLinkModelNames().back();

    bool ok = true;
    EigenSTL::vector_Isometry3d waypoints(req->waypoints.size());
    const std::string& default_frame = context_->planning_scene_monitor_->getRobotModel()->getModelFrame();
    bool no_transform = req->header.frame_id.empty() ||
                        moveit::core::Transforms::sameFrame(req->header.frame_id, default_frame) ||
                        moveit::core::Transforms::sameFrame(req->header.frame_id, link_name);

    for (std::size_t i = 0; i < req->waypoints.size(); ++i)
    {
      if (no_transform)
        tf2::fromMsg(req->waypoints[i], waypoints[i]);
      else
      {
        geometry_msgs::msg::PoseStamped p;
        p.header = req->header;
        p.pose = req->waypoints[i];
        if (performTransform(p, default_frame))
          tf2::fromMsg(p.pose, waypoints[i]);
        else
        {
          RCLCPP_ERROR(LOGGER, "Error encountered transforming waypoints to frame '%s'", default_frame.c_str());
          ok = false;
          break;
        }
      }
    }

    if (ok)
    {
      if (req->max_step < std::numeric_limits<double>::epsilon())
      {
        RCLCPP_ERROR(LOGGER, "Maximum step to take between consecutive configurations along Cartesian path"
                             "was not specified (this value needs to be > 0)");
        res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      }
      else
      {
        if (!waypoints.empty())
        {
          moveit::core::GroupStateValidityCallbackFn constraint_fn;
          std::unique_ptr<planning_scene_monitor::LockedPlanningSceneRO> ls;
          std::unique_ptr<kinematic_constraints::KinematicConstraintSet> kset;
          if (req->avoid_collisions || !moveit::core::isEmpty(req->path_constraints))
          {
            ls = std::make_unique<planning_scene_monitor::LockedPlanningSceneRO>(context_->planning_scene_monitor_);
            kset = std::make_unique<kinematic_constraints::KinematicConstraintSet>((*ls)->getRobotModel());
            kset->add(req->path_constraints, (*ls)->getTransforms());
            constraint_fn =
                [scene = req->avoid_collisions ? static_cast<const planning_scene::PlanningSceneConstPtr&>(*ls).get() :
                                                 nullptr,
                 kset = kset->empty() ? nullptr : kset.get()](moveit::core::RobotState* robot_state,
                                                              const moveit::core::JointModelGroup* joint_group,
                                                              const double* joint_group_variable_values) {
                  return isStateValid(scene, kset, robot_state, joint_group, joint_group_variable_values);
                };
          }
          bool global_frame = !moveit::core::Transforms::sameFrame(link_name, req->header.frame_id);
          RCLCPP_INFO(LOGGER,
                      "Attempting to follow %u waypoints for link '%s' using a step of %lf m "
                      "and jump threshold %lf (in %s reference frame)",
                      (unsigned int)waypoints.size(), link_name.c_str(), req->max_step, req->jump_threshold,
                      global_frame ? "global" : "link");
          std::vector<moveit::core::RobotStatePtr> traj;
          res->fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
              &start_state, jmg, traj, start_state.getLinkModel(link_name), waypoints, global_frame,
              moveit::core::MaxEEFStep(req->max_step), moveit::core::JumpThreshold(req->jump_threshold), constraint_fn);
          moveit::core::robotStateToRobotStateMsg(start_state, res->start_state);

          robot_trajectory::RobotTrajectory rt(context_->planning_scene_monitor_->getRobotModel(), req->group_name);
          for (const moveit::core::RobotStatePtr& traj_state : traj)
            rt.addSuffixWayPoint(traj_state, 0.0);

          // time trajectory
          // \todo optionally compute timing to move the eef with constant speed
          if (req->time_optimize)
          {
            auto orig_point_count = rt.getWayPointCount();
            trajectory_processing::TimeOptimalTrajectoryGeneration time_param;
            time_param.computeTimeStamps(rt, 1.0);
            RCLCPP_INFO(LOGGER, "Optimized trajectory with %u points to %u points", (unsigned int)orig_point_count,
                        (unsigned int)rt.getWayPointCount());
          }

          rt.getRobotTrajectoryMsg(res->solution);
          RCLCPP_INFO(LOGGER, "Computed Cartesian path with %u points (followed %lf%% of requested trajectory)",
                      (unsigned int)rt.getWayPointCount(), res->fraction * 100.0);
          if (display_computed_paths_ && rt.getWayPointCount() > 0)
          {
            moveit_msgs::msg::DisplayTrajectory disp;
            disp.model_id = context_->planning_scene_monitor_->getRobotModel()->getName();
            disp.trajectory.resize(1, res->solution);
            moveit::core::robotStateToRobotStateMsg(rt.getFirstWayPoint(), disp.trajectory_start);
            display_path_->publish(disp);
          }
        }
        res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      }
    }
    else
      res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE;
  }
  else
    res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;

  return true;
}

}  // namespace move_group

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(move_group::MoveGroupCartesianPathService, move_group::MoveGroupCapability)
