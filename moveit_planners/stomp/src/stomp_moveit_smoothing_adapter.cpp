/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, PickNik Inc.
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
 *   * Neither the name of PickNik Inc. nor the names of its
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

/** @file
 * @author Henning Kayser
 * @brief: A PlanningRequestAdapter plugin for optimizing already solved trajectories with STOMP.
 */

// ROS
#include <rclcpp/rclcpp.hpp>
#include <class_loader/class_loader.hpp>

// MoveIt
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/planning_interface/planning_interface.h>

// STOMP
#include <stomp_moveit/stomp_moveit_planning_context.hpp>

using namespace planning_interface;

namespace stomp_moveit
{
/** @brief This adapter uses STOMP for optimizing pre-solved trajectories */
class StompSmoothingAdapter : public planning_request_adapter::PlanningRequestAdapter
{
public:
  StompSmoothingAdapter() : planning_request_adapter::PlanningRequestAdapter()
  {
  }

  void initialize(const rclcpp::Node::SharedPtr& node, const std::string& parameter_namespace) override
  {
    param_listener_ = std::make_shared<stomp_moveit::ParamListener>(node, parameter_namespace);
  }

  std::string getDescription() const override
  {
    return "Stomp Smoothing Adapter";
  }

  bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& ps,
                    const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
                    std::vector<std::size_t>& /*added_path_index*/) const override
  {
    // Following call to planner() calls the motion planner defined for the pipeline and stores the trajectory inside
    // the MotionPlanResponse res variable which is then passed to STOMP for optimization
    if (!planner(ps, req, res))
    {
      return false;
    }

    // STOMP reads the seed trajectory from trajectory constraints so we need to convert the waypoints first
    const size_t seed_waypoint_count = res.trajectory->getWayPointCount();
    const std::vector<std::string> joint_names =
        res.trajectory->getFirstWayPoint().getJointModelGroup(req.group_name)->getActiveJointModelNames();
    const size_t joint_count = joint_names.size();
    planning_interface::MotionPlanRequest seed_req = req;
    seed_req.trajectory_constraints.constraints.clear();
    seed_req.trajectory_constraints.constraints.resize(seed_waypoint_count);
    for (size_t i = 0; i < seed_waypoint_count; ++i)
    {
      seed_req.trajectory_constraints.constraints[i].joint_constraints.resize(joint_count);
      for (size_t j = 0; j < joint_count; ++j)
      {
        seed_req.trajectory_constraints.constraints[i].joint_constraints[j].joint_name = joint_names[j];
        seed_req.trajectory_constraints.constraints[i].joint_constraints[j].position =
            res.trajectory->getWayPoint(i).getVariablePosition(joint_names[j]);
      }
    }

    // Initialize STOMP Planner
    PlanningContextPtr planning_context =
        std::make_shared<StompPlanningContext>("STOMP", req.group_name, param_listener_->get_params());
    planning_context->clear();
    planning_context->setPlanningScene(ps);
    planning_context->setMotionPlanRequest(seed_req);

    // Solve
    RCLCPP_INFO(rclcpp::get_logger("stomp_moveit"), "Smoothing result trajectory with STOMP");
    planning_interface::MotionPlanResponse stomp_res;
    bool success = planning_context->solve(stomp_res);
    if (success)
    {
      res.trajectory = stomp_res.trajectory;
      res.planning_time += stomp_res.planning_time;
    }
    res.error_code = stomp_res.error_code;
    return success;
  }

private:
  std::shared_ptr<stomp_moveit::ParamListener> param_listener_;
};
}  // namespace stomp_moveit

CLASS_LOADER_REGISTER_CLASS(stomp_moveit::StompSmoothingAdapter, planning_request_adapter::PlanningRequestAdapter);
