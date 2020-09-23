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

#include <moveit/move_group/move_group_context.h>

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/plan_execution/plan_with_sensing.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_move_group_capabilities_base.move_group_context");

move_group::MoveGroupContext::MoveGroupContext(
    const rclcpp::Node::SharedPtr& node, const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
    bool allow_trajectory_execution, bool debug)
  : node_(node)
  , planning_scene_monitor_(planning_scene_monitor)
  , allow_trajectory_execution_(allow_trajectory_execution)
  , debug_(debug)
{
  planning_pipeline_.reset(
      new planning_pipeline::PlanningPipeline(planning_scene_monitor_->getRobotModel(), node, "move_group"));

  if (allow_trajectory_execution_)
  {
    trajectory_execution_manager_.reset(new trajectory_execution_manager::TrajectoryExecutionManager(
        node_, planning_scene_monitor_->getRobotModel(), planning_scene_monitor_->getStateMonitor()));

    plan_execution_.reset(
        new plan_execution::PlanExecution(node_, planning_scene_monitor_, trajectory_execution_manager_));
    plan_with_sensing_.reset(new plan_execution::PlanWithSensing(node_, trajectory_execution_manager_));

    if (debug)
      plan_with_sensing_->displayCostSources(true);
  }

  // configure the planning pipeline
  planning_pipeline_->displayComputedMotionPlans(true);
  planning_pipeline_->checkSolutionPaths(true);

  if (debug_)
    planning_pipeline_->publishReceivedRequests(true);
}

move_group::MoveGroupContext::~MoveGroupContext()
{
  plan_with_sensing_.reset();
  plan_execution_.reset();
  trajectory_execution_manager_.reset();
  planning_pipeline_.reset();
  planning_scene_monitor_.reset();
}

bool move_group::MoveGroupContext::status() const
{
  const planning_interface::PlannerManagerPtr& planner_interface = planning_pipeline_->getPlannerManager();
  if (planner_interface)
  {
    RCLCPP_INFO_STREAM(LOGGER, "MoveGroup context using planning plugin " << planning_pipeline_->getPlannerPluginName());
    RCLCPP_INFO_STREAM(LOGGER, "MoveGroup context initialization complete");
    return true;
  }
  else
  {
    RCLCPP_WARN_STREAM(LOGGER, "MoveGroup running was unable to load " << planning_pipeline_->getPlannerPluginName());
    return false;
  }
}
