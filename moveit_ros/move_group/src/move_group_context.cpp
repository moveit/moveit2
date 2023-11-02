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

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/utils/logger.hpp>

namespace move_group
{
namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("MoveGroupContext");
}
}  // namespace

MoveGroupContext::MoveGroupContext(const moveit_cpp::MoveItCppPtr& moveit_cpp,
                                   const std::string& default_planning_pipeline, bool allow_trajectory_execution,
                                   bool debug)
  : moveit_cpp_(moveit_cpp)
  , planning_scene_monitor_(moveit_cpp->getPlanningSceneMonitorNonConst())
  , allow_trajectory_execution_(allow_trajectory_execution)
  , debug_(debug)
{
  // Check if default planning pipeline has been initialized successfully
  const auto& pipelines = moveit_cpp->getPlanningPipelines();
  const auto default_pipeline_it = pipelines.find(default_planning_pipeline);
  if (default_pipeline_it != pipelines.end())
  {
    planning_pipeline_ = default_pipeline_it->second;
  }
  else
  {
    RCLCPP_ERROR(
        getLogger(),
        "Failed to find default PlanningPipeline '%s' - please check MoveGroup's planning pipeline configuration.",
        default_planning_pipeline.c_str());
  }

  if (allow_trajectory_execution_)
  {
    trajectory_execution_manager_ = moveit_cpp_->getTrajectoryExecutionManagerNonConst();
    plan_execution_ = std::make_shared<plan_execution::PlanExecution>(moveit_cpp_->getNode(), planning_scene_monitor_,
                                                                      trajectory_execution_manager_);
  }
}

MoveGroupContext::~MoveGroupContext()
{
  plan_execution_.reset();
  trajectory_execution_manager_.reset();
  planning_pipeline_.reset();
  planning_scene_monitor_.reset();
}

bool MoveGroupContext::status() const
{
  const planning_interface::PlannerManagerPtr& planner_interface = planning_pipeline_->getPlannerManager();
  if (planner_interface)
  {
    RCLCPP_INFO_STREAM(getLogger(),
                       "MoveGroup context using planning plugin " << planning_pipeline_->getPlannerPluginName());
    RCLCPP_INFO_STREAM(getLogger(), "MoveGroup context initialization complete");
    return true;
  }
  else
  {
    RCLCPP_WARN_STREAM(getLogger(),
                       "MoveGroup running was unable to load " << planning_pipeline_->getPlannerPluginName());
    return false;
  }
}

}  // namespace move_group
