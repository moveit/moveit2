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

/* Author: Sebastian Jahr
   Desc: TODO */

#pragma once

#include <moveit/pipeline_planning_interface/plan_responses_container.hpp>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_interface/planning_request.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>

namespace moveit
{
namespace planning_interface
{
/** \brief A stopping criterion callback function for the parallel planning API of planning component */
typedef std::function<bool(const PlanResponsesContainer& plan_responses_container,
                           const std::vector<::planning_interface::MotionPlanRequest>& plan_requests)>
    StoppingCriterionFunction;

/** \brief A solution callback function type for the parallel planning API of planning component  */
typedef std::function<::planning_interface::MotionPlanResponse(
    const std::vector<::planning_interface::MotionPlanResponse>& solutions)>
    SolutionSelectionFunction;

::planning_interface::MotionPlanResponse
planWithSinglePipeline(const ::planning_interface::MotionPlanRequest& motion_plan_requests,
                       ::planning_scene::PlanningSceneConstPtr planning_scene,
                       const std::map<std::string, planning_pipeline::PlanningPipelinePtr>& planning_pipelines);

const std::vector<::planning_interface::MotionPlanResponse>
planWithParallelPipelines(const std::vector<::planning_interface::MotionPlanRequest>& motion_plan_requests,
                          ::planning_scene::PlanningSceneConstPtr planning_scene,
                          const std::map<std::string, planning_pipeline::PlanningPipelinePtr>& planning_pipelines,
                          StoppingCriterionFunction stopping_criterion_callback = nullptr,
                          SolutionSelectionFunction solution_selection_function = nullptr);
}  // namespace planning_interface
}  // namespace moveit
