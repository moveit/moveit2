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
   Desc: Functions to create and use a map of PlanningPipelines to solve MotionPlanRequests */

#pragma once

#include <moveit/planning_pipeline_interfaces/plan_responses_container.hpp>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_interface/planning_request.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>

namespace moveit
{
namespace planning_pipeline_interfaces
{
/** \brief A stopping criterion callback function for the parallel planning API of planning component
 * \param [in] plan_responses_container Container with responses to be taken into account for the stopping decision
 * \param [in] plan_requests Motion plan requests for the parallel planner
 * \return True to stop parallel planning and abort planning pipelines that haven't terminated by now
 */
typedef std::function<bool(const PlanResponsesContainer& plan_responses_container,
                           const std::vector<::planning_interface::MotionPlanRequest>& plan_requests)>
    StoppingCriterionFunction;

/** \brief A solution callback function type for the parallel planning API of planning component
 * \param [in] solutions Motion plan responses to choose from
 * \return Selected motion plan response
 */
typedef std::function<::planning_interface::MotionPlanResponse(
    const std::vector<::planning_interface::MotionPlanResponse>& solutions)>
    SolutionSelectionFunction;

/** \brief Function to calculate the MotionPlanResponse for a given MotionPlanRequest and a PlanningScene
 * \param [in] motion_plan_request Motion planning problem to be solved
 * \param [in] planning_scene Planning scene for which the given planning problem needs to be solved
 * \param [in] planning_pipelines Pipelines available to solve the problem, if the requested pipeline is not provided
 * the MotionPlanResponse will be FAILURE \return MotionPlanResponse for the given planning problem
 */
::planning_interface::MotionPlanResponse planWithSinglePipeline(
    const ::planning_interface::MotionPlanRequest& motion_plan_request,
    const ::planning_scene::PlanningSceneConstPtr& planning_scene,
    const std::unordered_map<std::string, planning_pipeline::PlanningPipelinePtr>& planning_pipelines);

/** \brief Function to solve multiple planning problems in parallel threads with multiple planning pipelines at the same
 time
 * \param [in] motion_plan_request Motion planning problems to be solved
 * \param [in] planning_scene Planning scene for which the given planning problem needs to be solved
 * \param [in] planning_pipelines Pipelines available to solve the problems, if a requested pipeline is not provided the
 MotionPlanResponse will be FAILURE
 * \param [in] stopping_criterion_callback If this function returns true, the planning pipelines that are still running
 will be terminated and the existing solutions will be evaluated. If no callback is provided, all planning pipelines
 terminate after the max. planning time defined in the MotionPlanningRequest is reached.
 * \param [in] solution_selection_function Function to select a specific solution out of all available solution. If no
 function is provided, all solutions are returned.
 + \return If a solution_selection_function is provided a vector containing the selected response is returned, otherwise
 the vector contains all solutions produced.
*/
const std::vector<::planning_interface::MotionPlanResponse> planWithParallelPipelines(
    const std::vector<::planning_interface::MotionPlanRequest>& motion_plan_requests,
    const ::planning_scene::PlanningSceneConstPtr& planning_scene,
    const std::unordered_map<std::string, planning_pipeline::PlanningPipelinePtr>& planning_pipelines,
    const StoppingCriterionFunction& stopping_criterion_callback = nullptr,
    const SolutionSelectionFunction& solution_selection_function = nullptr);

/** \brief Utility function to create a map of named planning pipelines
 * \param [in] pipeline_names Vector of planning pipeline names to be used. Each name is also the namespace from which
 * the pipeline parameters are loaded
 * \param [in] robot_model Robot model used to initialize the pipelines
 * \param [in] node Node used to load parameters
 * \param [in] parameter_namespace Optional prefix for the pipeline parameter
 * namespace. Empty by default, so only the pipeline name is used as namespace
 * \return Map of PlanningPipelinePtr's associated with a name for faster look-up
 */
std::unordered_map<std::string, planning_pipeline::PlanningPipelinePtr>
createPlanningPipelineMap(const std::vector<std::string>& pipeline_names,
                          const moveit::core::RobotModelConstPtr& robot_model, const rclcpp::Node::SharedPtr& node,
                          const std::string& parameter_namespace = std::string());
}  // namespace planning_pipeline_interfaces
}  // namespace moveit
