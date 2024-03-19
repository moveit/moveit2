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

/* Author: Sebastian Jahr */

#include <moveit/planning_pipeline_interfaces/planning_pipeline_interfaces.hpp>
#include <moveit/utils/logger.hpp>

#include <thread>

namespace moveit
{
namespace planning_pipeline_interfaces
{

rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.ros.planning_pipeline_interfaces");
}

::planning_interface::MotionPlanResponse
planWithSinglePipeline(const ::planning_interface::MotionPlanRequest& motion_plan_request,
                       const ::planning_scene::PlanningSceneConstPtr& planning_scene,
                       const std::unordered_map<std::string, planning_pipeline::PlanningPipelinePtr>& planning_pipelines)
{
  ::planning_interface::MotionPlanResponse motion_plan_response;
  auto it = planning_pipelines.find(motion_plan_request.pipeline_id);
  if (it == planning_pipelines.end())
  {
    RCLCPP_ERROR(getLogger(), "No planning pipeline available for name '%s'", motion_plan_request.pipeline_id.c_str());
    motion_plan_response.error_code = moveit::core::MoveItErrorCode::FAILURE;
    return motion_plan_response;
  }
  const planning_pipeline::PlanningPipelinePtr pipeline = it->second;
  if (!pipeline->generatePlan(planning_scene, motion_plan_request, motion_plan_response))
  {
    if ((motion_plan_response.error_code.val == moveit::core::MoveItErrorCode::SUCCESS) ||
        (motion_plan_response.error_code.val == moveit::core::MoveItErrorCode::UNDEFINED))
    {
      RCLCPP_ERROR(getLogger(), "Planning pipeline '%s' failed to plan, but did not set an error code",
                   motion_plan_request.pipeline_id.c_str());
      motion_plan_response.error_code = moveit::core::MoveItErrorCode::FAILURE;
    }
  }
  return motion_plan_response;
}

const std::vector<::planning_interface::MotionPlanResponse> planWithParallelPipelines(
    const std::vector<::planning_interface::MotionPlanRequest>& motion_plan_requests,
    const ::planning_scene::PlanningSceneConstPtr& planning_scene,
    const std::unordered_map<std::string, planning_pipeline::PlanningPipelinePtr>& planning_pipelines,
    const StoppingCriterionFunction& stopping_criterion_callback,
    const SolutionSelectionFunction& solution_selection_function)
{
  // Create solutions container
  PlanResponsesContainer plan_responses_container{ motion_plan_requests.size() };
  std::vector<std::thread> planning_threads;
  planning_threads.reserve(motion_plan_requests.size());

  // Print a warning if more parallel planning problems than available concurrent threads are defined. If
  // std::thread::hardware_concurrency() is not defined, the command returns 0 so the check does not work
  const auto hardware_concurrency = std::thread::hardware_concurrency();
  if (motion_plan_requests.size() > hardware_concurrency && hardware_concurrency != 0)
  {
    RCLCPP_WARN(getLogger(),
                "More parallel planning problems defined ('%ld') than possible to solve concurrently with the "
                "hardware ('%d')",
                motion_plan_requests.size(), hardware_concurrency);
  }

  // Launch planning threads
  for (const auto& request : motion_plan_requests)
  {
    auto planning_thread = std::thread([&]() {
      auto plan_solution = ::planning_interface::MotionPlanResponse();
      try
      {
        // Use planning scene if provided, otherwise the planning scene from planning scene monitor is used
        plan_solution = planWithSinglePipeline(request, planning_scene, planning_pipelines);
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(getLogger(), "Planning pipeline '%s' threw exception '%s'", request.pipeline_id.c_str(), e.what());
        plan_solution = ::planning_interface::MotionPlanResponse();
        plan_solution.error_code = moveit::core::MoveItErrorCode::FAILURE;
      }
      plan_solution.planner_id = request.planner_id;
      plan_responses_container.pushBack(plan_solution);

      if (stopping_criterion_callback != nullptr)
      {
        if (stopping_criterion_callback(plan_responses_container, motion_plan_requests))
        {
          // Terminate planning pipelines
          RCLCPP_INFO(getLogger(), "Stopping criterion met: Terminating planning pipelines that are still active");
          for (const auto& request : motion_plan_requests)
          {
            try
            {
              const auto& planning_pipeline = planning_pipelines.at(request.pipeline_id);
              if (planning_pipeline->isActive())
              {
                planning_pipeline->terminate();
              }
            }
            catch (const std::out_of_range&)
            {
              RCLCPP_WARN(getLogger(), "Cannot terminate pipeline '%s' because no pipeline with that name exists",
                          request.pipeline_id.c_str());
            }
          }
        }
      }
    });
    planning_threads.push_back(std::move(planning_thread));
  }

  // Wait for threads to finish
  for (auto& planning_thread : planning_threads)
  {
    if (planning_thread.joinable())
    {
      planning_thread.join();
    }
  }

  // If a solution selection function is provided, it is used to compute the return value
  if (solution_selection_function)
  {
    std::vector<::planning_interface::MotionPlanResponse> solutions;
    solutions.reserve(1);
    solutions.push_back(solution_selection_function(plan_responses_container.getSolutions()));
    return solutions;
  }

  // Otherwise, just return the unordered list of solutions
  return plan_responses_container.getSolutions();
}

std::unordered_map<std::string, planning_pipeline::PlanningPipelinePtr>
createPlanningPipelineMap(const std::vector<std::string>& pipeline_names,
                          const moveit::core::RobotModelConstPtr& robot_model, const rclcpp::Node::SharedPtr& node,
                          const std::string& parameter_namespace)
{
  std::unordered_map<std::string, planning_pipeline::PlanningPipelinePtr> planning_pipelines;
  for (const auto& planning_pipeline_name : pipeline_names)
  {
    // Check if pipeline already exists
    if (planning_pipelines.count(planning_pipeline_name) > 0)
    {
      RCLCPP_WARN(getLogger(), "Skipping duplicate entry for planning pipeline '%s'.", planning_pipeline_name.c_str());
      continue;
    }

    // Create planning pipeline
    planning_pipeline::PlanningPipelinePtr pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(
        robot_model, node, parameter_namespace + planning_pipeline_name);

    if (!pipeline)
    {
      RCLCPP_ERROR(getLogger(), "Failed to initialize planning pipeline '%s'.", planning_pipeline_name.c_str());
      continue;
    }

    planning_pipelines[planning_pipeline_name] = pipeline;
  }

  return planning_pipelines;
}

}  // namespace planning_pipeline_interfaces
}  // namespace moveit
