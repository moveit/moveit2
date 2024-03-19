/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Rice University
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
 *   * Neither the name of the Rice University nor the names of its
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

/* Author: Ryan Luna */

#include <moveit/benchmarks/BenchmarkOptions.h>
#include <moveit/utils/logger.hpp>

using moveit::getLogger;
using namespace moveit_ros_benchmarks;

namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.benchmarks.options");
}
}  // namespace

BenchmarkOptions::BenchmarkOptions(const rclcpp::Node::SharedPtr& node)
{
  if (!readBenchmarkOptions(node))
  {
    throw(std::runtime_error("Failed to initialize BenchmarkOptions"));
  }
}

bool BenchmarkOptions::readBenchmarkOptions(const rclcpp::Node::SharedPtr& node)
{
  if (node->has_parameter("benchmark_config.parameters.name"))
  {
    // Read warehouse options
    node->get_parameter_or(std::string("benchmark_config.warehouse.host"), hostname, std::string("127.0.0.1"));
    node->get_parameter_or(std::string("benchmark_config.warehouse.port"), port, 33829);

    if (!node->get_parameter("benchmark_config.warehouse.scene_name", scene_name))
    {
      RCLCPP_WARN(getLogger(), "Benchmark scene_name NOT specified");
    }

    RCLCPP_INFO(getLogger(), "Benchmark host: %s", hostname.c_str());
    RCLCPP_INFO(getLogger(), "Benchmark port: %d", port);
    RCLCPP_INFO(getLogger(), "Benchmark scene: %s", scene_name.c_str());
    // Read benchmark parameters
    node->get_parameter_or(std::string("benchmark_config.parameters.name"), benchmark_name, std::string(""));
    node->get_parameter_or(std::string("benchmark_config.parameters.runs"), runs, 10);
    node->get_parameter_or(std::string("benchmark_config.parameters.timeout"), timeout, 10.0);
    node->get_parameter_or(std::string("benchmark_config.parameters.output_directory"), output_directory,
                           std::string(""));
    node->get_parameter_or(std::string("benchmark_config.parameters.queries_regex"), query_regex, std::string(".*"));
    node->get_parameter_or(std::string("benchmark_config.parameters.start_states_regex"), start_state_regex,
                           std::string(""));
    node->get_parameter_or(std::string("benchmark_config.parameters.goal_constraints_regex"), goal_constraint_regex,
                           std::string(""));
    node->get_parameter_or(std::string("benchmark_config.parameters.path_constraints_regex"), path_constraint_regex,
                           std::string(""));
    node->get_parameter_or(std::string("benchmark_config.parameters.trajectory_constraints_regex"),
                           trajectory_constraint_regex, std::string(""));
    node->get_parameter_or(std::string("benchmark_config.parameters.predefined_poses"), predefined_poses, {});
    node->get_parameter_or(std::string("benchmark_config.parameters.predefined_poses_group"), predefined_poses_group,
                           std::string(""));

    if (!node->get_parameter(std::string("benchmark_config.parameters.group"), group_name))
    {
      RCLCPP_WARN(getLogger(), "Benchmark group NOT specified");
    }

    if (node->has_parameter("benchmark_config.parameters.workspace"))
    {
      // Read workspace parameters
      // Make sure all params exist
      if (!node->get_parameter("benchmark_config.parameters.workspace.frame_id", workspace.header.frame_id))
      {
        RCLCPP_WARN(getLogger(), "Workspace frame_id not specified in benchmark config");
      }

      node->get_parameter_or(std::string("benchmark_config.parameters.workspace.min_corner.x"), workspace.min_corner.x,
                             0.0);
      node->get_parameter_or(std::string("benchmark_config.parameters.workspace.min_corner.y"), workspace.min_corner.y,
                             0.0);
      node->get_parameter_or(std::string("benchmark_config.parameters.workspace.min_corner.z"), workspace.min_corner.z,
                             0.0);

      node->get_parameter_or(std::string("benchmark_config.parameters.workspace.max_corner.x"), workspace.max_corner.x,
                             0.0);
      node->get_parameter_or(std::string("benchmark_config.parameters.workspace.max_corner.y"), workspace.max_corner.y,
                             0.0);
      node->get_parameter_or(std::string("benchmark_config.parameters.workspace.max_corner.z"), workspace.max_corner.z,
                             0.0);

      workspace.header.stamp = rclcpp::Clock().now();
    }

    // Reading in goal_offset (or defaulting to zero)
    node->get_parameter_or(std::string("benchmark_config.parameters.goal_offset.x"), goal_offsets.at(0), 0.0);
    node->get_parameter_or(std::string("benchmark_config.parameters.goal_offset.y"), goal_offsets.at(1), 0.0);
    node->get_parameter_or(std::string("benchmark_config.parameters.goal_offset.z"), goal_offsets.at(2), 0.0);
    node->get_parameter_or(std::string("benchmark_config.parameters.goal_offset.roll"), goal_offsets.at(3), 0.0);
    node->get_parameter_or(std::string("benchmark_config.parameters.goal_offset.pitch"), goal_offsets.at(4), 0.0);
    node->get_parameter_or(std::string("benchmark_config.parameters.goal_offset.yaw"), goal_offsets.at(5), 0.0);

    RCLCPP_INFO(getLogger(), "Benchmark name: '%s'", benchmark_name.c_str());
    RCLCPP_INFO(getLogger(), "Benchmark #runs: %d", runs);
    RCLCPP_INFO(getLogger(), "Benchmark timeout: %f secs", timeout);
    RCLCPP_INFO(getLogger(), "Benchmark group: %s", group_name.c_str());
    RCLCPP_INFO(getLogger(), "Benchmark query regex: '%s'", query_regex.c_str());
    RCLCPP_INFO(getLogger(), "Benchmark start state regex: '%s':", start_state_regex.c_str());
    RCLCPP_INFO(getLogger(), "Benchmark goal constraint regex: '%s':", goal_constraint_regex.c_str());
    RCLCPP_INFO(getLogger(), "Benchmark path constraint regex: '%s':", path_constraint_regex.c_str());
    RCLCPP_INFO(getLogger(), "Benchmark goal offsets (%f %f %f, %f %f %f)", goal_offsets.at(0), goal_offsets.at(1),
                goal_offsets.at(2), goal_offsets.at(3), goal_offsets.at(4), goal_offsets.at(5));
    RCLCPP_INFO(getLogger(), "Benchmark output directory: %s", output_directory.c_str());
    RCLCPP_INFO_STREAM(getLogger(), "Benchmark workspace: min_corner: ["
                                        << workspace.min_corner.x << ", " << workspace.min_corner.y << ", "
                                        << workspace.min_corner.z << "], "
                                        << "max_corner: [" << workspace.max_corner.x << ", " << workspace.max_corner.y
                                        << ", " << workspace.max_corner.z << ']');
    // Read planner configuration
    if (!readPlannerConfigs(node))
    {
      return false;
    }
  }
  else
  {
    RCLCPP_ERROR(getLogger(), "No benchmark_config found on param server");
    return false;
  }
  return true;
}

void BenchmarkOptions::getPlanningPipelineNames(std::vector<std::string>& planning_pipeline_names) const
{
  planning_pipeline_names.clear();
  for (const std::pair<const std::string, std::vector<std::string>>& planning_pipeline : planning_pipelines)
  {
    planning_pipeline_names.push_back(planning_pipeline.first);
  }
}

bool BenchmarkOptions::readPlannerConfigs(const rclcpp::Node::SharedPtr& node)
{
  const std::string ns = "benchmark_config.planning_pipelines";
  // pipelines
  planning_pipelines.clear();

  std::vector<std::string> pipelines;
  if (!node->get_parameter(ns + ".pipelines", pipelines))
  {
    RCLCPP_WARN(getLogger(), "No single planning pipeline namespace `%s` configured.", (ns + ".pipelines").c_str());
  }

  for (const std::string& pipeline : pipelines)
  {
    if (!pipeline.empty())
    {
      std::string pipeline_name;
      const std::string pipeline_parameter_name = std::string(ns).append(".").append(pipeline).append(".name");
      if (!node->get_parameter(pipeline_parameter_name, pipeline_name))
      {
        RCLCPP_ERROR(getLogger(), "Fail to get the parameter in `%s` namespace.", pipeline_parameter_name.c_str());
        return false;
      }

      RCLCPP_INFO(getLogger(), "Reading in planner names for planning pipeline '%s'", pipeline_name.c_str());

      std::vector<std::string> planners;
      const std::string pipeline_parameter_planners = std::string(ns).append(".").append(pipeline).append(".planners");
      if (!node->get_parameter(pipeline_parameter_planners, planners))
      {
        RCLCPP_ERROR(getLogger(), "Fail to get the parameter in `%s` namespace.", pipeline_parameter_planners.c_str());
        return false;
      }

      for (const std::string& planner : planners)
      {
        RCLCPP_INFO(getLogger(), "  %s", planner.c_str());
      }

      planning_pipelines[pipeline_name] = planners;
    }
  }
  // parallel pipelines
  parallel_planning_pipelines.clear();

  std::vector<std::string> parallel_pipelines;
  if (!node->get_parameter(ns + ".parallel_pipelines", parallel_pipelines))
  {
    RCLCPP_WARN(getLogger(), "No parallel planning pipeline namespace `%s` configured.",
                (ns + ".parallel_pipelines").c_str());
  }

  for (const std::string& parallel_pipeline : parallel_pipelines)
  {
    if (!parallel_pipeline.empty())
    {  // Read pipelines
      RCLCPP_INFO(getLogger(), "Reading in parameters for parallel planning pipeline '%s'", parallel_pipeline.c_str());

      // Read pipelines
      std::vector<std::string> pipelines;
      const std::string pipelines_parameter =
          std::string(ns).append(".").append(parallel_pipeline).append(".pipelines");
      if (!node->get_parameter(pipelines_parameter, pipelines))
      {
        RCLCPP_ERROR(getLogger(), "Fail to get the parameter in `%s` namespace.", pipelines_parameter.c_str());
        return false;
      }

      // Read planner_ids
      std::vector<std::string> planner_ids;
      const std::string pipeline_planner_ids_parameter =
          std::string(ns).append(".").append(parallel_pipeline).append(".planner_ids");
      if (!node->get_parameter(pipeline_planner_ids_parameter, planner_ids))
      {
        RCLCPP_ERROR(getLogger(), "Fail to get the parameter in `%s` namespace.",
                     pipeline_planner_ids_parameter.c_str());
        return false;
      }

      if (pipelines.size() != planner_ids.size())
      {
        RCLCPP_ERROR(getLogger(), "Number of planner ids is unequal to the number of pipelines in %s.",
                     parallel_pipeline.c_str());
        return false;
      }

      std::vector<std::pair<std::string, std::string>> pipeline_planner_id_pairs;
      for (size_t i = 0; i < pipelines.size(); ++i)
      {
        pipeline_planner_id_pairs.push_back(std::pair<std::string, std::string>(pipelines.at(i), planner_ids.at(i)));
      }

      parallel_planning_pipelines[parallel_pipeline] = pipeline_planner_id_pairs;

      for (const auto& entry : parallel_planning_pipelines)
      {
        RCLCPP_INFO(getLogger(), "Parallel planning pipeline '%s'", entry.first.c_str());
        for (const auto& pair : entry.second)
        {
          RCLCPP_INFO(getLogger(), "  '%s': '%s'", pair.first.c_str(), pair.second.c_str());
        }
      }
    }
  }
  if (pipelines.empty() && parallel_pipelines.empty())
  {
    RCLCPP_ERROR(getLogger(), "No single or parallel planning pipelines configured for benchmarking.");
    return false;
  }
  return true;
}
