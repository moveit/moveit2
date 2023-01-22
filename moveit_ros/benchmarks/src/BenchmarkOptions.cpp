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

using namespace moveit_ros_benchmarks;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit.ros.benchmarks.BenchmarkOptions");

BenchmarkOptions::BenchmarkOptions()
{
}

BenchmarkOptions::BenchmarkOptions(const rclcpp::Node::SharedPtr& node)
{
  readBenchmarkOptions(node);
}

BenchmarkOptions::~BenchmarkOptions() = default;

// void BenchmarkOptions::setNamespace(const std::string& ros_namespace)
// {
//   readBenchmarkOptions(ros_namespace);
// }

void BenchmarkOptions::readBenchmarkOptions(const rclcpp::Node::SharedPtr& node)
{
  if (node->has_parameter("benchmark_config.parameters.name"))
  {
    readWarehouseOptions(node);
    readBenchmarkParameters(node);
    readPlannerConfigs(node);
  }
  else
  {
    RCLCPP_WARN(LOGGER, "No benchmark_config found on param server");
  }
}

const std::string& BenchmarkOptions::getHostName() const
{
  return hostname_;
}

int BenchmarkOptions::getPort() const
{
  return port_;
}

const std::string& BenchmarkOptions::getSceneName() const
{
  return scene_name_;
}

int BenchmarkOptions::getNumRuns() const
{
  return runs_;
}

double BenchmarkOptions::getTimeout() const
{
  return timeout_;
}

const std::string& BenchmarkOptions::getBenchmarkName() const
{
  return benchmark_name_;
}

const std::string& BenchmarkOptions::getGroupName() const
{
  return group_name_;
}

const std::string& BenchmarkOptions::getOutputDirectory() const
{
  return output_directory_;
}

const std::string& BenchmarkOptions::getQueryRegex() const
{
  return query_regex_;
}

const std::string& BenchmarkOptions::getStartStateRegex() const
{
  return start_state_regex_;
}

const std::string& BenchmarkOptions::getGoalConstraintRegex() const
{
  return goal_constraint_regex_;
}

const std::string& BenchmarkOptions::getPathConstraintRegex() const
{
  return path_constraint_regex_;
}

const std::string& BenchmarkOptions::getTrajectoryConstraintRegex() const
{
  return trajectory_constraint_regex_;
}

const std::vector<std::string>& BenchmarkOptions::getPredefinedPoses() const
{
  return predefined_poses_;
}

const std::string& BenchmarkOptions::getPredefinedPosesGroup() const
{
  return predefined_poses_group_;
}

void BenchmarkOptions::getGoalOffsets(std::vector<double>& offsets) const
{
  offsets.resize(6);
  memcpy(&offsets[0], goal_offsets, 6 * sizeof(double));
}

const std::map<std::string, std::vector<std::string>>& BenchmarkOptions::getPlanningPipelineConfigurations() const
{
  return planning_pipelines_;
}

void BenchmarkOptions::getPlanningPipelineNames(std::vector<std::string>& planning_pipeline_names) const
{
  planning_pipeline_names.clear();
  for (const std::pair<const std::string, std::vector<std::string>>& planning_pipeline : planning_pipelines_)
    planning_pipeline_names.push_back(planning_pipeline.first);
}

const std::string& BenchmarkOptions::getWorkspaceFrameID() const
{
  return workspace_.header.frame_id;
}

const moveit_msgs::msg::WorkspaceParameters& BenchmarkOptions::getWorkspaceParameters() const
{
  return workspace_;
}

void BenchmarkOptions::readWarehouseOptions(const rclcpp::Node::SharedPtr& node)
{
  node->get_parameter_or(std::string("benchmark_config.warehouse.host"), hostname_, std::string("127.0.0.1"));
  node->get_parameter_or(std::string("benchmark_config.warehouse.port"), port_, 33829);

  if (!node->get_parameter("benchmark_config.warehouse.scene_name", scene_name_))
    RCLCPP_WARN(LOGGER, "Benchmark scene_name NOT specified");

  RCLCPP_INFO(LOGGER, "Benchmark host: %s", hostname_.c_str());
  RCLCPP_INFO(LOGGER, "Benchmark port: %d", port_);
  RCLCPP_INFO(LOGGER, "Benchmark scene: %s", scene_name_.c_str());
}

void BenchmarkOptions::readBenchmarkParameters(const rclcpp::Node::SharedPtr& node)
{
  node->get_parameter_or(std::string("benchmark_config.parameters.name"), benchmark_name_, std::string(""));
  node->get_parameter_or(std::string("benchmark_config.parameters.runs"), runs_, 10);
  node->get_parameter_or(std::string("benchmark_config.parameters.timeout"), timeout_, 10.0);
  node->get_parameter_or(std::string("benchmark_config.parameters.output_directory"), output_directory_,
                         std::string(""));
  node->get_parameter_or(std::string("benchmark_config.parameters.queries"), query_regex_, std::string(".*"));
  node->get_parameter_or(std::string("benchmark_config.parameters.start_states"), start_state_regex_, std::string(""));
  node->get_parameter_or(std::string("benchmark_config.parameters.goal_constraints"), goal_constraint_regex_,
                         std::string(""));
  node->get_parameter_or(std::string("benchmark_config.parameters.path_constraints"), path_constraint_regex_,
                         std::string(""));
  node->get_parameter_or(std::string("benchmark_config.parameters.trajectory_constraints"),
                         trajectory_constraint_regex_, std::string(""));
  node->get_parameter_or(std::string("benchmark_config.parameters.predefined_poses"), predefined_poses_, {});
  node->get_parameter_or(std::string("benchmark_config.parameters.predefined_poses_group"), predefined_poses_group_,
                         std::string(""));

  if (!node->get_parameter(std::string("benchmark_config.parameters.group"), group_name_))
    RCLCPP_INFO(LOGGER, "Benchmark group NOT specified");

  if (node->has_parameter("benchmark_config.parameters.workspace"))
    readWorkspaceParameters(node);

  // Reading in goal_offset (or defaulting to zero)
  node->get_parameter_or(std::string("benchmark_config.parameters.goal_offset.x"), goal_offsets[0], 0.0);
  node->get_parameter_or(std::string("benchmark_config.parameters.goal_offset.y"), goal_offsets[1], 0.0);
  node->get_parameter_or(std::string("benchmark_config.parameters.goal_offset.z"), goal_offsets[2], 0.0);
  node->get_parameter_or(std::string("benchmark_config.parameters.goal_offset.roll"), goal_offsets[3], 0.0);
  node->get_parameter_or(std::string("benchmark_config.parameters.goal_offset.pitch"), goal_offsets[4], 0.0);
  node->get_parameter_or(std::string("benchmark_config.parameters.goal_offset.yaw"), goal_offsets[5], 0.0);

  RCLCPP_INFO(LOGGER, "Benchmark name: '%s'", benchmark_name_.c_str());
  RCLCPP_INFO(LOGGER, "Benchmark #runs: %d", runs_);
  RCLCPP_INFO(LOGGER, "Benchmark timeout: %f secs", timeout_);
  RCLCPP_INFO(LOGGER, "Benchmark group: %s", group_name_.c_str());
  RCLCPP_INFO(LOGGER, "Benchmark query regex: '%s'", query_regex_.c_str());
  RCLCPP_INFO(LOGGER, "Benchmark start state regex: '%s':", start_state_regex_.c_str());
  RCLCPP_INFO(LOGGER, "Benchmark goal constraint regex: '%s':", goal_constraint_regex_.c_str());
  RCLCPP_INFO(LOGGER, "Benchmark path constraint regex: '%s':", path_constraint_regex_.c_str());
  RCLCPP_INFO(LOGGER, "Benchmark goal offsets (%f %f %f, %f %f %f)", goal_offsets[0], goal_offsets[1], goal_offsets[2],
              goal_offsets[3], goal_offsets[4], goal_offsets[5]);
  RCLCPP_INFO(LOGGER, "Benchmark output directory: %s", output_directory_.c_str());
  RCLCPP_INFO_STREAM(LOGGER, "Benchmark workspace: min_corner: ["
                                 << workspace_.min_corner.x << ", " << workspace_.min_corner.y << ", "
                                 << workspace_.min_corner.z << "], "
                                 << "max_corner: [" << workspace_.max_corner.x << ", " << workspace_.max_corner.y
                                 << ", " << workspace_.max_corner.z << ']');
}

void BenchmarkOptions::readWorkspaceParameters(const rclcpp::Node::SharedPtr& node)
{
  // Make sure all params exist
  if (!node->get_parameter("benchmark_config.parameters.workspace.frame_id", workspace_.header.frame_id))
    RCLCPP_WARN(LOGGER, "Workspace frame_id not specified in benchmark config");

  node->get_parameter_or(std::string("benchmark_config.parameters.workspace.min_corner.x"), workspace_.min_corner.x,
                         0.0);
  node->get_parameter_or(std::string("benchmark_config.parameters.workspace.min_corner.y"), workspace_.min_corner.y,
                         0.0);
  node->get_parameter_or(std::string("benchmark_config.parameters.workspace.min_corner.z"), workspace_.min_corner.z,
                         0.0);

  node->get_parameter_or(std::string("benchmark_config.parameters.workspace.max_corner.x"), workspace_.max_corner.x,
                         0.0);
  node->get_parameter_or(std::string("benchmark_config.parameters.workspace.max_corner.y"), workspace_.max_corner.y,
                         0.0);
  node->get_parameter_or(std::string("benchmark_config.parameters.workspace.max_corner.z"), workspace_.max_corner.z,
                         0.0);

  workspace_.header.stamp = rclcpp::Clock().now();
}

void BenchmarkOptions::readPlannerConfigs(const rclcpp::Node::SharedPtr& node)
{
  planning_pipelines_.clear();

  const std::string np = "benchmark_config.planning_pipelines";
  std::vector<std::string> pipelines;
  if (!node->get_parameter(np + ".pipelines", pipelines))
  {
    RCLCPP_ERROR(LOGGER, "Fail to get the parameter in `%s` namespace.", (np + ".pipelines").c_str());
    return;
  }

  for (const std::string& pipeline : pipelines)
  {
    std::string pipeline_name;
    const std::string pipeline_parameter_name = std::string(np).append(".").append(pipeline).append(".name");
    if (!node->get_parameter(pipeline_parameter_name, pipeline_name))
    {
      RCLCPP_ERROR(LOGGER, "Fail to get the parameter in `%s` namespace.", pipeline_parameter_name.c_str());
      return;
    }

    RCLCPP_INFO(LOGGER, "Reading in planner names for planning pipeline '%s'", pipeline_name.c_str());

    std::vector<std::string> planners;
    if (!node->get_parameter(pipeline_parameter_name, planners))
    {
      RCLCPP_ERROR(LOGGER, "Fail to get the parameter in `%s` namespace.", pipeline_parameter_name.c_str());
      return;
    }

    for (const std::string& planner : planners)
      RCLCPP_INFO(LOGGER, "  %s", planner.c_str());

    planning_pipelines_[pipeline_name] = planners;
  }
}
