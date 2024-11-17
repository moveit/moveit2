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

#pragma once

#include <rclcpp/node.hpp>
#include <string>
#include <map>
#include <vector>
#include <moveit_msgs/msg/workspace_parameters.hpp>

static constexpr int CARTESIAN_DOF = 6;

namespace moveit_ros_benchmarks
{
/// TODO(sjahr): Replace with generate_parameter_library config
/// \brief Options to configure a benchmark experiment. The configuration is provided via ROS2 parameters
/// \details Parameter configuration example:
/// benchmark_config: # Benchmark param namespace
///     warehouse:
///         host: # Host address for warehouse
///         port: # Port name for warehouse
///         scene_name: # Scene name to load for this experiment
///     parameters:
///         name: # Experiment name
///         runs: # Number of experiment runs
///         group: # Joint group name
///         timeout: # Experiment timeout
///         output_directory: # Output directory for results file
///         queries_regex: # Number of queries
///         start_states_regex: # Start states
///         goal_constraints_regex: # Goal constrains
///         path_constraints_regex
///         trajectory_constraints_regex
///         predefined_poses_group: # Group where the predefined poses are specified
///         predefined_poses: # List of named targets
///     planning_pipelines:
///       pipeline_names: # List of pipeline names to be loaded by moveit_cpp
///       pipelines: # List of pipeline names to be used by the benchmark tool
///       my_pipeline: # Example pipeline definition
///         name: # Pipeline name
///         planners: # List of planners of the pipeline to be tested
///       parallel_pipelines: # List of parallel planning pipelines to be tested
///       my_parallel_planning_pipeline:
///         pipelines: # List of parallel planning pipelines
///         planner_ids: # Ordered! list planner_ids used by the individual pipelines listed in 'pipeline'
struct BenchmarkOptions
{
  /** \brief Constructor */
  BenchmarkOptions(const rclcpp::Node::SharedPtr& node);

  /** \brief Get all planning pipeline names */
  void getPlanningPipelineNames(std::vector<std::string>& planning_pipeline_names) const;

  /* \brief Get the frame id of the planning workspace */
  const std::string& getWorkspaceFrameID() const;
  /* \brief Get the parameter set of the planning workspace */
  const moveit_msgs::msg::WorkspaceParameters& getWorkspaceParameters() const;

  bool readBenchmarkOptions(const rclcpp::Node::SharedPtr& node);
  bool readPlannerConfigs(const rclcpp::Node::SharedPtr& node);

  void readWorkspaceParameters(const rclcpp::Node::SharedPtr& node);
  void readGoalOffset(const rclcpp::Node::SharedPtr& node);

  /// Warehouse parameters
  std::string hostname;    // Host address for warehouse
  int port;                // Port name for warehouse
  std::string scene_name;  // Scene name to load for this experiment

  /// Benchmark parameters
  int runs;                                   // Number of experiment runs
  double timeout;                             // Experiment timeout
  std::string benchmark_name;                 // Experiment name
  std::string group_name;                     // Joint group name
  std::string output_directory;               // Output directory for results file
  std::string query_regex;                    // Regex for queries in database
  std::string start_state_regex;              // Regex for start_states in database
  std::string goal_constraint_regex;          // Regex for goal_constraints in database
  std::string path_constraint_regex;          // Regex for path_constraints in database
  std::string trajectory_constraint_regex;    // Regex for trajectory_constraint in database
  std::vector<std::string> predefined_poses;  // List of named targets
  std::string predefined_poses_group;         // Group where the predefined poses are specified
  std::vector<double> goal_offsets =
      std::vector<double>(CARTESIAN_DOF);  // Offset applied to goal constraints: x, y, z, roll, pitch, yaw

  /// planner configurations
  std::map<std::string, std::vector<std::string>> planning_pipelines;
  std::map<std::string, std::vector<std::pair<std::string, std::string>>> parallel_planning_pipelines;

  moveit_msgs::msg::WorkspaceParameters workspace;
};
}  // namespace moveit_ros_benchmarks
