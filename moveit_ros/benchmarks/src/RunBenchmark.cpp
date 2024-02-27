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

#include <string>

#include <moveit/benchmarks/BenchmarkOptions.h>
#include <moveit/benchmarks/BenchmarkExecutor.h>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/utilities.hpp>
#include <moveit/utils/logger.hpp>

using moveit::getLogger;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("moveit_run_benchmark", node_options);
  moveit::setNodeLoggerName(node->get_name());

  // Read benchmark options from param server
  moveit_ros_benchmarks::BenchmarkOptions options(node);
  // Setup benchmark server
  moveit_ros_benchmarks::BenchmarkExecutor server(node);

  std::vector<std::string> planning_pipelines;
  options.getPlanningPipelineNames(planning_pipelines);
  if (!server.initialize(planning_pipelines))
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize benchmark server.");
    rclcpp::shutdown();
    return 1;
  }

  if (options.scene_name.empty())
  {
    std::vector<std::string> scene_names;
    try
    {
      warehouse_ros::DatabaseLoader db_loader(node);
      warehouse_ros::DatabaseConnection::Ptr warehouse_connection = db_loader.loadDatabase();
      warehouse_connection->setParams(options.hostname, options.port, 20);
      if (warehouse_connection->connect())
      {
        auto planning_scene_storage = moveit_warehouse::PlanningSceneStorage(warehouse_connection);
        planning_scene_storage.getPlanningSceneNames(scene_names);
        RCLCPP_INFO(node->get_logger(), "Loaded scene names");
      }
      else
      {
        RCLCPP_ERROR(node->get_logger(), "Failed to load scene names from DB");
        rclcpp::shutdown();
        return 1;
      }
    }
    catch (std::exception& e)
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to load scene names from DB: '%s'", e.what());
      rclcpp::shutdown();
      return 1;
    }
    // Running benchmarks
    for (const auto& name : scene_names)
    {
      options.scene_name = name;
      if (!server.runBenchmarks(options))
      {
        RCLCPP_ERROR(node->get_logger(), "Failed to run all benchmarks");
      }
    }
  }
  else
  {
    if (!server.runBenchmarks(options))
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to run all benchmarks");
    }
  }

  RCLCPP_INFO(node->get_logger(), "Finished benchmarking");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
