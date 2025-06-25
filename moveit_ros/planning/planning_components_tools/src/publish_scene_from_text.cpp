/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ioan A. Sucan
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
 *   * Neither the name of Ioan A. Sucan nor the names of its
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

#include <chrono>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/version.h>
#include <moveit/utils/logger.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#if RCLCPP_VERSION_GTE(20, 0, 0)
#include <rclcpp/event_handler.hpp>
#else
#include <rclcpp/qos_event.hpp>
#endif
#include <rclcpp/utilities.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("publish_planning_scene");
  moveit::setNodeLoggerName(node->get_name());

  bool publish_scene = false;  // Set by --scene flag
  bool publish_world = false;  // Set by --world flag, or the default if no flags specified
  std::string filename;

  boost::program_options::options_description desc;
  desc.add_options()("scene", "Publish full planning scene")("world", "Publish world scene (default)");
  desc.add_options()("input-file", boost::program_options::value<std::string>(&filename), "input file");

  boost::program_options::positional_options_description p;
  p.add("input-file", -1);

  boost::program_options::variables_map vm;
  boost::program_options::parsed_options po =
      boost::program_options::command_line_parser(argc, argv).options(desc).positional(p).run();
  boost::program_options::store(po, vm);
  boost::program_options::notify(vm);

  if (vm.count("scene"))
  {
    publish_scene = true;
  }
  if (vm.count("world"))
  {
    publish_world = true;
  }

  // If no flags specified, default to world (backwards compatibility)
  if (!vm.count("scene") && !vm.count("world"))
  {
    publish_world = true;
  }

  if (filename.empty())
  {
    RCLCPP_WARN(node->get_logger(),
                "A filename was expected as argument. That file should be a text representation of the geometry in a "
                "planning scene.");
    return 1;
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr pub_scene;
  rclcpp::Publisher<moveit_msgs::msg::PlanningSceneWorld>::SharedPtr pub_world_scene;

  if (publish_scene)
  {
    pub_scene = node->create_publisher<moveit_msgs::msg::PlanningScene>(
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_TOPIC, 1);
  }
  if (publish_world)
  {
    pub_world_scene = node->create_publisher<moveit_msgs::msg::PlanningSceneWorld>(
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC, 1);
  }

  robot_model_loader::RobotModelLoader::Options opt;
  opt.robot_description = "robot_description";
  opt.load_kinematics_solvers = false;

  auto rml = std::make_shared<robot_model_loader::RobotModelLoader>(node, opt);
  planning_scene::PlanningScene ps(rml->getModel());

  std::ifstream f(filename);
  if (ps.loadGeometryFromStream(f))
  {
    RCLCPP_INFO(node->get_logger(), "Publishing geometry from '%s' ...", filename.c_str());
    moveit_msgs::msg::PlanningScene ps_msg;
    ps.getPlanningSceneMsg(ps_msg);
    ps_msg.is_diff = true;

    // Wait for subscribers on both topics
    unsigned int attempts = 0;
    while (++attempts < 100)
    {
      bool scene_ready = !publish_scene || pub_scene->get_subscription_count() >= 1;
      bool world_ready = !publish_world || pub_world_scene->get_subscription_count() >= 1;
      if (scene_ready && world_ready)
        break;
      rclcpp::sleep_for(500ms);
    }

    if (publish_scene)
    {
      pub_scene->publish(ps_msg);
    }
    if (publish_world)
    {
      pub_world_scene->publish(ps_msg.world);
    }

    rclcpp::sleep_for(1s);
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to load geometry from file '%s'", filename.c_str());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
