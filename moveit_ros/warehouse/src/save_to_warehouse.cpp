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

#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/warehouse/state_storage.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/utils/logger.hpp>
#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <fmt/format.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/executors.hpp>
#include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/version.h>
#if RCLCPP_VERSION_GTE(20, 0, 0)
#include <rclcpp/event_handler.hpp>
#else
#include <rclcpp/qos_event.hpp>
#endif
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>

rclcpp::Logger getLogger()
{
  return moveit::getLogger("save_to_warehouse");
}

static const std::string ROBOT_DESCRIPTION = "robot_description";

void onSceneUpdate(planning_scene_monitor::PlanningSceneMonitor& psm, moveit_warehouse::PlanningSceneStorage& pss)
{
  RCLCPP_INFO(getLogger(), "Received an update to the planning scene...");

  if (!psm.getPlanningScene()->getName().empty())
  {
    if (!pss.hasPlanningScene(psm.getPlanningScene()->getName()))
    {
      moveit_msgs::msg::PlanningScene psmsg;
      psm.getPlanningScene()->getPlanningSceneMsg(psmsg);
      pss.addPlanningScene(psmsg);
    }
    else
    {
      RCLCPP_INFO(getLogger(), "Scene '%s' was previously added. Not adding again.",
                  psm.getPlanningScene()->getName().c_str());
    }
  }
  else
    RCLCPP_INFO(getLogger(), "Scene name is empty. Not saving.");
}

void onMotionPlanRequest(const moveit_msgs::msg::MotionPlanRequest& req,
                         planning_scene_monitor::PlanningSceneMonitor& psm, moveit_warehouse::PlanningSceneStorage& pss)
{
  if (psm.getPlanningScene()->getName().empty())
  {
    RCLCPP_INFO(getLogger(), "Scene name is empty. Not saving planning request.");
    return;
  }
  pss.addPlanningQuery(req, psm.getPlanningScene()->getName());
}

void onConstraints(const moveit_msgs::msg::Constraints& msg, moveit_warehouse::ConstraintsStorage& cs)
{
  if (msg.name.empty())
  {
    RCLCPP_INFO(getLogger(), "No name specified for constraints. Not saving.");
    return;
  }

  if (cs.hasConstraints(msg.name))
  {
    RCLCPP_INFO(getLogger(), "Replacing constraints '%s'", msg.name.c_str());
    cs.removeConstraints(msg.name);
    cs.addConstraints(msg);
  }
  else
  {
    RCLCPP_INFO(getLogger(), "Adding constraints '%s'", msg.name.c_str());
    cs.addConstraints(msg);
  }
}

void onRobotState(const moveit_msgs::msg::RobotState& msg, moveit_warehouse::RobotStateStorage& rs)
{
  std::vector<std::string> names;
  rs.getKnownRobotStates(names);
  std::set<std::string> names_set(names.begin(), names.end());
  std::size_t n = names.size();
  while (names_set.find("S" + std::to_string(n)) != names_set.end())
    n++;
  std::string name = "S" + std::to_string(n);
  RCLCPP_INFO(getLogger(), "Adding robot state '%s'", name.c_str());
  rs.addRobotState(msg, name);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("save_to_warehouse", node_options);
  moveit::setNodeLoggerName(node->get_name());

  boost::program_options::options_description desc;
  desc.add_options()("help", "Show help message")("host", boost::program_options::value<std::string>(),
                                                  "Host for the "
                                                  "DB.")("port", boost::program_options::value<std::size_t>(),
                                                         "Port for the DB.");

  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << '\n';
    return 1;
  }
  // Set up db
  warehouse_ros::DatabaseConnection::Ptr conn = moveit_warehouse::loadDatabase(node);
  if (vm.count("host") && vm.count("port"))
    conn->setParams(vm["host"].as<std::string>(), vm["port"].as<std::size_t>());
  if (!conn->connect())
    return 1;

  planning_scene_monitor::PlanningSceneMonitor psm(node, ROBOT_DESCRIPTION);
  if (!psm.getPlanningScene())
  {
    RCLCPP_ERROR(node->get_logger(), "Unable to initialize PlanningSceneMonitor");
    return 1;
  }

  psm.startSceneMonitor();
  psm.startWorldGeometryMonitor();
  moveit_warehouse::PlanningSceneStorage pss(conn);
  moveit_warehouse::ConstraintsStorage cs(conn);
  moveit_warehouse::RobotStateStorage rs(conn);
  std::vector<std::string> names;
  pss.getPlanningSceneNames(names);
  if (names.empty())
  {
    RCLCPP_INFO(node->get_logger(), "There are no previously stored scenes");
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "Previously stored scenes:");
    for (const std::string& name : names)
      RCLCPP_INFO(node->get_logger(), " * %s", name.c_str());
  }
  cs.getKnownConstraints(names);
  if (names.empty())
  {
    RCLCPP_INFO(node->get_logger(), "There are no previously stored constraints");
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "Previously stored constraints:");
    for (const std::string& name : names)
      RCLCPP_INFO(node->get_logger(), " * %s", name.c_str());
  }
  rs.getKnownRobotStates(names);
  if (names.empty())
  {
    RCLCPP_INFO(node->get_logger(), "There are no previously stored robot states");
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "Previously stored robot states:");
    for (const std::string& name : names)
      RCLCPP_INFO(node->get_logger(), " * %s", name.c_str());
  }

  psm.addUpdateCallback([&](auto&&) { return onSceneUpdate(psm, pss); });

  auto mplan_req_sub = node->create_subscription<moveit_msgs::msg::MotionPlanRequest>(
      "motion_plan_request", rclcpp::SystemDefaultsQoS(),
      [&](const moveit_msgs::msg::MotionPlanRequest& msg) { onMotionPlanRequest(msg, psm, pss); });
  auto constr_sub = node->create_subscription<moveit_msgs::msg::Constraints>(
      "constraints", rclcpp::SystemDefaultsQoS(),
      [&](const moveit_msgs::msg::Constraints& msg) { onConstraints(msg, cs); });
  auto state_sub = node->create_subscription<moveit_msgs::msg::RobotState>(
      "robot_state", rclcpp::SystemDefaultsQoS(),
      [&](const moveit_msgs::msg::RobotState& msg) { onRobotState(msg, rs); });

  std::vector<std::string> topics;
  psm.getMonitoredTopics(topics);
  RCLCPP_INFO_STREAM(node->get_logger(),
                     "Listening for scene updates on topics " << fmt::format("{}", fmt::join(topics, ", ")));
  RCLCPP_INFO_STREAM(node->get_logger(), "Listening for planning requests on topic " << mplan_req_sub->get_topic_name());
  RCLCPP_INFO_STREAM(node->get_logger(), "Listening for named constraints on topic " << constr_sub->get_topic_name());
  RCLCPP_INFO_STREAM(node->get_logger(), "Listening for states on topic " << state_sub->get_topic_name());

  rclcpp::spin(node);
  return 0;
}
