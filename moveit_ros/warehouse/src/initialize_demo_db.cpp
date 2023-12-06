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

#include <math.h>
#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/warehouse/state_storage.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <boost/algorithm/string/join.hpp>
#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/utilities.hpp>
#include <moveit/utils/logger.hpp>

static const std::string ROBOT_DESCRIPTION = "robot_description";

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("initialize_demo_db", node_options);
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

  moveit_warehouse::PlanningSceneStorage pss(conn);
  moveit_warehouse::ConstraintsStorage cs(conn);
  moveit_warehouse::RobotStateStorage rs(conn);
  pss.reset();
  cs.reset();
  rs.reset();

  // add default planning scenes
  moveit_msgs::msg::PlanningScene psmsg;
  psm.getPlanningScene()->getPlanningSceneMsg(psmsg);
  psmsg.name = "default";
  pss.addPlanningScene(psmsg);
  RCLCPP_INFO(node->get_logger(), "Added default scene: '%s'", psmsg.name.c_str());

  moveit_msgs::msg::RobotState rsmsg;
  moveit::core::robotStateToRobotStateMsg(psm.getPlanningScene()->getCurrentState(), rsmsg);
  rs.addRobotState(rsmsg, "default");
  RCLCPP_INFO(node->get_logger(), "Added default state");

  const std::vector<std::string>& gnames = psm.getRobotModel()->getJointModelGroupNames();
  for (const std::string& gname : gnames)
  {
    const moveit::core::JointModelGroup* jmg = psm.getRobotModel()->getJointModelGroup(gname);
    if (!jmg->isChain())
      continue;
    const std::vector<std::string>& lnames = jmg->getLinkModelNames();
    if (lnames.empty())
      continue;

    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = lnames.back();
    ocm.header.frame_id = psm.getRobotModel()->getModelFrame();
    ocm.orientation.x = 0.0;
    ocm.orientation.y = 0.0;
    ocm.orientation.z = 0.0;
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = M_PI;
    ocm.weight = 1.0;
    moveit_msgs::msg::Constraints cmsg;
    cmsg.orientation_constraints.resize(1, ocm);
    cmsg.name = ocm.link_name + ":upright";
    cs.addConstraints(cmsg, psm.getRobotModel()->getName(), jmg->getName());
    RCLCPP_INFO(node->get_logger(), "Added default constraint: '%s'", cmsg.name.c_str());
  }
  RCLCPP_INFO(node->get_logger(), "Default MoveIt Warehouse was reset.");

  rclcpp::spin(node);

  return 0;
}
