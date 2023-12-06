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
#include <moveit/utils/logger.hpp>
#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <rclcpp/executors/static_single_threaded_executor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/version.h>
#if RCLCPP_VERSION_GTE(20, 0, 0)
#include <rclcpp/event_handler.hpp>
#else
#include <rclcpp/qos_event.hpp>
#endif
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>

static const std::string PLANNING_SCENE_TOPIC = "planning_scene";
static const std::string PLANNING_REQUEST_TOPIC = "motion_plan_request";
static const std::string PLANNING_RESULTS_TOPIC = "motion_plan_results";

static const std::string CONSTRAINTS_TOPIC = "constraints";

static const std::string STATES_TOPIC = "robot_states";

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("publish_warehouse_data", node_options);
  moveit::setNodeLoggerName(node->get_name());

  // time to wait in between publishing messages
  double delay = 0.001;

  // clang-format off
  boost::program_options::options_description desc;
  desc.add_options()("help", "Show help message")("host", boost::program_options::value<std::string>(),
                                                  "Host for the "
                                                  "DB.")("port", boost::program_options::value<std::size_t>(),
                                                         "Port for the DB.")(
      "scene", boost::program_options::value<std::string>(), "Name of scene to publish.")(
      "planning_requests", "Also publish the planning requests that correspond to the scene")(
      "planning_results", "Also publish the planning results that correspond to the scene")(
      "constraint", boost::program_options::value<std::string>(), "Name of constraint to publish.")(
      "state", boost::program_options::value<std::string>(),
      "Name of the robot state to publish.")("delay", boost::program_options::value<double>()->default_value(delay),
                                             "Time to wait in between publishing messages (s)");
  // clang-format on
  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);

  if (vm.count("help") || (!vm.count("scene") && !vm.count("constraint") && !vm.count("state")))
  {
    std::cout << desc << '\n';
    return 1;
  }
  try
  {
    delay = vm["delay"].as<double>();
  }
  catch (...)
  {
    std::cout << desc << '\n';
    return 2;
  }
  // Set up db
  warehouse_ros::DatabaseConnection::Ptr conn = moveit_warehouse::loadDatabase(node);
  if (vm.count("host") && vm.count("port"))
    conn->setParams(vm["host"].as<std::string>(), vm["port"].as<std::size_t>());
  if (!conn->connect())
    return 1;

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  executor.add_node(node);

  rclcpp::Rate rate(static_cast<int64_t>(delay) * 1000ms);

  // publish the scene
  if (vm.count("scene"))
  {
    auto pub_scene = node->create_publisher<moveit_msgs::msg::PlanningScene>(PLANNING_SCENE_TOPIC, 10);
    bool req = vm.count("planning_requests");
    bool res = vm.count("planning_results");

    moveit_warehouse::PlanningSceneStorage pss(conn);
    executor.spin_once(0ns);

    std::vector<std::string> scene_names;
    pss.getPlanningSceneNames(vm["scene"].as<std::string>(), scene_names);

    for (const std::string& scene_name : scene_names)
    {
      moveit_warehouse::PlanningSceneWithMetadata pswm;
      if (pss.getPlanningScene(pswm, scene_name))
      {
        RCLCPP_INFO(node->get_logger(), "Publishing scene '%s'",
                    pswm->lookupString(moveit_warehouse::PlanningSceneStorage::PLANNING_SCENE_ID_NAME).c_str());
        pub_scene->publish(static_cast<const moveit_msgs::msg::PlanningScene&>(*pswm));
        executor.spin_once(0ns);

        // publish optional data associated to the scene
        if (req || res)
        {
          auto pub_req = node->create_publisher<moveit_msgs::msg::MotionPlanRequest>(PLANNING_REQUEST_TOPIC, 100);
          auto pub_res = node->create_publisher<moveit_msgs::msg::RobotTrajectory>(PLANNING_RESULTS_TOPIC, 100);
          std::vector<moveit_warehouse::MotionPlanRequestWithMetadata> planning_queries;
          std::vector<std::string> query_names;
          pss.getPlanningQueries(planning_queries, query_names, pswm->name);
          RCLCPP_INFO(node->get_logger(), "There are %d planning queries associated to the scene",
                      static_cast<int>(planning_queries.size()));
          rclcpp::sleep_for(500ms);
          for (std::size_t i = 0; i < planning_queries.size(); ++i)
          {
            if (req)
            {
              RCLCPP_INFO(node->get_logger(), "Publishing query '%s'", query_names[i].c_str());
              pub_req->publish(static_cast<const moveit_msgs::msg::MotionPlanRequest&>(*planning_queries[i]));
              executor.spin_once(0ns);
            }
            if (res)
            {
              std::vector<moveit_warehouse::RobotTrajectoryWithMetadata> planning_results;
              pss.getPlanningResults(planning_results, query_names[i], pswm->name);
              for (moveit_warehouse::RobotTrajectoryWithMetadata& planning_result : planning_results)
              {
                pub_res->publish(static_cast<const moveit_msgs::msg::RobotTrajectory&>(*planning_result));
                executor.spin_once(0ns);
              }
            }
          }
        }
        rate.sleep();
      }
    }
  }

  // publish constraints
  if (vm.count("constraint"))
  {
    moveit_warehouse::ConstraintsStorage cs(conn);
    auto pub_constr = node->create_publisher<moveit_msgs::msg::Constraints>(CONSTRAINTS_TOPIC, 100);
    std::vector<std::string> cnames;
    cs.getKnownConstraints(vm["constraint"].as<std::string>(), cnames);

    for (const std::string& cname : cnames)
    {
      moveit_warehouse::ConstraintsWithMetadata cwm;
      if (cs.getConstraints(cwm, cname))
      {
        RCLCPP_INFO(node->get_logger(), "Publishing constraints '%s'",
                    cwm->lookupString(moveit_warehouse::ConstraintsStorage::CONSTRAINTS_ID_NAME).c_str());
        pub_constr->publish(static_cast<const moveit_msgs::msg::Constraints&>(*cwm));
        executor.spin_once(0ns);
        rate.sleep();
      }
    }
  }

  // publish constraints
  if (vm.count("state"))
  {
    moveit_warehouse::RobotStateStorage rs(conn);
    auto pub_state = node->create_publisher<moveit_msgs::msg::RobotState>(STATES_TOPIC, 100);
    std::vector<std::string> rnames;
    rs.getKnownRobotStates(vm["state"].as<std::string>(), rnames);

    for (const std::string& rname : rnames)
    {
      moveit_warehouse::RobotStateWithMetadata rswm;
      if (rs.getRobotState(rswm, rname))
      {
        RCLCPP_INFO(node->get_logger(), "Publishing state '%s'",
                    rswm->lookupString(moveit_warehouse::RobotStateStorage::STATE_NAME).c_str());
        pub_state->publish(static_cast<const moveit_msgs::msg::RobotState&>(*rswm));
        executor.spin_once(0ns);
        rate.sleep();
      }
    }
  }

  rclcpp::sleep_for(1s);
  RCLCPP_INFO(node->get_logger(), "Done.");

  return 0;
}
