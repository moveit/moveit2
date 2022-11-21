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

#include <chrono>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("display_random_state");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("display_random_state");

  bool valid = false;
  bool invalid = false;
  for (int i = 0; i < argc; ++i)
  {
    if (strcmp(argv[i], "--valid") == 0)
    {
      valid = true;
      break;
    }
    if (strcmp(argv[i], "--invalid") == 0)
    {
      invalid = true;
      break;
    }
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  robot_model_loader::RobotModelLoader::Options opt;
  opt.robot_description_ = "robot_description";
  auto rml = std::make_shared<robot_model_loader::RobotModelLoader>(node, opt);
  planning_scene_monitor::PlanningSceneMonitor psm(node, rml);
  psm.startWorldGeometryMonitor();
  psm.startSceneMonitor();
  auto pub_scene = node->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);

  rclcpp::sleep_for(500ms);

  do
  {
    if (!psm.getPlanningScene())
    {
      RCLCPP_ERROR(LOGGER, "Planning scene did not load properly, exiting...");
      break;
    }

    std::cout << "Type a number and hit Enter. That number of ";
    if (valid)
    {
      std::cout << "valid ";
    }
    else if (invalid)
    {
      std::cout << "invalid ";
    }
    std::cout << "states will be randomly generated at an interval of one second and published as a planning scene."
              << '\n';
    std::size_t n;
    std::cin >> n;

    for (std::size_t i = 0; i < n; ++i)
    {
      if (valid)
      {
        bool found = false;
        unsigned int attempts = 0;
        do
        {
          attempts++;
          psm.getPlanningScene()->getCurrentStateNonConst().setToRandomPositions();
          collision_detection::CollisionRequest req;
          collision_detection::CollisionResult res;
          psm.getPlanningScene()->checkCollision(req, res);
          found = !res.collision;
        } while (!found && attempts < 100);
        if (!found)
        {
          std::cout << "Unable to find valid state" << '\n';
          continue;
        }
      }
      else if (invalid)
      {
        bool found = false;
        unsigned int attempts = 0;
        do
        {
          attempts++;
          psm.getPlanningScene()->getCurrentStateNonConst().setToRandomPositions();
          collision_detection::CollisionRequest req;
          collision_detection::CollisionResult res;
          psm.getPlanningScene()->checkCollision(req, res);
          found = res.collision;
        } while (!found && attempts < 100);
        if (!found)
        {
          std::cout << "Unable to find invalid state" << '\n';
          continue;
        }
      }
      else
        psm.getPlanningScene()->getCurrentStateNonConst().setToRandomPositions();

      moveit_msgs::msg::PlanningScene psmsg;
      psm.getPlanningScene()->getPlanningSceneMsg(psmsg);
      pub_scene->publish(psmsg);
      std::cout << psm.getPlanningScene()->getCurrentState() << '\n';

      rclcpp::sleep_for(1s);
    }
  } while (rclcpp::ok());

  rclcpp::shutdown();
  return 0;
}
