/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, PickNik Inc.
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

/* Author: Paul Gesel
   Description: Integration tests for the move_group default planning scene behavior
*/

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

TEST(MoveGroup, testDefaultPlanningScene)
{
  // PlanningSceneMonitor is used for getting the robot joint state and calculating the Cartesian pose for a given link.
  auto node = std::make_shared<rclcpp::Node>("move_group_test");
  auto planning_scene_monitor =
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
  planning_scene_monitor->requestPlanningSceneState();
  planning_scene::PlanningScenePtr ps = planning_scene_monitor->getPlanningScene();

  // ensure the planning scene and world are not null
  ASSERT_NE(ps, nullptr);
  ASSERT_NE(ps->getWorld(), nullptr);

  // check that geometry in test file matches the currecnt values
  std::unordered_set<std::string> expected_ids = { "Box_0", "Cylinder_0" };
  for (const auto& id : ps->getWorld()->getObjectIds())
  {
    EXPECT_NE(expected_ids.find(id), expected_ids.end());
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
