/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
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
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
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

#include <gtest/gtest.h>

#include <iostream>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "pilz_industrial_motion_planner/pilz_industrial_motion_planner.h"
#include "test_utils.h"

#include "rclcpp/rclcpp.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("unittest_pilz_industrial_motion_planner");

class CommandPlannerTest : public testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node_ = rclcpp::Node::make_shared("unittest_pilz_industrial_motion_planner", node_options);
    createPlannerInstance();
  }

  /**
   * @brief initialize the planner plugin
   * The planner is loaded using the pluginlib. Checks that the planner was
   * loaded properly.
   * Exceptions will cause test failure.
   *
   * This function should be called only once during initialization of the
   * class.
   */
  void createPlannerInstance()
  {
    // load robot model
    robot_model_loader::RobotModelLoader rm_loader(node_);
    robot_model_ = rm_loader.getModel();
    ASSERT_TRUE(bool(robot_model_)) << "Failed to load robot model";

    // Load planner name from node parameters
    ASSERT_TRUE(node_->has_parameter("planning_plugin")) << "Could not find parameter 'planning_plugin'";
    node_->get_parameter<std::string>("planning_plugin", planner_plugin_name_);

    // Load the plugin
    try
    {
      planner_plugin_loader_ = std::make_unique<pluginlib::ClassLoader<planning_interface::PlannerManager>>(
          "moveit_core", "planning_interface::PlannerManager");
    }
    catch (pluginlib::PluginlibException& ex)
    {
      RCLCPP_FATAL_STREAM(LOGGER, "Exception while creating planning plugin loader " << ex.what());
      FAIL();
    }

    // Create planner
    try
    {
      planner_instance_.reset(planner_plugin_loader_->createUnmanagedInstance(planner_plugin_name_));
      ASSERT_TRUE(planner_instance_->initialize(robot_model_, node_, ""))
          << "Initializing the planner instance failed.";
    }
    catch (pluginlib::PluginlibException& ex)
    {
      FAIL() << "Could not create planner " << ex.what() << "\n";
    }
  }

  void TearDown() override
  {
    planner_plugin_loader_->unloadLibraryForClass(planner_plugin_name_);
  }

protected:
  // ros stuff
  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelConstPtr robot_model_;

  std::string planner_plugin_name_;
  std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader_;
  planning_interface::PlannerManagerPtr planner_instance_;
};

/**
 * @brief Test that PTP can be loaded
 * This needs to be extended with every new planning Algorithm
 */
TEST_F(CommandPlannerTest, ObtainLoadedPlanningAlgorithms)
{
  // Check for the algorithms
  std::vector<std::string> algs;
  planner_instance_->getPlanningAlgorithms(algs);
  ASSERT_EQ(3u, algs.size()) << "Found more or less planning algorithms as expected! Found:"
                             << ::testing::PrintToString(algs);

  // Collect the algorithms, check for uniqueness
  std::set<std::string> algs_set;
  for (const auto& alg : algs)
  {
    algs_set.insert(alg);
  }
  ASSERT_EQ(algs.size(), algs_set.size()) << "There are two or more algorithms with the same name!";
  ASSERT_TRUE(algs_set.find("LIN") != algs_set.end());
  ASSERT_TRUE(algs_set.find("PTP") != algs_set.end());
  ASSERT_TRUE(algs_set.find("CIRC") != algs_set.end());
}

/**
 * @brief Check that all announced planning algorithms can perform the service
 * request if the planner_id is set.
 */
TEST_F(CommandPlannerTest, CheckValidAlgorithmsForServiceRequest)
{
  // Check for the algorithms
  std::vector<std::string> algs;
  planner_instance_->getPlanningAlgorithms(algs);

  for (const auto& alg : algs)
  {
    planning_interface::MotionPlanRequest req;
    req.planner_id = alg;

    EXPECT_TRUE(planner_instance_->canServiceRequest(req));
  }
}

/**
 * @brief Check that canServiceRequest(req) returns false if planner_id is not
 * supported
 */
TEST_F(CommandPlannerTest, CheckInvalidAlgorithmsForServiceRequest)
{
  planning_interface::MotionPlanRequest req;
  req.planner_id = "NON_EXISTEND_ALGORITHM_HASH_da39a3ee5e6b4b0d3255bfef95601890afd80709";

  EXPECT_FALSE(planner_instance_->canServiceRequest(req));
}

/**
 * @brief Check that canServiceRequest(req) returns false if planner_id is empty
 */
TEST_F(CommandPlannerTest, CheckEmptyPlannerIdForServiceRequest)
{
  planning_interface::MotionPlanRequest req;
  req.planner_id = "";

  EXPECT_FALSE(planner_instance_->canServiceRequest(req));
}

/**
 * @brief Check integrity against empty input
 */
TEST_F(CommandPlannerTest, CheckPlanningContextRequestNull)
{
  moveit_msgs::msg::MotionPlanRequest req;
  moveit_msgs::msg::MoveItErrorCodes error_code;
  EXPECT_EQ(nullptr, planner_instance_->getPlanningContext(nullptr, req, error_code));
}

/**
 * @brief Check that for the announced algorithms getPlanningContext does not
 * return nullptr
 */
TEST_F(CommandPlannerTest, CheckPlanningContextRequest)
{
  moveit_msgs::msg::MotionPlanRequest req;
  moveit_msgs::msg::MoveItErrorCodes error_code;

  req.group_name = "manipulator";
  // Check for the algorithms
  std::vector<std::string> algs;
  planner_instance_->getPlanningAlgorithms(algs);

  for (const auto& alg : algs)
  {
    req.planner_id = alg;

    EXPECT_NE(nullptr, planner_instance_->getPlanningContext(nullptr, req, error_code));
  }
}

/**
 * @brief Check the description can be obtained and is not empty
 */
TEST_F(CommandPlannerTest, CheckPlanningContextDescriptionNotEmptyAndStable)
{
  std::string desc = planner_instance_->getDescription();
  EXPECT_GT(desc.length(), 0u);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
