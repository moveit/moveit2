/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, PickNik Inc.
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

/* Author: Sebastian Jahr */

#include <gtest/gtest.h>

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/utils/robot_model_test_utils.h>

class TestPlanningPipeline : public testing::Test
{
protected:
  void SetUp() override
  {
    robot_model_ = moveit::core::RobotModelBuilder("empty_robot", "base_link").build();
    node_ = rclcpp::Node::make_shared("planning_pipeline_test");
  };
  void TearDown() override
  {
  }

  std::shared_ptr<const moveit::core::RobotModel> robot_model_;
}

TEST_F(TestPlanningPipeline, HappyPath)
{
  // GIVEN a valid configuration and a valid motion plan request
  // WHEN the planning pipeline is configured
  // THEN it is successful
  // WHEN generatePlan is called
  // THEN The plugins are called in the correct order
  // THEN A successful response is created
}

TEST_F(TestPlanningPipeline, HappyPathTerminate)
{
  // GIVEN a valid config and request
  // WHEN the pipeline is started and terminate is called
  // THEN the pipeline terminates the running planner and returns a result
}

TEST_F(TestPlanningPipeline, NoPlannerPluginConfigured)
{
  // GIVEN a configuration without planner plugin
  // WHEN the pipeline is configured
  // THEN an exception is thrown
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
