/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik, Inc.
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
 *   * Neither the name of PickNik nor the names of its
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

/* Author: Anthony Baker */

#include <memory>
#include <string>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "moveit/planning_pipeline/planning_pipeline.h"
#include "moveit/utils/robot_model_test_utils.h"
#include "rclcpp/node_interfaces/node_clock_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "moveit/planning_scene/planning_scene.h"

using ::testing::_;
using ::testing::Return;
struct MockPlanningPipelineMiddlewareHandle : public planning_pipeline::PlanningPipeline::MiddlewareHandle
{
  MOCK_METHOD(void, createDisplayPathPublisher, (const std::string&), (override));
  MOCK_METHOD(void, createReceivedRequestPublisher, (const std::string&), (override));
  MOCK_METHOD(void, createContactsPublisher, (const std::string&), (override));
  MOCK_METHOD(std::string, getContactsTopicName, (), (const, override));
  MOCK_METHOD(void, resetDisplayPathPublisher, (), (override));
  MOCK_METHOD(void, resetReceivedRequestPublisher, (), (override));
  MOCK_METHOD(void, resetContactsPublisher, (), (override));
  MOCK_METHOD(bool, hasParameter, (const std::string&), (const, override));
  MOCK_METHOD(void, getParameter, (const std::string&, std::string&), (const, override));
  MOCK_METHOD(void, createPlannerPlugin, (const moveit::core::RobotModelConstPtr&), (override));
  MOCK_METHOD(const planning_interface::PlannerManagerPtr&, getPlannerManager, (), (const, override));
  MOCK_METHOD(void, createAdapterPlugins, (const std::vector<std::string>&), (override));
  MOCK_METHOD(bool, plan,
              (const planning_scene::PlanningSceneConstPtr&, const planning_interface::MotionPlanRequest&,
               planning_interface::MotionPlanResponse&, std::vector<std::size_t>&),
              (override));
  MOCK_METHOD(void, terminatePlannerPlugin, (), (override));
  MOCK_METHOD(void, publishReceivedRequest, (const planning_interface::MotionPlanRequest&), (override));
  MOCK_METHOD(void, publishContactsMarkerArray, (const visualization_msgs::msg::MarkerArray&), (override));
  MOCK_METHOD(void, publishDisplayPathTrejectory, (const moveit_msgs::msg::DisplayTrajectory&), (override));
};

TEST(PlanningPipelineTests, construction)
{
  auto mock_middleware_handle{ std::make_unique<MockPlanningPipelineMiddlewareHandle>() };

  const auto parameter_namespace{ "Test" };

  EXPECT_CALL(*mock_middleware_handle, createPlannerPlugin(_)).Times(1);
  EXPECT_CALL(*mock_middleware_handle, createAdapterPlugins(_)).Times(1);
  EXPECT_CALL(*mock_middleware_handle, resetDisplayPathPublisher).Times(0);
  EXPECT_CALL(*mock_middleware_handle, createDisplayPathPublisher(_)).Times(1);
  EXPECT_CALL(*mock_middleware_handle, resetContactsPublisher).Times(0);
  EXPECT_CALL(*mock_middleware_handle, createContactsPublisher(_)).Times(1);

  // PlanningPipeline::configure is called during construction.
  planning_pipeline::PlanningPipeline pp{ moveit::core::loadTestingRobotModel("panda"),
                                          std::move(mock_middleware_handle), parameter_namespace };
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
