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

#include <moveit/planning_pipeline/planning_pipeline.hpp>
#include <moveit/utils/robot_model_test_utils.hpp>

namespace
{
const std::vector<std::string> REQUEST_ADAPTERS{ "planning_pipeline_test/AlwaysSuccessRequestAdapter",
                                                 "planning_pipeline_test/AlwaysSuccessRequestAdapter" };
const std::vector<std::string> RESPONSE_ADAPTERS{ "planning_pipeline_test/AlwaysSuccessResponseAdapter",
                                                  "planning_pipeline_test/AlwaysSuccessResponseAdapter" };
const std::vector<std::string> PLANNER_PLUGINS{ "planning_pipeline_test/DummyPlannerManager",
                                                "planning_pipeline_test/DummyPlannerManager" };

// Configure a pipeline from the node parameters and return the message of the thrown exception (empty if none)
std::string getConfigurationErrorMessage(const moveit::core::RobotModelConstPtr& robot_model,
                                         const std::shared_ptr<rclcpp::Node>& node,
                                         const std::string& parameter_namespace)
{
  try
  {
    planning_pipeline::PlanningPipeline pipeline(robot_model, node, parameter_namespace);
  }
  catch (const std::runtime_error& e)
  {
    return e.what();
  }
  return "";
}
}  // namespace
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

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<const moveit::core::RobotModel> robot_model_;
  planning_pipeline::PlanningPipelinePtr pipeline_ptr_{ nullptr };
};

TEST_F(TestPlanningPipeline, HappyPath)
{
  // GIVEN a valid configuration
  // WHEN the planning pipeline is configured
  // THEN it is successful
  EXPECT_NO_THROW(pipeline_ptr_ = std::make_shared<planning_pipeline::PlanningPipeline>(
                      robot_model_, node_, "", PLANNER_PLUGINS, REQUEST_ADAPTERS, RESPONSE_ADAPTERS));
  // THEN a planning pipeline exists
  EXPECT_NE(pipeline_ptr_, nullptr);
  // THEN the pipeline is inactive
  EXPECT_FALSE(pipeline_ptr_->isActive());
  // THEN the pipeline contains a valid robot model
  EXPECT_NE(pipeline_ptr_->getRobotModel(), nullptr);
  // THEN the loaded request adapter names equal the adapter names input vector
  const auto loaded_req_adapters = pipeline_ptr_->getRequestAdapterPluginNames();
  EXPECT_TRUE(std::equal(loaded_req_adapters.begin(), loaded_req_adapters.end(), REQUEST_ADAPTERS.begin()));
  // THEN the loaded request adapter names equal the adapter names input vector
  const auto loaded_res_adapters = pipeline_ptr_->getResponseAdapterPluginNames();
  EXPECT_TRUE(std::equal(loaded_res_adapters.begin(), loaded_res_adapters.end(), RESPONSE_ADAPTERS.begin()));
  // THEN the loaded planner plugin name equals the planner name input argument
  const auto loaded_planners = pipeline_ptr_->getPlannerPluginNames();
  EXPECT_TRUE(std::equal(loaded_planners.begin(), loaded_planners.end(), PLANNER_PLUGINS.begin()));

  // WHEN generatePlan is called
  // THEN A successful response is created
  planning_interface::MotionPlanResponse motion_plan_response;
  planning_interface::MotionPlanRequest motion_plan_request;
  const auto planning_scene_ptr = std::make_shared<planning_scene::PlanningScene>(robot_model_);
  EXPECT_TRUE(pipeline_ptr_->generatePlan(planning_scene_ptr, motion_plan_request, motion_plan_response));
  EXPECT_TRUE(motion_plan_response.error_code);
}

TEST_F(TestPlanningPipeline, NoPlannerPluginConfigured)
{
  // GIVEN a configuration without planner plugin
  // WHEN the pipeline is configured
  // THEN an exception is thrown
  EXPECT_THROW(pipeline_ptr_ = std::make_shared<planning_pipeline::PlanningPipeline>(
                   robot_model_, node_, "", std::vector<std::string>(), REQUEST_ADAPTERS, RESPONSE_ADAPTERS),
               std::runtime_error);

  // GIVEN a configuration with planner plugin called UNKNOWN
  // WHEN the pipeline is configured
  // THEN an exception is thrown
  EXPECT_THROW(pipeline_ptr_ = std::make_shared<planning_pipeline::PlanningPipeline>(
                   robot_model_, node_, "", std::vector<std::string>({ "UNKNOWN" }), REQUEST_ADAPTERS,
                   RESPONSE_ADAPTERS),
               std::runtime_error);
}

TEST_F(TestPlanningPipeline, LegacyPlanningPluginParameter)
{
  const std::string migration_hint = "'ompl.planning_plugin' has been replaced by 'ompl.planning_plugins'";
  const rclcpp::Parameter legacy_parameter("ompl.planning_plugin", "ompl_interface/OMPLPlanner");

  // GIVEN a configuration that sets the legacy parameter 'planning_plugin' instead of 'planning_plugins'
  const auto node = rclcpp::Node::make_shared("legacy_parameter_override_test",
                                              rclcpp::NodeOptions().parameter_overrides({ legacy_parameter }));
  // WHEN the pipeline is configured from the node parameters
  // THEN an exception is thrown that points to the renamed parameter
  std::string message = getConfigurationErrorMessage(robot_model_, node, "ompl");
  EXPECT_NE(message.find("Planning plugin name is empty"), std::string::npos) << message;
  EXPECT_NE(message.find(migration_hint), std::string::npos) << message;

  // GIVEN the same configuration on a node that declares parameters from overrides, like move_group
  const auto declaring_node = rclcpp::Node::make_shared("legacy_parameter_declared_test",
                                                        rclcpp::NodeOptions()
                                                            .allow_undeclared_parameters(true)
                                                            .automatically_declare_parameters_from_overrides(true)
                                                            .parameter_overrides({ legacy_parameter }));
  // WHEN the pipeline is configured from the node parameters
  // THEN an exception is thrown that points to the renamed parameter
  message = getConfigurationErrorMessage(robot_model_, declaring_node, "ompl");
  EXPECT_NE(message.find(migration_hint), std::string::npos) << message;

  // GIVEN the legacy parameter for a pipeline without namespace
  const auto root_node = rclcpp::Node::make_shared(
      "legacy_parameter_root_test", rclcpp::NodeOptions().parameter_overrides(
                                        { rclcpp::Parameter("planning_plugin", "ompl_interface/OMPLPlanner") }));
  // WHEN the pipeline is configured from the node parameters
  // THEN an exception is thrown that points to the renamed parameter without a namespace
  message = getConfigurationErrorMessage(robot_model_, root_node, "");
  EXPECT_NE(message.find("'planning_plugin' has been replaced by 'planning_plugins'"), std::string::npos) << message;

  // GIVEN a configuration without any planner plugin parameter
  // WHEN the pipeline is configured from the node parameters
  // THEN an exception is thrown without the migration hint
  message = getConfigurationErrorMessage(robot_model_, node_, "ompl");
  EXPECT_NE(message.find("Planning plugin name is empty"), std::string::npos) << message;
  EXPECT_EQ(message.find("has been replaced"), std::string::npos) << message;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
