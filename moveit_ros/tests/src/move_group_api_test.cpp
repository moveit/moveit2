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

/* Author: Sebastian Jahr
   Description: Integration tests for the move_group interface
*/

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "parameter_name_list.hpp"

class MoveGroupFixture : public testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    test_node_ = std::make_shared<rclcpp::Node>("move_group_param_test_node", node_options);
  }

  void TearDown() override
  {
  }

  std::shared_ptr<rclcpp::Node> test_node_;
};

TEST_F(MoveGroupFixture, testParamAPI)
{
  // Create parameter client
  auto params_client = std::make_shared<rclcpp::SyncParametersClient>(test_node_, "move_group");
  bool reach_param_client = params_client->wait_for_service(std::chrono::seconds(5));
  // This pattern helps with debugging if service is not available.
  if (!reach_param_client)
  {
    RCLCPP_ERROR(test_node_->get_logger(), "Couldn't reach parameter server. Is the move_group up and running?");
  }
  ASSERT_TRUE(reach_param_client);

  // GIVEN a running move_group
  // WHEN a parameter name from parameter API checked
  // THEN that parameter exists
  for (const auto& param_name : move_group_test::PARAMETER_NAME_LIST)
  {
    bool param_exists = false;
    if (params_client->wait_for_service(std::chrono::milliseconds(1)))
    {
      param_exists = params_client->has_parameter(param_name);
      if (!param_exists)
      {
        RCLCPP_ERROR(test_node_->get_logger(), "Parameter %s doesn't exists", param_name.c_str());
      }
    }
    else
    {
      RCLCPP_ERROR(test_node_->get_logger(), "Cannot read parameter %s because service couldn't be reached",
                   param_name.c_str());
    }
    EXPECT_TRUE(param_exists);
  }
}

// TODO(sjahr): Add more API tests e.g. from move_group_interface_cpp_test.cpp

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
