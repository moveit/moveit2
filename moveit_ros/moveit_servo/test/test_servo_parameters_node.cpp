/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

/*      Title     : test_servo_parameters_node.cpp
 *      Project   : moveit_servo
 *      Created   : 07/22/2020
 *      Author    : Adam Pettinger
 *      Desc      : The executable launched alongside test_servo_parameters gtest
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit_servo/servo_parameters.cpp>
#include <std_srvs/srv/trigger.hpp>
#include "test_parameter_struct.hpp"

bool LOAD_PARAMS_SUCCESS = false;
bool EXPECTED_LOAD_PARAMS_SUCCESS, GOT_EXPECTED, EQUALS_EXPECTED;

void loadResultCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*unused*/,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  // To pass (return true here), we need:
  // 1) To have gotten the parameter telling us if we should have success loading parameters
  // 2) Our success/fail loading parameters should match the expected
  response->success = (GOT_EXPECTED && LOAD_PARAMS_SUCCESS == EXPECTED_LOAD_PARAMS_SUCCESS);
}

void expectedResultCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*unused*/,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  response->success = EQUALS_EXPECTED;
}

int main(int argc, char** argv)
{
  // Init ROS objects
  rclcpp::init(argc, argv);
  static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.test_servo_parameters_node.cpp");
  auto node = std::make_shared<rclcpp::Node>("test_servo_parameters_node");

  // We need to know whether or not to expect valid parameter loading
  // This param passed alongside 'servo_params' in launch file
  node->declare_parameter<bool>("expect_valid_params", true);
  GOT_EXPECTED = node->get_parameter("expect_valid_params", EXPECTED_LOAD_PARAMS_SUCCESS);
  RCLCPP_INFO_STREAM(LOGGER,
                     "Got expect_valid_params? : " << GOT_EXPECTED << ", value = " << EXPECTED_LOAD_PARAMS_SUCCESS);

  // Create and try to load the parameters
  auto servo_parameters = std::make_shared<moveit_servo::ServoParameters>();
  LOAD_PARAMS_SUCCESS = moveit_servo::readParameters(servo_parameters, node, LOGGER);

  // Offer a service to report the success/fail of the loading
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr loading_service =
      node->create_service<std_srvs::srv::Trigger>("get_loading_result", &loadResultCB);

  // Check to see if the parameters we grabbed equal the expected ones for testing
  moveit_servo::ServoParametersPtr test_params = getTestParameters();
  EQUALS_EXPECTED = (*test_params == *servo_parameters);

  // Offer a service to report if the loading matches the test parameters
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr equal_expected_service =
      node->create_service<std_srvs::srv::Trigger>("get_equal_expected_result", &expectedResultCB);

  rclcpp::spin(node);
  rclcpp::shutdown();
}
