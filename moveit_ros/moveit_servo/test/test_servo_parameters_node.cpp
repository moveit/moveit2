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

bool load_params_success_ = false;

void resultCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  response->success = load_params_success_;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.test_servo_parameters_node.cpp");

  rclcpp::NodeOptions node_options;
  auto node = std::make_shared<rclcpp::Node>("servo_demo_node", node_options);
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  node->declare_parameter<std::string>("some_test_param", "Oscar Kilo");
  std::string the_param;
  bool got_param = node->get_parameter("some_test_param", the_param);
  RCLCPP_WARN_STREAM(LOGGER, "Got param? : " << got_param << ". What was it? " << the_param);

  auto servo_parameters = std::make_shared<moveit_servo::ServoParameters>();

  load_params_success_ = moveit_servo::readParameters(*node, LOGGER, *servo_parameters);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service =
    node->create_service<std_srvs::srv::Trigger>("get_result", &resultCB);

  rclcpp::spin(node);
  rclcpp::shutdown();
}