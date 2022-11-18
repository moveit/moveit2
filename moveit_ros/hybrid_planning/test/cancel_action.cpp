/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, PickNik Inc.
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

/* Author: Andy Zelenak
  Description: Cancel any and all hybrid planning actions.
*/

#include <moveit_msgs/action/hybrid_planner.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/create_client.hpp>

namespace
{
using HPAction = moveit_msgs::action::HybridPlanner;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  auto node = rclcpp::Node::make_shared("action_server_cancellation", "", node_options);

  rclcpp_action::Client<HPAction>::SharedPtr client_ptr =
      rclcpp_action::create_client<HPAction>(node, "/test/hybrid_planning/run_hybrid_planning");
  if (!client_ptr->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }
  client_ptr->async_cancel_all_goals();
  RCLCPP_ERROR(node->get_logger(), "Cancellation was requested.");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
