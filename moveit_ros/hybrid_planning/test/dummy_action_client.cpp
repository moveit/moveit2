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

/* Author: Sebastian Jahr
 */

#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <moveit/hybrid_planning/global_planner_component.h>
#include <moveit/hybrid_planning/hybrid_planning_manager.h>
#include <moveit/hybrid_planning/local_planner_component.h>

#include <moveit_msgs/action/run_hybrid_planning.hpp>
#include <moveit_msgs/action/plan_global_trajectory.hpp>

const rclcpp::Logger LOGGER = rclcpp::get_logger("dummy_hybrid_planning_client");

class HybridPlanningDemo
{
public:
  HybridPlanningDemo(const rclcpp::Node::SharedPtr& node)
  {
    node_ = node;
    hp_action_client_ =
        rclcpp_action::create_client<moveit_msgs::action::RunHybridPlanning>(node_, "hybrid_planning_request");
  }

  void run()
  {
    if (!hp_action_client_->wait_for_action_server(std::chrono::seconds(20)))
    {
      RCLCPP_ERROR(LOGGER, "Hybrid planning action server not available after waiting");
      return;
    }


    auto send_goal_options = rclcpp_action::Client<moveit_msgs::action::RunHybridPlanning>::SendGoalOptions();
    send_goal_options.result_callback =
        [](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::RunHybridPlanning>::WrappedResult& result) {
          switch (result.code)
          {
            case rclcpp_action::ResultCode::SUCCEEDED:
              RCLCPP_INFO(LOGGER, "Hybrid planning goal succeded");
              break;
            case rclcpp_action::ResultCode::ABORTED:
              RCLCPP_ERROR(LOGGER, "Hybrid planning goal was aborted");
              return;
            case rclcpp_action::ResultCode::CANCELED:
              RCLCPP_ERROR(LOGGER, "Hybrid planning goal was canceled");
              return;
            default:
              RCLCPP_ERROR(LOGGER, "Unknown hybrid planning result code");
              return;
              RCLCPP_INFO(LOGGER, "Hybrid planning result received");
          }
        };
    send_goal_options.feedback_callback =
        [](rclcpp_action::ClientGoalHandle<moveit_msgs::action::RunHybridPlanning>::SharedPtr,
           const std::shared_ptr<const moveit_msgs::action::RunHybridPlanning::Feedback> feedback) {
          RCLCPP_INFO(LOGGER, feedback->feedback);
        };

    auto goal_msg = moveit_msgs::action::RunHybridPlanning::Goal();
    RCLCPP_INFO(LOGGER, "Sending hybrid planning goal");
    // Ask server to achieve some goal and wait until it's accepted
    auto goal_handle_future = hp_action_client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<moveit_msgs::action::RunHybridPlanning>::SharedPtr hp_action_client_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("hybrid_planner_demo", "", node_options);

  HybridPlanningDemo demo(node);
  std::thread run_demo([&demo]() {
    rclcpp::sleep_for(std::chrono::seconds(5));
    demo.run();
  });

  rclcpp::spin(node);
  run_demo.join();

  return 0;
}