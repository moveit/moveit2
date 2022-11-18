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
   Desc:   Launch hybrid planning and test basic functionality
*/

#include <gtest/gtest.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>

#include <moveit_msgs/action/hybrid_planner.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>

namespace moveit_hybrid_planning
{
using namespace std::chrono_literals;

class HybridPlanningFixture : public ::testing::Test
{
public:
  HybridPlanningFixture() : node_(std::make_shared<rclcpp::Node>("hybrid_planning_testing"))
  {
    action_successful_ = false;
    action_aborted_ = false;
    action_complete_ = false;

    executor_.add_node(node_);

    std::string hybrid_planning_action_name = "";
    node_->declare_parameter("hybrid_planning_action_name", "");
    if (node_->has_parameter("hybrid_planning_action_name"))
    {
      node_->get_parameter<std::string>("hybrid_planning_action_name", hybrid_planning_action_name);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "hybrid_planning_action_name parameter was not defined");
      std::exit(EXIT_FAILURE);
    }

    hp_action_client_ =
        rclcpp_action::create_client<moveit_msgs::action::HybridPlanner>(node_, hybrid_planning_action_name);

    // Add new collision object as soon as global trajectory is available.
    global_solution_subscriber_ = node_->create_subscription<moveit_msgs::msg::MotionPlanResponse>(
        "global_trajectory", rclcpp::SystemDefaultsQoS(),
        [this](const moveit_msgs::msg::MotionPlanResponse::SharedPtr /* unused */) {});

    RCLCPP_INFO(node_->get_logger(), "Initialize Planning Scene Monitor");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());

    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, "robot_description",
                                                                                             "planning_scene_monitor");
    if (!planning_scene_monitor_->getPlanningScene())
    {
      RCLCPP_ERROR(node_->get_logger(), "The planning scene was not retrieved!");
      std::exit(EXIT_FAILURE);
    }
    else
    {
      planning_scene_monitor_->startStateMonitor();
      planning_scene_monitor_->setPlanningScenePublishingFrequency(100);
      planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                            "/planning_scene");
      planning_scene_monitor_->startSceneMonitor();
    }

    if (!planning_scene_monitor_->waitForCurrentRobotState(node_->now(), 5))
    {
      RCLCPP_ERROR(node_->get_logger(), "Timeout when waiting for /joint_states updates. Is the robot running?");
      std::exit(EXIT_FAILURE);
    }

    if (!hp_action_client_->wait_for_action_server(20s))
    {
      RCLCPP_ERROR(node_->get_logger(), "Hybrid planning action server not available after waiting");
      std::exit(EXIT_FAILURE);
    }

    // Setup motion planning goal taken from motion_planning_api tutorial
    const std::string planning_group = "panda_arm";
    robot_model_loader::RobotModelLoader robot_model_loader(node_, "robot_description");
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();

    // Create a RobotState and JointModelGroup
    const auto robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(planning_group);

    // Configure a valid robot state
    robot_state->setToDefaultValues(joint_model_group, "ready");
    robot_state->update();
    // Lock the planning scene as briefly as possible
    {
      planning_scene_monitor::LockedPlanningSceneRW locked_planning_scene(planning_scene_monitor_);
      locked_planning_scene->setCurrentState(*robot_state);
    }

    const moveit::core::RobotState goal_state([robot_model, joint_model_group]() {
      moveit::core::RobotState goal_state(robot_model);
      std::vector<double> joint_values = { 0.0, 0.0, 0.0, 0.0, 0.0, 1.571, 0.785 };
      goal_state.setJointGroupPositions(joint_model_group, joint_values);
      return goal_state;
    }());

    // Create desired motion goal
    const moveit_msgs::msg::MotionPlanRequest goal_motion_request(
        [robot_state, planning_group, goal_state, joint_model_group]() {
          moveit_msgs::msg::MotionPlanRequest goal_motion_request;
          moveit::core::robotStateToRobotStateMsg(*robot_state, goal_motion_request.start_state);
          goal_motion_request.group_name = planning_group;
          goal_motion_request.num_planning_attempts = 10;
          goal_motion_request.max_velocity_scaling_factor = 0.1;
          goal_motion_request.max_acceleration_scaling_factor = 0.1;
          goal_motion_request.allowed_planning_time = 2.0;
          goal_motion_request.planner_id = "ompl";
          goal_motion_request.pipeline_id = "ompl";
          goal_motion_request.goal_constraints.resize(1);
          goal_motion_request.goal_constraints[0] =
              kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
          return goal_motion_request;
        }());

    // Create Hybrid Planning action request
    moveit_msgs::msg::MotionSequenceItem sequence_item;
    sequence_item.req = goal_motion_request;
    sequence_item.blend_radius = 0.0;  // Single goal

    moveit_msgs::msg::MotionSequenceRequest sequence_request;
    sequence_request.items.push_back(sequence_item);

    goal_action_request_ = moveit_msgs::action::HybridPlanner::Goal();
    goal_action_request_.planning_group = planning_group;
    goal_action_request_.motion_sequence = sequence_request;

    send_goal_options_ = rclcpp_action::Client<moveit_msgs::action::HybridPlanner>::SendGoalOptions();
    send_goal_options_.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::HybridPlanner>::WrappedResult& result) {
          switch (result.code)
          {
            case rclcpp_action::ResultCode::SUCCEEDED:
              action_successful_ = true;
              RCLCPP_INFO(node_->get_logger(), "Hybrid planning goal succeeded");
              break;
            case rclcpp_action::ResultCode::ABORTED:
              action_aborted_ = true;
              RCLCPP_ERROR(node_->get_logger(), "Hybrid planning goal was aborted");
              break;
            case rclcpp_action::ResultCode::CANCELED:
              RCLCPP_ERROR(node_->get_logger(), "Hybrid planning goal was canceled");
              break;
            default:
              RCLCPP_ERROR(node_->get_logger(), "Unknown hybrid planning result code");
              break;
          }
          action_complete_ = true;
          return;
        };
    send_goal_options_.feedback_callback =
        [this](rclcpp_action::ClientGoalHandle<moveit_msgs::action::HybridPlanner>::SharedPtr /*unused*/,
               const std::shared_ptr<const moveit_msgs::action::HybridPlanner::Feedback> feedback) {
          RCLCPP_INFO(node_->get_logger(), feedback->feedback.c_str());
        };
  }

  rclcpp::executors::MultiThreadedExecutor executor_;

protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<moveit_msgs::action::HybridPlanner>::SharedPtr hp_action_client_;
  rclcpp::Subscription<moveit_msgs::msg::MotionPlanResponse>::SharedPtr global_solution_subscriber_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  std::atomic_bool action_successful_;
  std::atomic_bool action_complete_;
  std::atomic_bool action_aborted_;

  // Action request
  moveit_msgs::action::HybridPlanner::Goal goal_action_request_;
  rclcpp_action::Client<moveit_msgs::action::HybridPlanner>::SendGoalOptions send_goal_options_;
};  // class HybridPlanningFixture

// Make a hybrid planning request and verify it completes
TEST_F(HybridPlanningFixture, ActionCompletion)
{
  std::thread run_thread([this]() {
    // Send the goal
    auto goal_handle_future = hp_action_client_->async_send_goal(goal_action_request_, send_goal_options_);
  });

  rclcpp::Rate rate(10);
  while (!action_complete_)
  {
    executor_.spin_once();
    rate.sleep();
  }
  run_thread.join();
  ASSERT_TRUE(action_successful_);
}

// Make a hybrid planning request then abort it
TEST_F(HybridPlanningFixture, ActionAbortion)
{
  std::thread run_thread([this]() {
    // Send the goal
    auto goal_handle_future = hp_action_client_->async_send_goal(goal_action_request_, send_goal_options_);

    // Cancel the goal
    hp_action_client_->async_cancel_all_goals();
  });

  rclcpp::Rate rate(10);
  while (!action_complete_)
  {
    executor_.spin_once();
    rate.sleep();
  }
  run_thread.join();
  ASSERT_FALSE(action_successful_);
  ASSERT_TRUE(action_aborted_);
}
}  // namespace moveit_hybrid_planning

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);

  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
