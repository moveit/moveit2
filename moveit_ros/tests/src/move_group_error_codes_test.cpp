/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026
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
 *   * Neither the name of the copyright holder nor the names of its
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

#include <chrono>
#include <memory>

#include <gtest/gtest.h>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/action/move_group_sequence.hpp>
#include <moveit_msgs/msg/motion_sequence_item.hpp>
#include <moveit_msgs/msg/motion_sequence_request.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr const char* kPandaArmGroup = "panda_arm";
constexpr const char* kPandaEeLink = "panda_link8";
constexpr const char* kPandaBaseFrame = "panda_link0";

geometry_msgs::msg::PoseStamped unreachableCartesianGoal()
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = kPandaBaseFrame;
  pose.pose.orientation.w = 1.0;
  pose.pose.position.y = 27.0;
  return pose;
}

moveit_msgs::msg::MotionPlanRequest pilzCartesianRequest(const std::string& planner_id)
{
  moveit_msgs::msg::MotionPlanRequest request;
  request.group_name = kPandaArmGroup;
  request.pipeline_id = "pilz_industrial_motion_planner";
  request.planner_id = planner_id;
  request.allowed_planning_time = 10.0;
  request.max_velocity_scaling_factor = 0.5;
  request.max_acceleration_scaling_factor = 0.5;
  request.goal_constraints.push_back(
      kinematic_constraints::constructGoalConstraints(kPandaEeLink, unreachableCartesianGoal()));
  return request;
}

class MoveGroupErrorCodesFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = rclcpp::Node::make_shared("move_group_error_codes_test");
    move_group_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(node_, "move_action");
    sequence_client_ =
        rclcpp_action::create_client<moveit_msgs::action::MoveGroupSequence>(node_, "sequence_move_group");

    ASSERT_TRUE(move_group_client_->wait_for_action_server(120s))
        << "move_action server not available";
    ASSERT_TRUE(sequence_client_->wait_for_action_server(120s))
        << "sequence_move_group server not available";
  }

  int32_t sendMoveGroupPlanOnly(const moveit_msgs::msg::MotionPlanRequest& motion_request)
  {
    moveit_msgs::action::MoveGroup::Goal goal;
    goal.request = motion_request;
    goal.planning_options.plan_only = true;

    auto goal_handle_future = move_group_client_->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(node_, goal_handle_future, 120s) != rclcpp::FutureReturnCode::SUCCESS)
    {
      ADD_FAILURE() << "Timed out sending move_action goal";
      return moveit_msgs::msg::MoveItErrorCodes::UNDEFINED;
    }

    const auto goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
      ADD_FAILURE() << "move_action goal rejected";
      return moveit_msgs::msg::MoveItErrorCodes::UNDEFINED;
    }

    auto result_future = move_group_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(node_, result_future, 120s) != rclcpp::FutureReturnCode::SUCCESS)
    {
      ADD_FAILURE() << "Timed out waiting for move_action result";
      return moveit_msgs::msg::MoveItErrorCodes::UNDEFINED;
    }

    return result_future.get().result->error_code.val;
  }

  int32_t sendSequencePlanOnly(const moveit_msgs::msg::MotionSequenceRequest& sequence_request)
  {
    moveit_msgs::action::MoveGroupSequence::Goal goal;
    goal.request = sequence_request;
    goal.planning_options.plan_only = true;

    auto goal_handle_future = sequence_client_->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(node_, goal_handle_future, 120s) != rclcpp::FutureReturnCode::SUCCESS)
    {
      ADD_FAILURE() << "Timed out sending sequence_move_group goal";
      return moveit_msgs::msg::MoveItErrorCodes::UNDEFINED;
    }

    const auto goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
      ADD_FAILURE() << "sequence_move_group goal rejected";
      return moveit_msgs::msg::MoveItErrorCodes::UNDEFINED;
    }

    auto result_future = sequence_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(node_, result_future, 120s) != rclcpp::FutureReturnCode::SUCCESS)
    {
      ADD_FAILURE() << "Timed out waiting for sequence_move_group result";
      return moveit_msgs::msg::MoveItErrorCodes::UNDEFINED;
    }

    return result_future.get().result->response.error_code.val;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr move_group_client_;
  rclcpp_action::Client<moveit_msgs::action::MoveGroupSequence>::SharedPtr sequence_client_;
};

TEST_F(MoveGroupErrorCodesFixture, PilzMoveActionPreservesNoIkSolution)
{
  const int32_t error_code = sendMoveGroupPlanOnly(pilzCartesianRequest("PTP"));
  EXPECT_EQ(error_code, moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION);
}

TEST_F(MoveGroupErrorCodesFixture, PilzSequencePreservesNoIkSolution)
{
  moveit_msgs::msg::MotionSequenceItem item;
  item.req = pilzCartesianRequest("LIN");
  item.blend_radius = 0.0;

  moveit_msgs::msg::MotionSequenceRequest sequence_request;
  sequence_request.items.push_back(item);

  const int32_t error_code = sendSequencePlanOnly(sequence_request);
  EXPECT_EQ(error_code, moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION);
}

TEST_F(MoveGroupErrorCodesFixture, OmplMoveActionPreservesInvalidGroupName)
{
  moveit_msgs::msg::MotionPlanRequest request;
  request.group_name = "nonexistent_group";
  request.pipeline_id = "ompl";
  request.planner_id = "RRTConnect";
  request.allowed_planning_time = 5.0;

  const int32_t error_code = sendMoveGroupPlanOnly(request);
  EXPECT_EQ(error_code, moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME);
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
