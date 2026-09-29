/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, MoveIt Contributors
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

/* Description: Integration tests for MoveItErrorCodes on move_group actions */

#include <chrono>
#include <memory>

#include <gtest/gtest.h>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/action/move_group_sequence.hpp>
#include <moveit_msgs/msg/motion_sequence_item.hpp>
#include <moveit_msgs/msg/motion_sequence_request.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr const char* kPandaArmGroup = "panda_arm";
constexpr const char* kPandaEeLink = "panda_link8";
constexpr const char* kPandaBaseFrame = "panda_link0";
constexpr auto kActionServerWait = 30s;
constexpr auto kActionCallWait = 30s;

/** Cartesian goal outside the Panda workspace (expects NO_IK_SOLUTION). */
geometry_msgs::msg::PoseStamped unreachableCartesianGoal()
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = kPandaBaseFrame;
  pose.pose.orientation.w = 1.0;
  pose.pose.position.y = 27.0;
  return pose;
}

/** Build a Pilz motion plan request to the unreachable goal. */
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

/** Connects to move_action and sequence_move_group for plan-only error checks. */
class MoveGroupErrorCodesFixture : public ::testing::Test
{
protected:
  /** Wait for both action servers before each test case. */
  void SetUp() override
  {
    node_ = rclcpp::Node::make_shared("move_group_error_codes_test");
    move_group_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(node_, "move_action");
    sequence_client_ =
        rclcpp_action::create_client<moveit_msgs::action::MoveGroupSequence>(node_, "sequence_move_group");

    ASSERT_TRUE(move_group_client_->wait_for_action_server(kActionServerWait)) << "move_action server not available";
    ASSERT_TRUE(sequence_client_->wait_for_action_server(kActionServerWait))
        << "sequence_move_group server not available";

    auto joint_state_sub = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
          if (!msg->position.empty())
          {
            received_joint_state_ = true;
          }
        });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_);
    const rclcpp::Time deadline = node_->get_clock()->now() + kActionServerWait;
    while (rclcpp::ok() && !received_joint_state_ && node_->get_clock()->now() < deadline)
    {
      executor.spin_some(100ms);
    }
    ASSERT_TRUE(received_joint_state_) << "Timed out waiting for /joint_states";
    (void)joint_state_sub;
  }

  bool received_joint_state_{ false };

  /** Send a plan-only action goal and return the result error_code field. */
  template <typename ActionT, typename ExtractCodeFn>
  int32_t sendPlanOnly(const typename rclcpp_action::Client<ActionT>::SharedPtr& client, typename ActionT::Goal goal,
                       const char* action_name, ExtractCodeFn extract_code)
  {
    goal.planning_options.plan_only = true;

    auto goal_handle_future = client->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(node_, goal_handle_future, kActionCallWait) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      ADD_FAILURE() << "Timed out sending " << action_name << " goal";
      return moveit_msgs::msg::MoveItErrorCodes::UNDEFINED;
    }

    const auto goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
      ADD_FAILURE() << action_name << " goal rejected";
      return moveit_msgs::msg::MoveItErrorCodes::UNDEFINED;
    }

    auto result_future = client->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(node_, result_future, kActionCallWait) != rclcpp::FutureReturnCode::SUCCESS)
    {
      ADD_FAILURE() << "Timed out waiting for " << action_name << " result";
      return moveit_msgs::msg::MoveItErrorCodes::UNDEFINED;
    }

    return extract_code(result_future.get().result);
  }

  /** Plan-only /move_action; returns response error_code.val. */
  int32_t sendMoveGroupPlanOnly(const moveit_msgs::msg::MotionPlanRequest& motion_request)
  {
    moveit_msgs::action::MoveGroup::Goal goal;
    goal.request = motion_request;
    return sendPlanOnly<moveit_msgs::action::MoveGroup>(
        move_group_client_, goal, "move_action",
        [](const moveit_msgs::action::MoveGroup::Result::SharedPtr& result) { return result->error_code.val; });
  }

  /** Plan-only /sequence_move_group; returns response.error_code.val. */
  int32_t sendSequencePlanOnly(const moveit_msgs::msg::MotionSequenceRequest& sequence_request)
  {
    moveit_msgs::action::MoveGroupSequence::Goal goal;
    goal.request = sequence_request;
    return sendPlanOnly<moveit_msgs::action::MoveGroupSequence>(
        sequence_client_, goal, "sequence_move_group",
        [](const moveit_msgs::action::MoveGroupSequence::Result::SharedPtr& result) {
          return result->response.error_code.val;
        });
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr move_group_client_;
  rclcpp_action::Client<moveit_msgs::action::MoveGroupSequence>::SharedPtr sequence_client_;
};

/** Pilz PTP via /move_action should surface NO_IK_SOLUTION, not FAILURE. */
TEST_F(MoveGroupErrorCodesFixture, PilzMoveActionPreservesNoIkSolution)
{
  const int32_t error_code = sendMoveGroupPlanOnly(pilzCartesianRequest("PTP"));
  EXPECT_EQ(error_code, moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION);
}

/** Pilz LIN via /sequence_move_group should surface NO_IK_SOLUTION. */
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

/** OMPL with an invalid group should return INVALID_GROUP_NAME. */
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

/** Run gtest with rclcpp init/shutdown. */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
