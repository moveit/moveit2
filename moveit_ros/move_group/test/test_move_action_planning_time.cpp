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

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include <moveit/move_group/capability_names.hpp>
#include <moveit/move_group/move_group_capability.hpp>
#include <moveit/move_group/move_group_context.hpp>
#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "move_action_test_plugins.hpp"

namespace
{
constexpr char URDF[] = R"(<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link"/>
  <link name="link1">
    <collision>
      <origin xyz="0.5 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10.0" velocity="1.0"/>
  </joint>
</robot>)";

constexpr char SRDF[] = R"(<?xml version="1.0"?>
<robot name="test_robot">
  <group name="arm">
    <joint name="joint1"/>
  </group>
</robot>)";

class MoveGroupHarness
{
public:
  using MoveGroupAction = moveit_msgs::action::MoveGroup;
  using GoalHandle = rclcpp_action::ClientGoalHandle<MoveGroupAction>;
  using WrappedResult = GoalHandle::WrappedResult;

  MoveGroupHarness()
  {
    int argc = 0;
    rclcpp::init(argc, nullptr);

    rclcpp::NodeOptions opts;
    opts.automatically_declare_parameters_from_overrides(true);
    opts.parameter_overrides({
        rclcpp::Parameter("robot_description", std::string(URDF)),
        rclcpp::Parameter("robot_description_semantic", std::string(SRDF)),
        rclcpp::Parameter("planning_pipelines.pipeline_names", std::vector<std::string>{ "test_pipeline" }),
        rclcpp::Parameter("test_pipeline.planning_plugins",
                          std::vector<std::string>{ "move_group_test/MockPlannerManager" }),
        rclcpp::Parameter("moveit_controller_manager", std::string("move_group_test/MockControllerManager")),
        rclcpp::Parameter("trajectory_execution.manage_controllers", false),
        rclcpp::Parameter("trajectory_execution.allowed_execution_duration_scaling", 2.0),
        rclcpp::Parameter("trajectory_execution.allowed_goal_duration_margin", 1.0),
        rclcpp::Parameter("trajectory_execution.allowed_start_tolerance", 0.05),
    });

    server_node_ = std::make_shared<rclcpp::Node>("test_move_group_server", opts);
    moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(server_node_);

    {
      planning_scene_monitor::LockedPlanningSceneRW lscene(moveit_cpp_->getPlanningSceneMonitorNonConst());
      moveit::core::RobotState& state = lscene->getCurrentStateNonConst();
      state.setToDefaultValues();
      lscene->setCurrentState(state);
    }

    context_ = std::make_shared<move_group::MoveGroupContext>(moveit_cpp_, "test_pipeline", true, false);
    EXPECT_TRUE(context_->status());

    capability_loader_ = std::make_unique<pluginlib::ClassLoader<move_group::MoveGroupCapability>>(
        "moveit_ros_move_group", "move_group::MoveGroupCapability");
    move_action_cap_ = capability_loader_->createSharedInstance("move_group/MoveGroupMoveAction");
    move_action_cap_->setContext(context_);
    move_action_cap_->initialize();

    client_node_ = std::make_shared<rclcpp::Node>("test_move_group_client");
    joint_state_pub_ = client_node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    planning_scene_pub_ = client_node_->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);
    auto publish_joint_state = [this]() {
      sensor_msgs::msg::JointState js;
      js.header.stamp = client_node_->now();
      js.name = { "joint1" };
      js.position = { 0.0 };
      joint_state_pub_->publish(js);
    };
    joint_state_timer_ = client_node_->create_wall_timer(std::chrono::milliseconds(50), publish_joint_state);
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(server_node_);
    executor_->add_node(client_node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
    publish_joint_state();
    EXPECT_TRUE(moveit_cpp_->getPlanningSceneMonitorNonConst()->waitForCurrentRobotState(server_node_->now(), 5.0));

    client_ = rclcpp_action::create_client<MoveGroupAction>(client_node_, move_group::MOVE_ACTION);
    EXPECT_TRUE(client_->wait_for_action_server(std::chrono::seconds(10)));

    move_group_test::getMockPlannerControl().reset();
    move_group_test::getMockControllerControl().reset();
  }

  MoveGroupAction::Goal makeGoal(double target_pos, bool plan_only, bool replan = false, int replan_attempts = 1,
                                 double replan_delay = 0.0)
  {
    MoveGroupAction::Goal goal;
    goal.request.group_name = "arm";
    goal.request.allowed_planning_time = 1.0;
    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name = "joint1";
    jc.position = target_pos;
    jc.tolerance_above = 0.01;
    jc.tolerance_below = 0.01;
    jc.weight = 1.0;
    moveit_msgs::msg::Constraints c;
    c.joint_constraints.push_back(jc);
    goal.request.goal_constraints.push_back(c);
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
    goal.planning_options.plan_only = plan_only;
    goal.planning_options.replan = replan;
    goal.planning_options.replan_attempts = replan_attempts;
    goal.planning_options.replan_delay = replan_delay;
    return goal;
  }

  GoalHandle::SharedPtr sendGoalBounded(const MoveGroupAction::Goal& goal)
  {
    auto goal_future = client_->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    {
      ADD_FAILURE() << "Timed out waiting for MoveGroup goal acceptance";
      return nullptr;
    }
    auto goal_handle = goal_future.get();
    if (!goal_handle)
    {
      ADD_FAILURE() << "MoveGroup goal was rejected";
      return nullptr;
    }
    return goal_handle;
  }

  std::optional<WrappedResult> waitForResultBounded(const GoalHandle::SharedPtr& goal_handle)
  {
    if (!goal_handle)
    {
      return std::nullopt;
    }
    auto result_future = client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    {
      ADD_FAILURE() << "Timed out waiting for MoveGroup action result";
      return std::nullopt;
    }
    return result_future.get();
  }

  std::optional<WrappedResult> sendAndWait(const MoveGroupAction::Goal& goal)
  {
    auto goal_handle = sendGoalBounded(goal);
    if (!goal_handle)
    {
      return std::nullopt;
    }
    return waitForResultBounded(goal_handle);
  }

  void publishBlockingBoxAtHalfRad()
  {
    moveit_msgs::msg::CollisionObject box;
    box.id = "blocking_box";
    box.header.frame_id = "base_link";
    box.operation = moveit_msgs::msg::CollisionObject::ADD;
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = shape_msgs::msg::SolidPrimitive::BOX;
    prim.dimensions = { 0.3, 0.3, 0.3 };
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.44;
    pose.position.y = 0.24;
    pose.position.z = 0.0;
    pose.orientation.w = 1.0;
    box.primitives.push_back(prim);
    box.primitive_poses.push_back(pose);

    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff = true;
    scene.world.collision_objects.push_back(box);
    planning_scene_pub_->publish(scene);
  }

  rclcpp::Node::SharedPtr server_node_;
  rclcpp::Node::SharedPtr client_node_;
  moveit_cpp::MoveItCppPtr moveit_cpp_;
  move_group::MoveGroupContextPtr context_;
  std::unique_ptr<pluginlib::ClassLoader<move_group::MoveGroupCapability>> capability_loader_;
  move_group::MoveGroupCapabilityPtr move_action_cap_;
  rclcpp_action::Client<MoveGroupAction>::SharedPtr client_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;
  rclcpp::TimerBase::SharedPtr joint_state_timer_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
};

// The upstream action callback uses a detached worker that can outlive its result.
// Keep its dependencies alive until process exit; this does not assert worker completion.
MoveGroupHarness& processHarness()
{
  static auto* s_harness = new MoveGroupHarness;
  return *s_harness;
}

TEST(MoveActionPlanningTimeTest, PlanOnlyReportsPlanningTime)
{
  auto& h = processHarness();
  auto& planner = move_group_test::getMockPlannerControl();
  auto& controller = move_group_test::getMockControllerControl();
  planner.push({ true, moveit_msgs::msg::MoveItErrorCodes::SUCCESS, 0.125 });

  const auto wrapped = h.sendAndWait(h.makeGoal(0.5, /*plan_only=*/true));
  ASSERT_TRUE(wrapped.has_value());
  ASSERT_EQ(wrapped->code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(wrapped->result->error_code.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
  EXPECT_FALSE(wrapped->result->planned_trajectory.joint_trajectory.points.empty());
  EXPECT_DOUBLE_EQ(wrapped->result->planning_time, 0.125);
  EXPECT_EQ(planner.call_count.load(), 1);
  EXPECT_EQ(controller.dispatch_count.load(), 0);
}

TEST(MoveActionPlanningTimeTest, PlanAndExecuteReportsPlanningTime)
{
  auto& h = processHarness();
  auto& planner = move_group_test::getMockPlannerControl();
  auto& controller = move_group_test::getMockControllerControl();
  planner.push({ true, moveit_msgs::msg::MoveItErrorCodes::SUCCESS, 0.250 });

  const auto wrapped = h.sendAndWait(h.makeGoal(0.5, /*plan_only=*/false));
  ASSERT_TRUE(wrapped.has_value());
  ASSERT_EQ(wrapped->code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(wrapped->result->error_code.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
  EXPECT_FALSE(wrapped->result->planned_trajectory.joint_trajectory.points.empty());
  EXPECT_DOUBLE_EQ(wrapped->result->planning_time, 0.250);
  EXPECT_EQ(planner.call_count.load(), 1);
  EXPECT_EQ(controller.dispatch_count.load(), 1);

  const auto dispatched = controller.getLastTrajectory();
  ASSERT_FALSE(dispatched.joint_trajectory.points.empty());
  ASSERT_EQ(dispatched.joint_trajectory.points.back().positions.size(), 1u);
  EXPECT_DOUBLE_EQ(dispatched.joint_trajectory.points.back().positions[0], 0.5);
}

TEST(MoveActionPlanningTimeTest, AlreadySatisfiedGoalSkipsPlanningWithZeroTime)
{
  auto& h = processHarness();
  auto& planner = move_group_test::getMockPlannerControl();
  auto& controller = move_group_test::getMockControllerControl();

  const auto wrapped = h.sendAndWait(h.makeGoal(0.0, /*plan_only=*/false));
  ASSERT_TRUE(wrapped.has_value());
  ASSERT_EQ(wrapped->code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(wrapped->result->error_code.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
  EXPECT_DOUBLE_EQ(wrapped->result->planning_time, 0.0);
  EXPECT_EQ(planner.call_count.load(), 0);
  EXPECT_EQ(controller.dispatch_count.load(), 0);
}

TEST(MoveActionPlanningTimeTest, PlanningFailureReportsAttemptPlanningTime)
{
  auto& h = processHarness();
  auto& planner = move_group_test::getMockPlannerControl();
  auto& controller = move_group_test::getMockControllerControl();
  planner.push({ false, moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED, 0.375 });

  const auto wrapped = h.sendAndWait(h.makeGoal(0.5, /*plan_only=*/false));
  ASSERT_TRUE(wrapped.has_value());
  ASSERT_EQ(wrapped->code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(wrapped->result->error_code.val, moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED);
  EXPECT_DOUBLE_EQ(wrapped->result->planning_time, 0.375);
  EXPECT_EQ(planner.call_count.load(), 1);
  EXPECT_EQ(controller.dispatch_count.load(), 0);
}

TEST(MoveActionPlanningTimeTest, ExecutionFailureReportsControlFailedAndPlanningTime)
{
  auto& h = processHarness();
  auto& planner = move_group_test::getMockPlannerControl();
  auto& controller = move_group_test::getMockControllerControl();
  planner.push({ true, moveit_msgs::msg::MoveItErrorCodes::SUCCESS, 0.180 });
  controller.push({ moveit_controller_manager::ExecutionStatus::ABORTED, 0, nullptr });

  const auto wrapped = h.sendAndWait(h.makeGoal(0.5, /*plan_only=*/false));
  ASSERT_TRUE(wrapped.has_value());
  ASSERT_EQ(wrapped->code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(wrapped->result->error_code.val, moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED);
  EXPECT_DOUBLE_EQ(wrapped->result->planning_time, 0.180);
  EXPECT_EQ(planner.call_count.load(), 1);
  EXPECT_EQ(controller.dispatch_count.load(), 1);
}

TEST(MoveActionPlanningTimeTest, ReplanningReportsCumulativePlanningTime)
{
  auto& h = processHarness();
  auto& planner = move_group_test::getMockPlannerControl();
  auto& controller = move_group_test::getMockControllerControl();
  planner.push({ false, moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED, 0.111 });
  planner.push({ true, moveit_msgs::msg::MoveItErrorCodes::SUCCESS, 0.222 });

  const auto wrapped = h.sendAndWait(h.makeGoal(0.5, /*plan_only=*/false, /*replan=*/true, /*replan_attempts=*/2));
  ASSERT_TRUE(wrapped.has_value());
  ASSERT_EQ(wrapped->code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(wrapped->result->error_code.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
  EXPECT_FALSE(wrapped->result->planned_trajectory.joint_trajectory.points.empty());
  EXPECT_NEAR(wrapped->result->planning_time, 0.333, 1e-9);
  EXPECT_EQ(planner.call_count.load(), 2);
  EXPECT_EQ(controller.dispatch_count.load(), 1);
}

TEST(MoveActionPlanningTimeTest, SuccessfulFirstPlanInvalidatedThenFailedReplanExcludesExecAndDelay)
{
  auto& h = processHarness();
  auto& planner = move_group_test::getMockPlannerControl();
  auto& controller = move_group_test::getMockControllerControl();
  planner.push({ true, moveit_msgs::msg::MoveItErrorCodes::SUCCESS, 0.111 });
  planner.push({ false, moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED, 0.222 });
  controller.push(
      { moveit_controller_manager::ExecutionStatus::SUCCEEDED, 300, [&h]() { h.publishBlockingBoxAtHalfRad(); } });

  const auto wrapped = h.sendAndWait(
      h.makeGoal(0.5, /*plan_only=*/false, /*replan=*/true, /*replan_attempts=*/2, /*replan_delay=*/0.25));
  ASSERT_TRUE(wrapped.has_value());
  ASSERT_EQ(wrapped->code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(wrapped->result->error_code.val, moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED);
  // Sum of planner durations (0.111 + 0.222), excluding 300ms execution and 250ms replan_delay.
  EXPECT_NEAR(wrapped->result->planning_time, 0.333, 1e-9);
  EXPECT_EQ(planner.call_count.load(), 2);
  EXPECT_EQ(controller.dispatch_count.load(), 1);
}

TEST(MoveActionPlanningTimeTest, CancellationDuringExecutionReportsCanceledAndPlanningTime)
{
  auto& h = processHarness();
  auto& planner = move_group_test::getMockPlannerControl();
  auto& controller = move_group_test::getMockControllerControl();
  planner.push({ true, moveit_msgs::msg::MoveItErrorCodes::SUCCESS, 0.150 });
  controller.push({ moveit_controller_manager::ExecutionStatus::SUCCEEDED, 500, nullptr });

  auto goal_handle = h.sendGoalBounded(h.makeGoal(0.5, /*plan_only=*/false));
  ASSERT_NE(goal_handle, nullptr);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (controller.dispatch_count.load() == 0 && std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(controller.dispatch_count.load(), 1);

  auto cancel_future = h.client_->async_cancel_goal(goal_handle);
  ASSERT_EQ(cancel_future.wait_for(std::chrono::seconds(5)), std::future_status::ready);

  const auto wrapped = h.waitForResultBounded(goal_handle);
  ASSERT_TRUE(wrapped.has_value());
  EXPECT_EQ(wrapped->code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(wrapped->result->error_code.val, moveit_msgs::msg::MoveItErrorCodes::PREEMPTED);
  EXPECT_GE(controller.cancel_count.load(), 1);
  EXPECT_DOUBLE_EQ(wrapped->result->planning_time, 0.150);
}

TEST(MoveActionPlanningTimeTest, NoStalePlanningTimeAcrossRequests)
{
  auto& h = processHarness();
  auto& planner = move_group_test::getMockPlannerControl();
  auto& controller = move_group_test::getMockControllerControl();
  planner.push({ true, moveit_msgs::msg::MoveItErrorCodes::SUCCESS, 0.250 });

  const auto first = h.sendAndWait(h.makeGoal(0.5, /*plan_only=*/false));
  ASSERT_TRUE(first.has_value());
  ASSERT_EQ(first->code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_DOUBLE_EQ(first->result->planning_time, 0.250);
  EXPECT_EQ(controller.dispatch_count.load(), 1);

  const auto second = h.sendAndWait(h.makeGoal(0.0, /*plan_only=*/false));
  ASSERT_TRUE(second.has_value());
  ASSERT_EQ(second->code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_DOUBLE_EQ(second->result->planning_time, 0.0);
  EXPECT_EQ(controller.dispatch_count.load(), 1);

  planner.push({ true, moveit_msgs::msg::MoveItErrorCodes::SUCCESS, 0.050 });
  const auto third = h.sendAndWait(h.makeGoal(0.5, /*plan_only=*/false));
  ASSERT_TRUE(third.has_value());
  ASSERT_EQ(third->code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_DOUBLE_EQ(third->result->planning_time, 0.050);
  EXPECT_EQ(controller.dispatch_count.load(), 2);
}

}  // namespace

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  if (::testing::GTEST_FLAG(list_tests))
  {
    return RUN_ALL_TESTS();
  }
  const std::string filter = ::testing::GTEST_FLAG(filter);
  const auto* tests = ::testing::UnitTest::GetInstance();
  bool single_test = false;
  for (int i = 0; i < tests->total_test_suite_count(); ++i)
  {
    const auto* suite = tests->GetTestSuite(i);
    for (int j = 0; j < suite->total_test_count(); ++j)
    {
      single_test |= filter == std::string(suite->name()) + "." + suite->GetTestInfo(j)->name();
    }
  }
  if (!single_test || ::testing::GTEST_FLAG(repeat) != 1 || std::getenv("ROS_DOMAIN_ID") == nullptr)
  {
    std::fprintf(stderr, "Run one exact GTest through run_move_action_test.py (see CTest registration).\n");
    return 2;
  }
  const int result = RUN_ALL_TESTS();
  std::fflush(nullptr);
  // GTest has written its XML. Exit without destroying resources used by detached workers.
  std::_Exit(result);
}
