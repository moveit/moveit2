/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_set>
#include <vector>

#include <gtest/gtest.h>

#include <control_msgs/action/parallel_gripper_command.hpp>
#include <controller_manager_msgs/srv/detail/list_controllers__struct.hpp>
#include <controller_manager_msgs/srv/detail/switch_controller__struct.hpp>
#include <eigen3/Eigen/Eigen>
#include <moveit/controller_manager/controller_manager.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class MockControllersManagerService final : public rclcpp::Node
{
public:
  MockControllersManagerService() : Node("list_controllers_service")
  {
    list_controller_service_ = create_service<controller_manager_msgs::srv::ListControllers>(
        "controller_manager/list_controllers",
        [this](const std::shared_ptr<controller_manager_msgs::srv::ListControllers::Request>& request,
               const std::shared_ptr<controller_manager_msgs::srv::ListControllers::Response>& response) {
          handleListControllersService(request, response);
        });
    switch_controller_service_ = create_service<controller_manager_msgs::srv::SwitchController>(
        "controller_manager/switch_controller",
        [this](const std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request>& request,
               const std::shared_ptr<controller_manager_msgs::srv::SwitchController::Response>& response) {
          handleSwitchControllerService(request, response);
        });
  }

  controller_manager_msgs::srv::SwitchController::Request last_request;

private:
  void handleListControllersService(
      const std::shared_ptr<controller_manager_msgs::srv::ListControllers::Request>& /*request*/,
      const std::shared_ptr<controller_manager_msgs::srv::ListControllers::Response>& response)
  {
    controller_manager_msgs::msg::ChainConnection chain_connection_a;
    chain_connection_a.name = "B";
    chain_connection_a.reference_interfaces = { "ref_1", "ref_2", "ref_3" };
    controller_manager_msgs::msg::ControllerState controller_a;
    controller_a.chain_connections.push_back(chain_connection_a);
    controller_a.name = "A";
    controller_a.is_chained = true;
    controller_a.required_command_interfaces = { "jnt_1", "jnt_2", "jnt_3" };
    controller_a.type = "joint_trajectory_controller/JointTrajectoryController";
    controller_a.state = activate_set_.find(controller_a.name) != activate_set_.end() ? "active" : "inactive";

    controller_manager_msgs::msg::ControllerState controller_b;
    controller_b.name = "B";
    controller_b.required_command_interfaces = { "jnt_4", "jnt_5" };
    controller_b.reference_interfaces = { "ref_1", "ref_2", "ref_3" };
    controller_b.type = "joint_trajectory_controller/JointTrajectoryController";
    controller_b.state = activate_set_.find(controller_b.name) != activate_set_.end() ? "active" : "inactive";

    response->controller = { controller_a, controller_b };
  }

  void handleSwitchControllerService(
      const std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request>& request,
      const std::shared_ptr<controller_manager_msgs::srv::SwitchController::Response>& response)
  {
    last_request = *request;
    for (const auto& deactivate : request->deactivate_controllers)
    {
      activate_set_.erase(deactivate);
    }
    for (const auto& activate : request->activate_controllers)
    {
      activate_set_.insert(activate);
    }
    response->ok = true;
  }

  std::unordered_set<std::string> activate_set_;
  rclcpp::Service<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controller_service_;
  rclcpp::Service<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_service_;
};

TEST(ControllerManagerPlugin, SwitchControllers)
{
  // GIVEN a ClassLoader for MoveItControllerManager
  pluginlib::ClassLoader<moveit_controller_manager::MoveItControllerManager> loader(
      "moveit_core", "moveit_controller_manager::MoveItControllerManager");

  // WHEN we load the custom plugin defined in this package
  // THEN loading succeeds
  auto instance = loader.createUniqueInstance("moveit_ros_control_interface/Ros2ControlManager");

  const auto mock_service = std::make_shared<MockControllersManagerService>();
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread ros_thread([&executor, &mock_service]() {
    executor.add_node(mock_service);
    executor.spin();
  });
  instance->initialize(mock_service);

  // A and B should start
  instance->switchControllers({ "/A" }, {});
  EXPECT_EQ(mock_service->last_request.activate_controllers, std::vector<std::string>({ "A", "B" }));
  EXPECT_EQ(mock_service->last_request.deactivate_controllers, std::vector<std::string>({}));
  // A and B should stop
  instance->switchControllers({}, { "/B" });
  EXPECT_EQ(mock_service->last_request.activate_controllers, std::vector<std::string>({}));
  EXPECT_EQ(mock_service->last_request.deactivate_controllers, std::vector<std::string>({ "A", "B" }));
  // Only B should start
  instance->switchControllers({ "/B" }, {});
  EXPECT_EQ(mock_service->last_request.activate_controllers, std::vector<std::string>({ "B" }));
  EXPECT_EQ(mock_service->last_request.deactivate_controllers, std::vector<std::string>({}));
  // Only B should stop
  instance->switchControllers({}, { "/B" });
  EXPECT_EQ(mock_service->last_request.activate_controllers, std::vector<std::string>({}));
  EXPECT_EQ(mock_service->last_request.deactivate_controllers, std::vector<std::string>({ "B" }));
  // Multiple activations results in only 1
  instance->switchControllers({ "/B", "/B", "/B", "/B" }, {});
  EXPECT_EQ(mock_service->last_request.activate_controllers, std::vector<std::string>({ "B" }));
  EXPECT_EQ(mock_service->last_request.deactivate_controllers, std::vector<std::string>({}));
  instance->switchControllers({}, { "/B" });
  // Multiple activation and deactivate of same controller results in empty list
  instance->switchControllers({ "/B", "/A" }, { "/A", "/A", "/A" });
  EXPECT_EQ(mock_service->last_request.activate_controllers, std::vector<std::string>({ "B" }));
  EXPECT_EQ(mock_service->last_request.deactivate_controllers, std::vector<std::string>({}));

  executor.cancel();
  ros_thread.join();
}

// Mock controller manager exposing a single parallel gripper controller, plus the controller's action server.
class MockParallelGripperControllerManager final : public rclcpp::Node
{
public:
  explicit MockParallelGripperControllerManager(bool active) : Node("parallel_gripper_controller_manager")
  {
    list_controller_service_ = create_service<controller_manager_msgs::srv::ListControllers>(
        "controller_manager/list_controllers",
        [active](const std::shared_ptr<controller_manager_msgs::srv::ListControllers::Request>& /*request*/,
                 const std::shared_ptr<controller_manager_msgs::srv::ListControllers::Response>& response) {
          controller_manager_msgs::msg::ControllerState controller;
          controller.name = "gripper_controller";
          controller.type = "parallel_gripper_action_controller/GripperActionController";
          controller.state = active ? "active" : "inactive";
          // ros2_control only reports claimed interfaces for active controllers
          controller.required_command_interfaces = { "gripper_joint/position", "gripper_joint/max_effort" };
          if (active)
          {
            controller.claimed_interfaces = controller.required_command_interfaces;
          }
          response->controller = { controller };
        });

    using GripperCommand = control_msgs::action::ParallelGripperCommand;
    action_server_ = rclcpp_action::create_server<GripperCommand>(
        this, "gripper_controller/gripper_cmd",
        [this](const rclcpp_action::GoalUUID& /*uuid*/, const std::shared_ptr<const GripperCommand::Goal>& goal) {
          std::scoped_lock<std::mutex> lock(goal_mutex_);
          received_goal_ = *goal;
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>>& /*goal_handle*/) {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>>& goal_handle) {
          goal_handle->succeed(std::make_shared<GripperCommand::Result>());
        });
  }

  std::optional<control_msgs::action::ParallelGripperCommand::Goal> receivedGoal()
  {
    std::scoped_lock<std::mutex> lock(goal_mutex_);
    return received_goal_;
  }

private:
  rclcpp::Service<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controller_service_;
  rclcpp_action::Server<control_msgs::action::ParallelGripperCommand>::SharedPtr action_server_;
  std::mutex goal_mutex_;
  std::optional<control_msgs::action::ParallelGripperCommand::Goal> received_goal_;
};

// Spins a node in a background thread for the lifetime of this object, so the thread is joined even if an assertion
// fails.
class ScopedSpinner
{
public:
  explicit ScopedSpinner(const rclcpp::Node::SharedPtr& node)
  {
    executor_.add_node(node);
    thread_ = std::thread([this]() { executor_.spin(); });
  }
  ~ScopedSpinner()
  {
    executor_.cancel();
    thread_.join();
  }

private:
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread thread_;
};

class ParallelGripperControllerHandle : public testing::TestWithParam<bool /* controller is active */>
{
};

TEST_P(ParallelGripperControllerHandle, CommandsJointClaimedByController)
{
  pluginlib::ClassLoader<moveit_controller_manager::MoveItControllerManager> loader(
      "moveit_core", "moveit_controller_manager::MoveItControllerManager");
  auto instance = loader.createUniqueInstance("moveit_ros_control_interface/Ros2ControlManager");

  const auto mock_service = std::make_shared<MockParallelGripperControllerManager>(GetParam());
  const ScopedSpinner spinner(mock_service);
  instance->initialize(mock_service);

  // GIVEN the handle allocated for the parallel gripper controller
  std::vector<std::string> controller_names;
  instance->getControllersList(controller_names);
  ASSERT_EQ(controller_names, std::vector<std::string>({ "/gripper_controller" }));
  const auto handle = instance->getControllerHandle("/gripper_controller");
  ASSERT_TRUE(handle);

  // WHEN a gripper trajectory is sent
  moveit_msgs::msg::RobotTrajectory trajectory;
  trajectory.joint_trajectory.joint_names = { "gripper_joint" };
  trajectory.joint_trajectory.points.resize(1);
  trajectory.joint_trajectory.points[0].positions = { 0.02 };

  // THEN the goal is sent to the controller's action server for the joint claimed by the controller.
  // Retry until the action client has discovered the server.
  bool sent = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!sent && std::chrono::steady_clock::now() < deadline)
  {
    sent = handle->sendTrajectory(trajectory);
    if (!sent)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  EXPECT_TRUE(sent);
  if (sent)
  {
    EXPECT_TRUE(handle->waitForExecution(rclcpp::Duration::from_seconds(5.0)));
  }

  const auto goal = mock_service->receivedGoal();
  ASSERT_TRUE(goal.has_value());
  EXPECT_EQ(goal->command.name, std::vector<std::string>({ "gripper_joint" }));
  EXPECT_EQ(goal->command.position, std::vector<double>({ 0.02 }));
}

INSTANTIATE_TEST_SUITE_P(ControllerManagerPlugin, ParallelGripperControllerHandle, testing::Bool(),
                         [](const testing::TestParamInfo<bool>& info) { return info.param ? "Active" : "Inactive"; });

TEST(ControllerManagerPlugin, LoadMoveItControllerManagerPlugin)
{
  // GIVEN a ClassLoader for MoveItControllerManager
  pluginlib::ClassLoader<moveit_controller_manager::MoveItControllerManager> loader(
      "moveit_core", "moveit_controller_manager::MoveItControllerManager");

  // WHEN we load the custom plugin defined in this package
  // THEN loading succeeds
  EXPECT_NO_THROW(loader.createUniqueInstance("moveit_ros_control_interface/Ros2ControlManager"));
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
