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

#include <vector>

#include <gtest/gtest.h>

#include <controller_manager_msgs/srv/detail/list_controllers__struct.hpp>
#include <controller_manager_msgs/srv/detail/switch_controller__struct.hpp>
#include <eigen3/Eigen/Eigen>
#include <moveit/controller_manager/controller_manager.h>
#include <pluginlib/class_loader.hpp>

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
