/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Nanjing University
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
 *   * Neither the name of Nanjing University nor the names of its
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

#include <moveit/controller_manager/controller_manager.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace trajectory_execution_manager_test
{
class ControllerHandle : public moveit_controller_manager::MoveItControllerHandle
{
public:
  ControllerHandle() : MoveItControllerHandle("test_controller")
  {
  }

  bool sendTrajectory(const moveit_msgs::msg::RobotTrajectory&) override
  {
    return true;
  }

  bool cancelExecution() override
  {
    return true;
  }

  bool waitForExecution(const rclcpp::Duration&) override
  {
    return true;
  }

  moveit_controller_manager::ExecutionStatus getLastExecutionStatus() override
  {
    return moveit_controller_manager::ExecutionStatus::SUCCEEDED;
  }
};

class ControllerManager : public moveit_controller_manager::MoveItControllerManager
{
public:
  void initialize(const rclcpp::Node::SharedPtr&) override
  {
  }

  moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string&) override
  {
    return std::make_shared<ControllerHandle>();
  }

  void getControllersList(std::vector<std::string>& names) override
  {
    names = { "test_controller" };
  }

  void getActiveControllers(std::vector<std::string>& names) override
  {
    names = { "test_controller" };
  }

  void getControllerJoints(const std::string&, std::vector<std::string>& joints) override
  {
    joints = { "world-base-joint", "base-tool-joint" };
  }

  ControllerState getControllerState(const std::string&) override
  {
    ControllerState state;
    state.active_ = true;
    state.default_ = true;
    return state;
  }

  bool switchControllers(const std::vector<std::string>&, const std::vector<std::string>&) override
  {
    return true;
  }
};
}  // namespace trajectory_execution_manager_test

PLUGINLIB_EXPORT_CLASS(trajectory_execution_manager_test::ControllerManager,
                       moveit_controller_manager::MoveItControllerManager)
