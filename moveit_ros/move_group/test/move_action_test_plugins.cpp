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

#include "move_action_test_plugins.hpp"

#include <chrono>
#include <thread>

#include <class_loader/class_loader.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>

namespace move_group_test
{
MockPlannerControl& getMockPlannerControl()
{
  static MockPlannerControl s_instance;
  return s_instance;
}

MockControllerControl& getMockControllerControl()
{
  static MockControllerControl s_instance;
  return s_instance;
}

class MockPlanningContext : public planning_interface::PlanningContext
{
public:
  MockPlanningContext(const std::string& name, const std::string& group,
                      const planning_scene::PlanningSceneConstPtr& scene,
                      const planning_interface::MotionPlanRequest& req)
    : planning_interface::PlanningContext(name, group), scene_(scene), req_(req)
  {
  }

  void solve(planning_interface::MotionPlanResponse& res) override
  {
    auto& ctrl = getMockPlannerControl();
    ctrl.call_count.fetch_add(1);
    const PlannedBehavior behavior = ctrl.popOrDefault();

    res.planning_time = behavior.planning_time;
    res.error_code.val = behavior.error_code;

    if (behavior.succeed)
    {
      const auto& model = scene_->getRobotModel();
      auto traj = std::make_shared<robot_trajectory::RobotTrajectory>(model, getGroupName());
      moveit::core::RobotState start_state = scene_->getCurrentState();
      moveit::core::RobotState goal_state = start_state;
      if (!req_.goal_constraints.empty())
      {
        for (const auto& jc : req_.goal_constraints.front().joint_constraints)
        {
          if (model->hasJointModel(jc.joint_name))
          {
            goal_state.setJointPositions(jc.joint_name, &jc.position);
          }
        }
      }
      traj->addSuffixWayPoint(start_state, 0.0);
      traj->addSuffixWayPoint(goal_state, 0.05);
      res.trajectory = traj;
      return;
    }

    res.trajectory.reset();
  }

  void solve(planning_interface::MotionPlanDetailedResponse& res) override
  {
    planning_interface::MotionPlanResponse simple;
    solve(simple);
    res.error_code = simple.error_code;
    res.processing_time.push_back(simple.planning_time);
    if (simple.trajectory)
    {
      res.trajectory.push_back(simple.trajectory);
      res.description.push_back("mock");
    }
  }

  bool terminate() override
  {
    return true;
  }

  void clear() override
  {
  }

private:
  planning_scene::PlanningSceneConstPtr scene_;
  planning_interface::MotionPlanRequest req_;
};

class MockPlannerManager : public planning_interface::PlannerManager
{
public:
  ~MockPlannerManager() override = default;

  planning_interface::PlanningContextPtr
  getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                     const planning_interface::MotionPlanRequest& req,
                     moveit_msgs::msg::MoveItErrorCodes& /*error_code*/) const override
  {
    return std::make_shared<MockPlanningContext>("MockPlanningContext", req.group_name, planning_scene, req);
  }

  bool canServiceRequest(const planning_interface::MotionPlanRequest& /*req*/) const override
  {
    return true;
  }

  std::string getDescription() const override
  {
    return "MockPlannerManager";
  }
};

class MockControllerHandle : public moveit_controller_manager::MoveItControllerHandle
{
public:
  explicit MockControllerHandle(const std::string& name) : moveit_controller_manager::MoveItControllerHandle(name)
  {
  }

  bool sendTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory) override
  {
    auto& ctrl = getMockControllerControl();
    ctrl.dispatch_count.fetch_add(1);
    {
      std::lock_guard<std::mutex> lock(ctrl.mutex);
      ctrl.last_trajectory = trajectory;
    }

    const ExecutionBehavior behavior = ctrl.popOrDefault();
    {
      std::lock_guard<std::mutex> lock(handle_mutex_);
      canceled_.store(false);
      delay_ms_.store(behavior.delay_ms);
      last_status_.store(behavior.status);
    }

    if (behavior.on_send)
    {
      behavior.on_send();
    }
    return true;
  }

  bool cancelExecution() override
  {
    getMockControllerControl().cancel_count.fetch_add(1);
    canceled_.store(true);
    last_status_.store(moveit_controller_manager::ExecutionStatus::PREEMPTED);
    return true;
  }

  bool waitForExecution(const rclcpp::Duration& /*timeout*/) override
  {
    const int total_ms = delay_ms_.load();
    for (int elapsed = 0; elapsed < total_ms && !canceled_.load(); elapsed += 10)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return true;
  }

  moveit_controller_manager::ExecutionStatus getLastExecutionStatus() override
  {
    return last_status_.load();
  }

private:
  std::mutex handle_mutex_;
  std::atomic<bool> canceled_{ false };
  std::atomic<int> delay_ms_{ 0 };
  std::atomic<moveit_controller_manager::ExecutionStatus> last_status_{
    moveit_controller_manager::ExecutionStatus::SUCCEEDED
  };
};

class MockControllerManager : public moveit_controller_manager::MoveItControllerManager
{
public:
  void initialize(const rclcpp::Node::SharedPtr& /*node*/) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    handle_ = std::make_shared<MockControllerHandle>("arm_controller");
  }

  moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string& name) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (name == "arm_controller")
      return handle_;
    return nullptr;
  }

  void getControllersList(std::vector<std::string>& names) override
  {
    names = { "arm_controller" };
  }

  void getActiveControllers(std::vector<std::string>& names) override
  {
    names = { "arm_controller" };
  }

  void getControllerJoints(const std::string& name, std::vector<std::string>& joints) override
  {
    if (name == "arm_controller")
      joints = { "joint1" };
    else
      joints.clear();
  }

  ControllerState getControllerState(const std::string& /*name*/) override
  {
    ControllerState state;
    state.active_ = true;
    state.default_ = true;
    return state;
  }

  bool switchControllers(const std::vector<std::string>& /*activate*/,
                         const std::vector<std::string>& /*deactivate*/) override
  {
    return true;
  }

private:
  std::mutex mutex_;
  moveit_controller_manager::MoveItControllerHandlePtr handle_;
};
}  // namespace move_group_test

CLASS_LOADER_REGISTER_CLASS(move_group_test::MockPlannerManager, planning_interface::PlannerManager)
CLASS_LOADER_REGISTER_CLASS(move_group_test::MockControllerManager, moveit_controller_manager::MoveItControllerManager)
