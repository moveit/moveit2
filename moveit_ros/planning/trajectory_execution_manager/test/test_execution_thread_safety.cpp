/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Siddartha Reddy Chinthala
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

/* Desc: Regression test for https://github.com/moveit/moveit2/issues/3965
   ("crash on aborted goal"): concurrent execute()/stopExecution() calls on
   one TrajectoryExecutionManager must not terminate the process. */

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <moveit/planning_scene_monitor/current_state_monitor.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <srdfdom/model.h>
#include <tf2_ros/buffer.hpp>
#include <urdf_parser/urdf_parser.h>

namespace
{
// Minimal 3-joint robot matching the fake "left_arm" controller of the test
// controller manager plugin (test_moveit_controller_manager.hpp), which
// controls joints lj1, lj2 and lj3.
const char* kTestUrdf = R"(
  <robot name="test_robot">
    <link name="base_link"/>
    <link name="link1"/>
    <link name="link2"/>
    <link name="link3"/>
    <joint name="lj1" type="revolute">
      <parent link="base_link"/>
      <child link="link1"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
    </joint>
    <joint name="lj2" type="revolute">
      <parent link="link1"/>
      <child link="link2"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
    </joint>
    <joint name="lj3" type="revolute">
      <parent link="link2"/>
      <child link="link3"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
    </joint>
  </robot>)";

const char* kTestSrdf = R"(<robot name="test_robot"></robot>)";

moveit::core::RobotModelPtr loadTestRobotModel()
{
  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(kTestUrdf);
  EXPECT_NE(urdf_model, nullptr);
  auto srdf_model = std::make_shared<srdf::Model>();
  EXPECT_TRUE(srdf_model->initString(*urdf_model, kTestSrdf));
  return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
}

moveit_msgs::msg::RobotTrajectory makeTestTrajectory()
{
  moveit_msgs::msg::RobotTrajectory traj;
  traj.joint_trajectory.joint_names = { "lj1", "lj2", "lj3" };
  traj.joint_trajectory.points.resize(2);
  traj.joint_trajectory.points[0].positions = { 0.0, 0.0, 0.0 };
  traj.joint_trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(0.0);
  traj.joint_trajectory.points[1].positions = { 0.1, 0.1, 0.1 };
  traj.joint_trajectory.points[1].time_from_start = rclcpp::Duration::from_seconds(0.5);
  return traj;
}
}  // namespace

class ExecutionThreadSafetyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        { { "moveit_controller_manager", "test_moveit_controller_manager/TestControllerManager" } });
    // Declare the override above: Rolling defaults
    // automatically_declare_parameters_from_overrides to false, so without this
    // get_parameter("moveit_controller_manager") fails and initialize() falls
    // back to single-plugin selection.
    options.automatically_declare_parameters_from_overrides(true);
    node_ = std::make_shared<rclcpp::Node>("test_execution_thread_safety", options);

    robot_model_ = loadTestRobotModel();
    ASSERT_NE(robot_model_, nullptr);
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    csm_ = std::make_shared<planning_scene_monitor::CurrentStateMonitor>(node_, robot_model_, tf_buffer, false);

    tem_ = std::make_shared<trajectory_execution_manager::TrajectoryExecutionManager>(node_, robot_model_, csm_);
    // No live robot state is available in this test: skip start-point
    // validation and waiting for the robot to stop, and disable execution
    // duration monitoring (the fake controller handle returns immediately).
    tem_->setAllowedStartTolerance(0.0);
    tem_->enableExecutionDurationMonitoring(false);
    tem_->setWaitForTrajectoryCompletion(false);
  }

  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelConstPtr robot_model_;
  planning_scene_monitor::CurrentStateMonitorPtr csm_;
  trajectory_execution_manager::TrajectoryExecutionManagerPtr tem_;
};

TEST_F(ExecutionThreadSafetyTest, ConcurrentExecuteAndStopDoesNotTerminate)
{
  // Regression test for https://github.com/moveit/moveit2/issues/3965
  // ("crash on aborted goal"): execute() installed the execution thread
  // without holding execution_thread_mutex_, while stopExecution() joined
  // and reset it under the mutex. When a second execution started as the
  // previous one completed (e.g. a goal aborted/preempted by a new one),
  // the new thread could be destroyed while still joinable, and destroying
  // a joinable std::thread calls std::terminate(), killing move_group.
  // stopExecution() also set execution_complete_ without notifying
  // execution_complete_condition_, so a stop arriving while another thread
  // waited in waitForExecution() left that thread sleeping forever.
  // Hammering execute()/stopExecution() from two threads reproduces both:
  // before the fix the test hangs (lost wakeup) and ThreadSanitizer reports
  // the data race on execution_thread_; after the fix it completes.
  auto traj = makeTestTrajectory();
  ASSERT_TRUE(tem_->push(traj, "left_arm"));

  constexpr int kIterations = 5000;
  // The stopper must race for the whole run, not just a fixed number of
  // calls: it spins far faster than the executor, so a fixed-count stopper
  // finishes (and exits) long before the executor's iterations are done,
  // leaving almost no overlap in which the race can fire.
  std::atomic<bool> executor_done{ false };
  std::thread executor([this, &executor_done] {
    for (int i = 0; i < kIterations; ++i)
    {
      // auto_clear=false: keep the pushed trajectory so every iteration
      // starts a fresh execution thread.
      tem_->execute(trajectory_execution_manager::TrajectoryExecutionManager::ExecutionCompleteCallback(), false);
      tem_->waitForExecution();
    }
    executor_done = true;
  });
  std::thread stopper([this, &executor_done] {
    while (!executor_done)
      tem_->stopExecution(false);
  });
  executor.join();
  stopper.join();

  // Survival is the assertion. A final clean execution must still work.
  ASSERT_TRUE(tem_->push(traj, "left_arm"));
  auto status = tem_->executeAndWait();
  EXPECT_EQ(status, moveit_controller_manager::ExecutionStatus::SUCCEEDED);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
