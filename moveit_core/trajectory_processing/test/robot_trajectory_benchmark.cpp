/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, PickNik Robotics.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Mario Prats */

// To run this benchmark, 'cd' to the build/moveit_core/trajectory_processing directory and directly run the binary.

#include <benchmark/benchmark.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

// Robot and planning group to use in the benchmarks.
constexpr char TEST_ROBOT[] = "panda";
constexpr char TEST_GROUP[] = "panda_arm";

// Benchmark manual creation of a trajectory with a given number of waypoints.
// This includes creating and updating the individual RobotState's.
static void robotTrajectoryCreate(benchmark::State& st)
{
  int n_states = st.range(0);
  const moveit::core::RobotModelPtr& robot_model = moveit::core::loadTestingRobotModel(TEST_ROBOT);

  // Make sure the group exists, otherwise exit early with an error.
  if (!robot_model->hasJointModelGroup(TEST_GROUP))
  {
    st.SkipWithError("The planning group doesn't exist.");
    return;
  }
  auto* group = robot_model->getJointModelGroup(TEST_GROUP);

  // Robot state.
  moveit::core::RobotState robot_state(robot_model);
  robot_state.setToDefaultValues();

  for (auto _ : st)
  {
    auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, group);
    for (int i = 0; i < n_states; ++i)
    {
      // Create a sinusoidal test trajectory for all the joints.
      const double joint_value = std::sin(0.001 * i);
      const double duration_from_previous = 0.1;

      moveit::core::RobotState robot_state_waypoint(robot_state);
      Eigen::VectorXd joint_values = Eigen::VectorXd::Constant(group->getActiveVariableCount(), joint_value);
      robot_state_waypoint.setJointGroupActivePositions(group, joint_values);
      trajectory->addSuffixWayPoint(robot_state_waypoint, duration_from_previous);
    }
  }
}

// Benchmark timing of a trajectory with a given number of waypoints, via TOTG.
static void robotTrajectoryTiming(benchmark::State& st)
{
  int n_states = st.range(0);
  const moveit::core::RobotModelPtr& robot_model = moveit::core::loadTestingRobotModel(TEST_ROBOT);

  // Make sure the group exists, otherwise exit early with an error.
  if (!robot_model->hasJointModelGroup(TEST_GROUP))
  {
    st.SkipWithError("The planning group doesn't exist.");
    return;
  }
  auto* group = robot_model->getJointModelGroup(TEST_GROUP);

  // Robot state.
  moveit::core::RobotState robot_state(robot_model);
  robot_state.setToDefaultValues();

  // Trajectory.
  auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, group);
  Eigen::VectorXd joint_values = Eigen::VectorXd::Zero(group->getActiveVariableCount());
  for (int i = 0; i < n_states; ++i)
  {
    // Create a sinusoidal test trajectory for all the joints.
    const double joint_value = std::sin(0.001 * i);
    const double duration_from_previous = 0.0;

    moveit::core::RobotState robot_state_waypoint(robot_state);
    joint_values = Eigen::VectorXd::Constant(group->getActiveVariableCount(), joint_value);
    robot_state_waypoint.setJointGroupActivePositions(group, joint_values);
    trajectory->addSuffixWayPoint(robot_state_waypoint, duration_from_previous);
  }

  // Add some velocity / acceleration limits, which are needed for TOTG.
  std::unordered_map<std::string, double> velocity_limits, acceleration_limits;
  for (const auto& joint_name : group->getActiveJointModelNames())
  {
    velocity_limits[joint_name] = 1.0;
    acceleration_limits[joint_name] = 2.0;
  }

  for (auto _ : st)
  {
    trajectory_processing::TimeOptimalTrajectoryGeneration totg(/*path_tolerance=*/0.0);
    totg.computeTimeStamps(*trajectory, velocity_limits, acceleration_limits);
  }
}

BENCHMARK(robotTrajectoryCreate)->RangeMultiplier(10)->Range(10, 100000)->Unit(benchmark::kMillisecond);
BENCHMARK(robotTrajectoryTiming)->RangeMultiplier(10)->Range(10, 20000)->Unit(benchmark::kMillisecond);
