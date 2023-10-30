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

/* Author: Mario Prats, Robert Haschke */

// To run this benchmark, 'cd' to the build/moveit_core/robot_state directory and directly run the binary.

#include <benchmark/benchmark.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <random_numbers/random_numbers.h>

// Robot and planning group for benchmarks, except for RobotStateUpdate and RobotStateForwardKinematics, which will use
// the pr2.
constexpr char TEST_ROBOT[] = "panda";
constexpr char TEST_GROUP[] = "panda_arm";

static void MultiplyAffineTimesMatrix(benchmark::State& st)
{
  int n_iters = st.range(0);
  Eigen::Isometry3d isometry = Eigen::Translation3d(1, 2, 3) *
                               Eigen::AngleAxisd(0.13 * M_PI, Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(0.29 * M_PI, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(0.42 * M_PI, Eigen::Vector3d::UnitZ());
  for (auto _ : st)
  {
    for (int i = 0; i < n_iters; ++i)
    {
      Eigen::Affine3d result;
      benchmark::DoNotOptimize(result = isometry.affine() * isometry.matrix());
      benchmark::ClobberMemory();
    }
  }
}

static void MultiplyMatrixTimesMatrix(benchmark::State& st)
{
  int n_iters = st.range(0);
  Eigen::Isometry3d isometry = Eigen::Translation3d(1, 2, 3) *
                               Eigen::AngleAxisd(0.13 * M_PI, Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(0.29 * M_PI, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(0.42 * M_PI, Eigen::Vector3d::UnitZ());
  for (auto _ : st)
  {
    for (int i = 0; i < n_iters; ++i)
    {
      Eigen::Matrix4d result;
      benchmark::DoNotOptimize(result = isometry.matrix() * isometry.matrix());
      benchmark::ClobberMemory();
    }
  }
}

static void MultiplyIsometryTimesIsometry(benchmark::State& st)
{
  int n_iters = st.range(0);
  Eigen::Isometry3d isometry = Eigen::Translation3d(1, 2, 3) *
                               Eigen::AngleAxisd(0.13 * M_PI, Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(0.29 * M_PI, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(0.42 * M_PI, Eigen::Vector3d::UnitZ());
  for (auto _ : st)
  {
    for (int i = 0; i < n_iters; ++i)
    {
      Eigen::Isometry3d result;
      benchmark::DoNotOptimize(result = isometry * isometry);
      benchmark::ClobberMemory();
    }
  }
}

static void InverseIsometry3d(benchmark::State& st)
{
  int n_iters = st.range(0);
  Eigen::Isometry3d isometry = Eigen::Translation3d(1, 2, 3) *
                               Eigen::AngleAxisd(0.13 * M_PI, Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(0.29 * M_PI, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(0.42 * M_PI, Eigen::Vector3d::UnitZ());
  for (auto _ : st)
  {
    for (int i = 0; i < n_iters; ++i)
    {
      Eigen::Isometry3d result;
      benchmark::DoNotOptimize(result = isometry.inverse());
      benchmark::ClobberMemory();
    }
  }
}

static void InverseAffineIsometry(benchmark::State& st)
{
  int n_iters = st.range(0);
  Eigen::Isometry3d isometry = Eigen::Translation3d(1, 2, 3) *
                               Eigen::AngleAxisd(0.13 * M_PI, Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(0.29 * M_PI, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(0.42 * M_PI, Eigen::Vector3d::UnitZ());
  Eigen::Affine3d affine;
  affine.matrix() = isometry.matrix();

  for (auto _ : st)
  {
    for (int i = 0; i < n_iters; ++i)
    {
      Eigen::Affine3d result;
      benchmark::DoNotOptimize(result = affine.inverse(Eigen::Isometry).affine());
      benchmark::ClobberMemory();
    }
  }
}

static void InverseAffine(benchmark::State& st)
{
  int n_iters = st.range(0);
  Eigen::Isometry3d isometry = Eigen::Translation3d(1, 2, 3) *
                               Eigen::AngleAxisd(0.13 * M_PI, Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(0.29 * M_PI, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(0.42 * M_PI, Eigen::Vector3d::UnitZ());
  Eigen::Affine3d affine;
  affine.matrix() = isometry.matrix();

  for (auto _ : st)
  {
    for (int i = 0; i < n_iters; ++i)
    {
      Eigen::Affine3d result;
      benchmark::DoNotOptimize(result = affine.inverse().affine());
      benchmark::ClobberMemory();
    }
  }
}

static void InverseMatrix4d(benchmark::State& st)
{
  int n_iters = st.range(0);
  Eigen::Isometry3d isometry = Eigen::Translation3d(1, 2, 3) *
                               Eigen::AngleAxisd(0.13 * M_PI, Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(0.29 * M_PI, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(0.42 * M_PI, Eigen::Vector3d::UnitZ());
  Eigen::Affine3d affine;
  affine.matrix() = isometry.matrix();

  for (auto _ : st)
  {
    for (int i = 0; i < n_iters; ++i)
    {
      Eigen::Affine3d result;
      benchmark::DoNotOptimize(result = affine.matrix().inverse());
      benchmark::ClobberMemory();
    }
  }
}

static void RobotStateConstruct(benchmark::State& st)
{
  int n_states = st.range(0);
  const moveit::core::RobotModelPtr& robot_model = moveit::core::loadTestingRobotModel(TEST_ROBOT);

  // Make sure the group exists, otherwise exit early with an error.
  if (!robot_model->hasJointModelGroup(TEST_GROUP))
  {
    st.SkipWithError("The planning group doesn't exist.");
    return;
  }

  for (auto _ : st)
  {
    for (int i = 0; i < n_states; i++)
    {
      std::unique_ptr<moveit::core::RobotState> robot_state;
      benchmark::DoNotOptimize(robot_state = std::make_unique<moveit::core::RobotState>(robot_model));
      benchmark::ClobberMemory();
    }
  }
}

static void RobotStateCopy(benchmark::State& st)
{
  int n_states = st.range(0);
  const moveit::core::RobotModelPtr& robot_model = moveit::core::loadTestingRobotModel(TEST_ROBOT);

  // Make sure the group exists, otherwise exit early with an error.
  if (!robot_model->hasJointModelGroup(TEST_GROUP))
  {
    st.SkipWithError("The planning group doesn't exist.");
    return;
  }

  // Robot state.
  moveit::core::RobotState robot_state(robot_model);
  robot_state.setToDefaultValues();

  for (auto _ : st)
  {
    for (int i = 0; i < n_states; i++)
    {
      std::unique_ptr<moveit::core::RobotState> robot_state_copy;
      benchmark::DoNotOptimize(robot_state_copy = std::make_unique<moveit::core::RobotState>(robot_state));
      benchmark::ClobberMemory();
    }
  }
}

static void RobotStateUpdate(benchmark::State& st)
{
  int n_states = st.range(0);
  const moveit::core::RobotModelPtr& robot_model = moveit::core::loadTestingRobotModel("pr2");
  moveit::core::RobotState state(robot_model);

  for (auto _ : st)
  {
    for (int i = 0; i < n_states; ++i)
    {
      state.setToRandomPositions();
      state.update();
      benchmark::ClobberMemory();
    }
  }
}

static void RobotStateForwardKinematics(benchmark::State& st)
{
  int n_states = st.range(0);
  const moveit::core::RobotModelPtr& robot_model = moveit::core::loadTestingRobotModel("pr2");
  moveit::core::RobotState state(robot_model);

  for (auto _ : st)
  {
    for (int i = 0; i < n_states; ++i)
    {
      state.setToRandomPositions();
      Eigen::Isometry3d transform;
      benchmark::DoNotOptimize(transform = state.getGlobalLinkTransform(robot_model->getLinkModel("r_wrist_roll_"
                                                                                                  "link")));
      benchmark::ClobberMemory();
    }
  }
}

static void MoveItJacobian(benchmark::State& st)
{
  // Load a test robot model.
  const moveit::core::RobotModelPtr& robot_model = moveit::core::loadTestingRobotModel(TEST_ROBOT);

  // Make sure the group exists, otherwise exit early with an error.
  if (!robot_model->hasJointModelGroup(TEST_GROUP))
  {
    st.SkipWithError("The planning group doesn't exist.");
    return;
  }

  // Robot state.
  moveit::core::RobotState kinematic_state(robot_model);
  const moveit::core::JointModelGroup* jmg = kinematic_state.getJointModelGroup(TEST_GROUP);

  // Provide our own random number generator to setToRandomPositions to get a deterministic sequence of joint
  // configurations.
  random_numbers::RandomNumberGenerator rng(0);

  for (auto _ : st)
  {
    // Time only the jacobian computation, not the forward kinematics.
    st.PauseTiming();
    kinematic_state.setToRandomPositions(jmg, rng);
    kinematic_state.updateLinkTransforms();
    st.ResumeTiming();
    kinematic_state.getJacobian(jmg);
  }
}

static void KdlJacobian(benchmark::State& st)
{
  const moveit::core::RobotModelPtr& robot_model = moveit::core::loadTestingRobotModel(TEST_ROBOT);

  // Make sure the group exists, otherwise exit early with an error.
  if (!robot_model->hasJointModelGroup(TEST_GROUP))
  {
    st.SkipWithError("The planning group doesn't exist.");
    return;
  }

  // Robot state.
  moveit::core::RobotState kinematic_state(robot_model);
  const moveit::core::JointModelGroup* jmg = kinematic_state.getJointModelGroup(TEST_GROUP);

  // Provide our own random number generator to setToRandomPositions to get a deterministic sequence of joint
  // configurations.
  random_numbers::RandomNumberGenerator rng(0);

  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(*robot_model->getURDF(), kdl_tree))
  {
    st.SkipWithError("Can't create KDL tree.");
    return;
  }

  KDL::Chain kdl_chain;
  if (!kdl_tree.getChain(jmg->getJointModels().front()->getParentLinkModel()->getName(),
                         jmg->getLinkModelNames().back(), kdl_chain))
  {
    st.SkipWithError("Can't create KDL Chain.");
    return;
  }

  KDL::ChainJntToJacSolver jacobian_solver(kdl_chain);

  for (auto _ : st)
  {
    st.PauseTiming();
    kinematic_state.setToRandomPositions(jmg, rng);
    kinematic_state.updateLinkTransforms();
    KDL::Jacobian jacobian(kdl_chain.getNrOfJoints());
    KDL::JntArray kdl_q;
    kdl_q.resize(kdl_chain.getNrOfJoints());
    kinematic_state.copyJointGroupPositions(jmg, &kdl_q.data[0]);
    st.ResumeTiming();
    jacobian_solver.JntToJac(kdl_q, jacobian);
  }
}

BENCHMARK(MultiplyAffineTimesMatrix)->Arg(1e7)->Unit(benchmark::kMillisecond);
BENCHMARK(MultiplyMatrixTimesMatrix)->Arg(1e7)->Unit(benchmark::kMillisecond);
BENCHMARK(MultiplyIsometryTimesIsometry)->Arg(1e7)->Unit(benchmark::kMillisecond);

BENCHMARK(InverseIsometry3d)->Arg(1e7)->Unit(benchmark::kMillisecond);
BENCHMARK(InverseAffineIsometry)->Arg(1e7)->Unit(benchmark::kMillisecond);
BENCHMARK(InverseAffine)->Arg(1e7)->Unit(benchmark::kMillisecond);
BENCHMARK(InverseMatrix4d)->Arg(1e7)->Unit(benchmark::kMillisecond);

BENCHMARK(RobotStateConstruct)->RangeMultiplier(10)->Range(100, 10000)->Unit(benchmark::kMillisecond);
BENCHMARK(RobotStateCopy)->RangeMultiplier(10)->Range(100, 10000)->Unit(benchmark::kMillisecond);
BENCHMARK(RobotStateUpdate)->RangeMultiplier(10)->Range(10, 1000)->Unit(benchmark::kMillisecond);
BENCHMARK(RobotStateForwardKinematics)->RangeMultiplier(10)->Range(10, 1000)->Unit(benchmark::kMillisecond);

BENCHMARK(MoveItJacobian);
BENCHMARK(KdlJacobian);
