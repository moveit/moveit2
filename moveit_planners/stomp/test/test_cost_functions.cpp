/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, PickNik Inc.
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
 *   * Neither the name of PickNik Inc. nor the names of its
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

/** @file
 * @author Henning Kayser
 */

#include <gtest/gtest.h>
#include <stomp_moveit/cost_functions.hpp>

constexpr size_t TIMESTEPS = 100;
constexpr size_t VARIABLES = 6;
constexpr double PENALTY = 1.0;

TEST(NoiseGeneratorTest, testGetCostFunctionAllValidStates)
{
  // GIVEN a cost function with a state validator that only returns valid costs of 0.0
  auto state_validator_fn = [](const Eigen::VectorXd& /* state_positions */) { return 0.0; };
  auto cost_fn =
      stomp_moveit::costs::getCostFunctionFromStateValidator(state_validator_fn, 0.1 /* interpolation_step_size */);

  // GIVEN a trajectory with TIMESTEPS states, with waypoints interpolating from 0.0 to 1.0 joint values
  Eigen::MatrixXd values = Eigen::MatrixXd::Zero(VARIABLES, TIMESTEPS);
  const int last_timestep = values.cols() - 1;
  for (int timestep = 0; timestep <= last_timestep; ++timestep)
  {
    values.col(timestep).fill(static_cast<double>(timestep) / last_timestep);
  }

  // WHEN the cost function is applied to the trajectory
  Eigen::VectorXd costs(TIMESTEPS);
  bool validity;
  ASSERT_TRUE(cost_fn(values, costs, validity));

  // THEN the trajectory must be valid and have zero costs
  EXPECT_TRUE(validity);
  EXPECT_EQ(costs.sum(), 0.0);
}

TEST(NoiseGeneratorTest, testGetCostFunctionInvalidStates)
{
  // GIVEN a cost function with a simulated state validator that tags selected timesteps as invalid.
  // The state validation function is called once per timestep since interpolation is disabled.
  // This assumption is confirmed as boundary assumption after calling the solver.
  static const std::set<int> INVALID_TIMESTEPS(
      { 0, 10, 11, 12, 25, 26, 27, 46, 63, 64, 65, 66, 67, 68, 69, 97, 98, 99 });
  size_t timestep_counter = 0;
  auto state_validator_fn = [&](const Eigen::VectorXd& /* state_positions */) {
    return PENALTY * INVALID_TIMESTEPS.count(timestep_counter++);
  };
  auto cost_fn =
      stomp_moveit::costs::getCostFunctionFromStateValidator(state_validator_fn, 0.0 /* interpolation disabled */);

  // GIVEN a trajectory with TIMESTEPS states, with waypoints interpolating from 0.0 to 1.0 joint values
  Eigen::MatrixXd values = Eigen::MatrixXd::Zero(VARIABLES, TIMESTEPS);
  const int last_timestep = values.cols() - 1;
  for (int timestep = 0; timestep <= last_timestep; ++timestep)
  {
    values.col(timestep).fill(static_cast<double>(timestep) / last_timestep);
  }

  // WHEN the cost function is applied to the trajectory
  Eigen::VectorXd costs(TIMESTEPS);
  bool validity;
  ASSERT_TRUE(cost_fn(values, costs, validity));

  // THEN the following boundary assumptions about cost function outputs, costs and validity need to be met
  EXPECT_FALSE(validity);                // invalid states must result in an invalid trajectory
  EXPECT_EQ(timestep_counter, 100u);     // 100 timesteps checked without interpolation
  EXPECT_LE(costs.maxCoeff(), PENALTY);  // the highest cost must not be higher than the configured penalty
  EXPECT_GE(costs.minCoeff(), 0.0);      // no negative cost values should be computed

  // THEN the total cost must equal the sum of penalties produced by all invalid timesteps
  EXPECT_DOUBLE_EQ(costs.sum(), PENALTY * INVALID_TIMESTEPS.size());

  // THEN invalid timesteps must account for the majority of the total cost.
  // We expect that invalid windows cover at least 2*sigma (=68.1%) of each cost distribution.
  const std::vector<int> invalid_timesteps_vec(INVALID_TIMESTEPS.begin(), INVALID_TIMESTEPS.end());
  EXPECT_GE(costs(invalid_timesteps_vec).sum(), 0.681 * PENALTY);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
