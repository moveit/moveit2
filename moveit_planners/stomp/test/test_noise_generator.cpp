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
#include <stomp_moveit/noise_generators.hpp>

constexpr size_t TIMESTEPS = 100;
constexpr size_t VARIABLES = 6;
static const std::vector<double> STDDEV(VARIABLES, 0.2);
static const Eigen::MatrixXd VALUES = Eigen::MatrixXd::Constant(VARIABLES, TIMESTEPS, 1.0);
static const Eigen::MatrixXd NOISY_VALUES = Eigen::MatrixXd::Constant(VARIABLES, TIMESTEPS, 0.0);
static const Eigen::MatrixXd NOISE = Eigen::MatrixXd::Constant(VARIABLES, TIMESTEPS, 0.0);

TEST(NoiseGeneratorTest, testStartEndUnchanged)
{
  auto noise_gen = stomp_moveit::noise::getNormalDistributionGenerator(TIMESTEPS, STDDEV);

  auto noise = NOISE;
  auto noisy_values = NOISY_VALUES;

  noise_gen(VALUES, noisy_values, noise);

  // Test that noise creates output unlike the input
  EXPECT_NE(noise, NOISE);
  EXPECT_NE(noisy_values, NOISY_VALUES);
  EXPECT_NE(VALUES, noisy_values);

  // Test that the dimensions of the output matches the input
  EXPECT_EQ(noise.size(), NOISE.size());
  EXPECT_EQ(noisy_values.size(), NOISY_VALUES.size());

  // Test that start and end columns (=states) are not modified
  EXPECT_EQ(VALUES.col(0), noisy_values.col(0));
  EXPECT_EQ(VALUES.col(TIMESTEPS - 1), noisy_values.col(TIMESTEPS - 1));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
