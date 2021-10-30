/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, PickNik Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/* Author: Andy Zelenak */

#include <gtest/gtest.h>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#include <moveit/utils/robot_model_test_utils.h>

namespace
{
constexpr double TIMESTEP = 0.01;  // sec

class RuckigTests : public testing::Test
{
protected:
  void SetUp() override
  {
    robot_model_ = moveit::core::loadTestingRobotModel("panda");
    trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, "panda_arm");
  }

  moveit::core::RobotModelPtr robot_model_;
  robot_trajectory::RobotTrajectoryPtr trajectory_;
  trajectory_processing::RuckigSmoothing smoother_;
};

}  // namespace

TEST_F(RuckigTests, empty_trajectory)
{
  // This should fail because the trajectory is empty
  EXPECT_FALSE(
      smoother_.applySmoothing(*trajectory_, 1.0 /* max vel scaling factor */, 1.0 /* max accel scaling factor */));
}

TEST_F(RuckigTests, basic_trajectory)
{
  // robot_state::RobotState robot_state(robot_model_);
  // trajectory_->addSuffixWayPoint(&robot_state, TIMESTEP);
  // trajectory_->addSuffixWayPoint(&robot_state, TIMESTEP);

  // EXPECT_TRUE(smoother_.applySmoothing(trajectory,
  //                                       1.0 /* max vel scaling factor */,
  //                                       1.0 /* max accel scaling factor */));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
