/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
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
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
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

#include <memory>

#include <gtest/gtest.h>

#include <pilz_industrial_motion_planner/trajectory_generator.h>

using namespace pilz_industrial_motion_planner;

/**
 * @brief Checks that each derived MoveItErrorCodeException contains the correct
 * error code.
 */
TEST(TrajectoryGeneratorTest, TestExceptionErrorCodeMapping)
{
  {
    auto tgil_ex = std::make_shared<TrajectoryGeneratorInvalidLimitsException>("");
    EXPECT_EQ(tgil_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::FAILURE);
  }

  {
    auto vsi_ex = std::make_shared<VelocityScalingIncorrect>("");
    EXPECT_EQ(vsi_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN);
  }

  {
    auto asi_ex = std::make_shared<AccelerationScalingIncorrect>("");
    EXPECT_EQ(asi_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN);
  }

  {
    auto upg_ex = std::make_shared<UnknownPlanningGroup>("");
    EXPECT_EQ(upg_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME);
  }

  {
    auto njniss_ex = std::make_shared<NoJointNamesInStartState>("");
    EXPECT_EQ(njniss_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE);
  }

  {
    auto smiss_ex = std::make_shared<SizeMismatchInStartState>("");
    EXPECT_EQ(smiss_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE);
  }

  {
    auto jofssoor_ex = std::make_shared<JointsOfStartStateOutOfRange>("");
    EXPECT_EQ(jofssoor_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE);
  }

  {
    auto nzviss_ex = std::make_shared<NonZeroVelocityInStartState>("");
    EXPECT_EQ(nzviss_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE);
  }

  {
    auto neogcg_ex = std::make_shared<NotExactlyOneGoalConstraintGiven>("");
    EXPECT_EQ(neogcg_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }

  {
    auto oogta_ex = std::make_shared<OnlyOneGoalTypeAllowed>("");
    EXPECT_EQ(oogta_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }

  {
    auto ssgsm_ex = std::make_shared<StartStateGoalStateMismatch>("");
    EXPECT_EQ(ssgsm_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }

  {
    auto jcdnbtg_ex = std::make_shared<JointConstraintDoesNotBelongToGroup>("");
    EXPECT_EQ(jcdnbtg_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }

  {
    auto jogoor_ex = std::make_shared<JointsOfGoalOutOfRange>("");
    EXPECT_EQ(jogoor_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }

  {
    auto pcnm_ex = std::make_shared<PositionConstraintNameMissing>("");
    EXPECT_EQ(pcnm_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }

  {
    auto ocnm_ex = std::make_shared<OrientationConstraintNameMissing>("");
    EXPECT_EQ(ocnm_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }

  {
    auto pocnm_ex = std::make_shared<PositionOrientationConstraintNameMismatch>("");
    EXPECT_EQ(pocnm_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }

  {
    auto nisa_ex = std::make_shared<NoIKSolverAvailable>("");
    EXPECT_EQ(nisa_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION);
  }

  {
    auto nppg_ex = std::make_shared<NoPrimitivePoseGiven>("");
    EXPECT_EQ(nppg_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
