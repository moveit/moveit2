/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, MoveIt contributors
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
 *   * Neither the name of MoveIt nor the names of its
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

// Regression test for https://github.com/moveit/moveit2/issues/3848:
// a passive re-sync of the query start/goal state (e.g. the post-execution refresh of the
// "<current>" start state, or the continuous scene-monitor resync) must not silently steal the
// Joints tab's active/editable view away from whichever state the user is actually working with.

#include <gtest/gtest.h>
#include <QApplication>
#include <moveit/utils/robot_model_test_utils.hpp>
#include <moveit/robot_interaction/robot_interaction.hpp>
#include <moveit/robot_interaction/interaction_handler.hpp>
#include <moveit/motion_planning_rviz_plugin/motion_planning_frame_joints_widget.hpp>
#include <rclcpp/rclcpp.hpp>

using moveit_rviz_plugin::MotionPlanningFrameJointsWidget;

namespace
{
class JointsWidgetFocusTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    robot_model_ = moveit::core::loadTestingRobotModel("panda");
    ASSERT_TRUE(robot_model_ != nullptr);
    node_ = std::make_shared<rclcpp::Node>("joints_widget_focus_test_node");
    robot_interaction_ = std::make_shared<robot_interaction::RobotInteraction>(robot_model_, node_);

    moveit::core::RobotState start_state(robot_model_);
    start_state.setToDefaultValues();
    moveit::core::RobotState goal_state(robot_model_);
    goal_state.setToDefaultValues();

    start_handler_ =
        std::make_shared<robot_interaction::InteractionHandler>(robot_interaction_, "test_start", start_state);
    goal_handler_ =
        std::make_shared<robot_interaction::InteractionHandler>(robot_interaction_, "test_goal", goal_state);

    // display == nullptr is safe here: none of the slots exercised below ever call into it
    // (planning_display_ is only touched by JMGItemModel::dataChanged, which this test never
    // triggers, and is guarded by ignore_state_changes_ for the internal resync path anyway).
    widget_ = std::make_unique<MotionPlanningFrameJointsWidget>(nullptr);
    widget_->changePlanningGroup("panda_arm", start_handler_, goal_handler_);
  }

  moveit::core::RobotModelPtr robot_model_;
  rclcpp::Node::SharedPtr node_;
  robot_interaction::RobotInteractionPtr robot_interaction_;
  robot_interaction::InteractionHandlerPtr start_handler_;
  robot_interaction::InteractionHandlerPtr goal_handler_;
  std::unique_ptr<MotionPlanningFrameJointsWidget> widget_;
};

TEST_F(JointsWidgetFocusTest, GoalShownByDefault)
{
  EXPECT_TRUE(widget_->isShowingGoalState());
}

// This is the exact regression from #3848: after "Plan and Execute", MotionPlanningFrame
// refreshes the query *start* state to mirror the robot's live pose (since the Start State combo
// box defaults to "<current>"). That refresh must not silently switch the Joints tab away from
// the goal state the user was editing.
TEST_F(JointsWidgetFocusTest, PassiveStartStateSyncDoesNotStealGoalFocus)
{
  ASSERT_TRUE(widget_->isShowingGoalState());

  widget_->queryStartStateChanged(/*passive_sync=*/true);

  EXPECT_TRUE(widget_->isShowingGoalState())
      << "a passive '<current>' start-state resync (e.g. post-execution) must not steal the "
         "Joints tab's focus away from the goal state";
}

// Symmetric case: a passive goal-state resync (e.g. the continuous scene-monitor sync of a
// "<current>" goal) must not steal focus away from the start state either.
TEST_F(JointsWidgetFocusTest, PassiveGoalStateSyncDoesNotStealStartFocus)
{
  widget_->queryStartStateChanged(/*passive_sync=*/false);
  ASSERT_FALSE(widget_->isShowingGoalState());

  widget_->queryGoalStateChanged(/*passive_sync=*/true);

  EXPECT_FALSE(widget_->isShowingGoalState())
      << "a passive goal-state resync must not steal the Joints tab's focus away from the start "
         "state";
}

// Genuine, user-driven state changes (Start/Goal State combo box selection, or an interactive
// marker drag) must still switch the Joints tab's focus, exactly as before this fix.
TEST_F(JointsWidgetFocusTest, ExplicitStateChangeStillSwitchesFocus)
{
  ASSERT_TRUE(widget_->isShowingGoalState());

  widget_->queryStartStateChanged(/*passive_sync=*/false);
  EXPECT_FALSE(widget_->isShowingGoalState());

  widget_->queryGoalStateChanged(/*passive_sync=*/false);
  EXPECT_TRUE(widget_->isShowingGoalState());
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
