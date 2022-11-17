/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik Inc.
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

/* Author: Boston Cleek
   Desc: move group interface ompl constrained planning capabilities for planning and execution
*/

// Testing
#include <gtest/gtest.h>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/macros/console_colors.h>
#include <moveit_msgs/msg/constraints.hpp>

// accuracy tested for position and orientation
static const double EPSILON = 1e-2;

static const std::string PLANNING_GROUP = "panda_arm";
static const double PLANNING_TIME_S = 60.0;
static const double MAX_VELOCITY_SCALE = 1.0;
static const double MAX_ACCELERATION_SCALE = 1.0;
static const double PLANNING_ATTEMPTS = 5.0;
static const double GOAL_TOLERANCE = 1e-5;

class ConstrainedPlanningTestFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, PLANNING_GROUP);
    move_group_->setPlanningTime(PLANNING_TIME_S);
    move_group_->setNumPlanningAttempts(PLANNING_ATTEMPTS);
    move_group_->setGoalTolerance(GOAL_TOLERANCE);
    move_group_->setMaxVelocityScalingFactor(MAX_VELOCITY_SCALE);
    move_group_->setMaxAccelerationScalingFactor(MAX_ACCELERATION_SCALE);

    ref_link_ = move_group_->getPoseReferenceFrame();
    ee_link_ = move_group_->getEndEffectorLink();

    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { this->executor_->spin(); });
  }

  ConstrainedPlanningTestFixture()
    : node_(std::make_shared<rclcpp::Node>("ompl_constrained_planning_testing"))
    , executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
  {
  }

  void TearDown() override
  {
    executor_->cancel();
    if (executor_thread_.joinable())
    {
      executor_thread_.join();
    }
  }

  void planAndMove(const geometry_msgs::msg::PoseStamped& pose_goal_stamped,
                   const moveit_msgs::msg::Constraints& path_constraint)
  {
    SCOPED_TRACE("planAndMove");

    ASSERT_TRUE(move_group_->setPoseTarget(pose_goal_stamped));
    move_group_->setPathConstraints(path_constraint);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    ASSERT_EQ(move_group_->plan(plan), moveit::core::MoveItErrorCode::SUCCESS);
    ASSERT_EQ(move_group_->move(), moveit::core::MoveItErrorCode::SUCCESS);
  }

  void testPose(const geometry_msgs::msg::PoseStamped& pose_goal_stamped)
  {
    SCOPED_TRACE("testPose");

    const geometry_msgs::msg::PoseStamped actual_pose_stamped = move_group_->getCurrentPose();
    Eigen::Isometry3d goal_pose, actual_pose;

    tf2::fromMsg(pose_goal_stamped.pose, goal_pose);
    tf2::fromMsg(actual_pose_stamped.pose, actual_pose);

    std::stringstream ss;
    ss << "expected: \n" << goal_pose.matrix() << "\nactual: \n" << actual_pose.matrix();
    EXPECT_TRUE(actual_pose.isApprox(goal_pose, EPSILON)) << ss.str();
  }

protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Executor::SharedPtr executor_;
  std::thread executor_thread_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  std::string ref_link_, ee_link_;
  // mutable std::mutex latest_state_mutex_;
};

TEST_F(ConstrainedPlanningTestFixture, PositionConstraint)
{
  SCOPED_TRACE("PositionConstraint");

  // Start
  move_group_->setStartStateToCurrentState();
  const geometry_msgs::msg::PoseStamped eef_pose = move_group_->getCurrentPose();

  // Create a goal
  geometry_msgs::msg::PoseStamped pose_goal = eef_pose;
  pose_goal.pose.position.y += 0.3;
  pose_goal.pose.position.z -= 0.3;

  // Box position constraint
  moveit_msgs::msg::PositionConstraint position_constraint;
  position_constraint.header.frame_id = ref_link_;
  position_constraint.link_name = ee_link_;
  position_constraint.weight = 1.0;

  shape_msgs::msg::SolidPrimitive cbox;
  cbox.type = shape_msgs::msg::SolidPrimitive::BOX;
  cbox.dimensions = { 0.1, 0.4, 0.4 };
  position_constraint.constraint_region.primitives.emplace_back(cbox);

  geometry_msgs::msg::Pose cbox_pose;
  cbox_pose.position.x = eef_pose.pose.position.x;
  cbox_pose.position.y = 0.15;
  cbox_pose.position.z = 0.45;
  cbox_pose.orientation.w = 1.0;
  position_constraint.constraint_region.primitive_poses.emplace_back(cbox_pose);

  // Create path constraint
  moveit_msgs::msg::Constraints path_constraint;
  path_constraint.name = "position constraint";
  path_constraint.position_constraints.emplace_back(position_constraint);

  planAndMove(pose_goal, path_constraint);

  testPose(pose_goal);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);

  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
