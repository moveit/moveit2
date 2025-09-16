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

#include <boost/core/demangle.hpp>
#include <gtest/gtest.h>

#include <moveit/planning_interface/planning_interface.hpp>

#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>

#include <pilz_industrial_motion_planner/joint_limits_container.hpp>
#include <pilz_industrial_motion_planner/planning_context_circ.hpp>
#include <pilz_industrial_motion_planner/planning_context_lin.hpp>
#include <pilz_industrial_motion_planner/planning_context_ptp.hpp>

#include "test_utils.hpp"

// parameters from parameter server
const std::string PARAM_PLANNING_GROUP_NAME("planning_group");
const std::string PARAM_TARGET_LINK_NAME("target_link");

/**
 * A value type container to combine type and value
 * In the tests types are trajectory generators.
 * value = 0 refers to robot model without gripper
 * value = 1 refers to robot model with gripper
 */
template <typename T, int N>
class ValueTypeContainer
{
public:
  typedef T Type_;
  static const int VALUE = N;
};
template <typename T, int N>
const int ValueTypeContainer<T, N>::VALUE;

typedef ValueTypeContainer<pilz_industrial_motion_planner::PlanningContextPTP, 0> PTP_NO_GRIPPER;
typedef ValueTypeContainer<pilz_industrial_motion_planner::PlanningContextPTP, 1> PTP_WITH_GRIPPER;
typedef ValueTypeContainer<pilz_industrial_motion_planner::PlanningContextLIN, 0> LIN_NO_GRIPPER;
typedef ValueTypeContainer<pilz_industrial_motion_planner::PlanningContextLIN, 1> LIN_WITH_GRIPPER;
typedef ValueTypeContainer<pilz_industrial_motion_planner::PlanningContextCIRC, 0> CIRC_NO_GRIPPER;
typedef ValueTypeContainer<pilz_industrial_motion_planner::PlanningContextCIRC, 1> CIRC_WITH_GRIPPER;

typedef ::testing::Types<PTP_NO_GRIPPER, PTP_WITH_GRIPPER, LIN_NO_GRIPPER, LIN_WITH_GRIPPER, CIRC_NO_GRIPPER,
                         CIRC_WITH_GRIPPER>
    PlanningContextTestTypes;

/**
 * type parameterized test fixture
 */
template <typename T>
class PlanningContextTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node_ = rclcpp::Node::make_shared("unittest_planning_context", node_options);

    // load robot model
    rm_loader_ = std::make_unique<robot_model_loader::RobotModelLoader>(node_);
    robot_model_ = rm_loader_->getModel();
    ASSERT_FALSE(robot_model_ == nullptr) << "There is no robot model!";

    // get parameters
    ASSERT_TRUE(node_->has_parameter(PARAM_PLANNING_GROUP_NAME)) << "Could not find parameter 'planning_group'";
    node_->get_parameter<std::string>(PARAM_PLANNING_GROUP_NAME, planning_group_);

    ASSERT_TRUE(node_->has_parameter(PARAM_TARGET_LINK_NAME)) << "Could not find parameter 'target_link'";
    node_->get_parameter<std::string>(PARAM_TARGET_LINK_NAME, target_link_);

    pilz_industrial_motion_planner::JointLimitsContainer joint_limits =
        testutils::createFakeLimits(robot_model_->getVariableNames());

    cartesian_limits::Params cartesian_limit;
    cartesian_limit.max_trans_vel = 1.0 * M_PI;
    cartesian_limit.max_trans_acc = 1.0 * M_PI;
    cartesian_limit.max_trans_dec = 1.0 * M_PI;
    cartesian_limit.max_rot_vel = 1.0 * M_PI;

    pilz_industrial_motion_planner::LimitsContainer limits;
    limits.setJointLimits(joint_limits);
    limits.setCartesianLimits(cartesian_limit);

    auto interpolation_param_listener =
        std::make_shared<interpolation::ParamListener>(node_, "robot_description_planning.interpolation");

    planning_context_ = std::unique_ptr<typename T::Type_>(new typename T::Type_(
        "TestPlanningContext", planning_group_, robot_model_, limits, interpolation_param_listener));

    // Define and set the current scene
    auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model_);
    moveit::core::RobotState current_state(robot_model_);
    current_state.setToDefaultValues();
    current_state.setJointGroupPositions(planning_group_, std::vector<double>{ 0, 1.57, 1.57, 0, 0.2, 0 });
    scene->setCurrentState(current_state);
    planning_context_->setPlanningScene(scene);  // TODO Check what happens if this is missing
  }

  void TearDown() override
  {
    robot_model_.reset();
  }

  /**
   * @brief Generate a valid fully defined request
   */
  planning_interface::MotionPlanRequest getValidRequest(const std::string& context_name) const
  {
    planning_interface::MotionPlanRequest req;

    req.planner_id =
        std::string(context_name).erase(0, std::string("pilz_industrial_motion_planner::PlanningContext").length());
    req.group_name = this->planning_group_;
    req.max_velocity_scaling_factor = 0.01;
    req.max_acceleration_scaling_factor = 0.01;

    // start state
    moveit::core::RobotState rstate(this->robot_model_);
    rstate.setToDefaultValues();
    // state state in joint space, used as initial positions, since IK does not
    // work at zero positions
    rstate.setJointGroupPositions(this->planning_group_,
                                  std::vector<double>{ 4.430233957464225e-12, 0.007881892504574495, -1.8157263253868452,
                                                       1.1801525390026025e-11, 1.8236082178909834,
                                                       8.591793942969161e-12 });
    Eigen::Isometry3d start_pose(Eigen::Isometry3d::Identity());
    start_pose.translation() = Eigen::Vector3d(0.3, 0, 0.65);
    rstate.setFromIK(this->robot_model_->getJointModelGroup(this->planning_group_), start_pose);
    moveit::core::robotStateToRobotStateMsg(rstate, req.start_state, false);

    // goal constraint
    Eigen::Isometry3d goal_pose(Eigen::Isometry3d::Identity());
    goal_pose.translation() = Eigen::Vector3d(0, 0.3, 0.65);
    Eigen::Matrix3d goal_rotation;
    goal_rotation = Eigen::AngleAxisd(0 * M_PI, Eigen::Vector3d::UnitZ());
    goal_pose.linear() = goal_rotation;
    rstate.setFromIK(this->robot_model_->getJointModelGroup(this->planning_group_), goal_pose);
    req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(
        rstate, this->robot_model_->getJointModelGroup(this->planning_group_)));

    // path constraint
    req.path_constraints.name = "center";
    moveit_msgs::msg::PositionConstraint center_point;
    center_point.link_name = this->target_link_;
    geometry_msgs::msg::Pose center_position;
    center_position.position.x = 0.0;
    center_position.position.y = 0.0;
    center_position.position.z = 0.65;
    center_point.constraint_region.primitive_poses.push_back(center_position);
    req.path_constraints.position_constraints.push_back(center_point);

    return req;
  }

protected:
  // ros stuff
  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelConstPtr robot_model_;
  std::unique_ptr<robot_model_loader::RobotModelLoader> rm_loader_;

  std::unique_ptr<planning_interface::PlanningContext> planning_context_;

  std::string planning_group_, target_link_;
};

// Define the types we need to test
TYPED_TEST_SUITE(PlanningContextTest, PlanningContextTestTypes, /* ... */);

/**
 * @brief No request is set. Check the output of solve. Using robot model
 * without gripper.
 */
TYPED_TEST(PlanningContextTest, NoRequest)
{
  planning_interface::MotionPlanResponse res;
  this->planning_context_->solve(res);

  EXPECT_EQ(moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN, res.error_code.val)
      << testutils::demangle(typeid(TypeParam).name());
}

/**
 * @brief Solve a valid request.
 */
TYPED_TEST(PlanningContextTest, SolveValidRequest)
{
  planning_interface::MotionPlanResponse res;
  planning_interface::MotionPlanRequest req = this->getValidRequest(testutils::demangle(typeid(TypeParam).name()));

  this->planning_context_->setMotionPlanRequest(req);

  // TODO Formulate valid request
  this->planning_context_->solve(res);

  EXPECT_EQ(moveit_msgs::msg::MoveItErrorCodes::SUCCESS, res.error_code.val)
      << testutils::demangle(typeid(TypeParam).name());

  planning_interface::MotionPlanDetailedResponse res_detailed;
  this->planning_context_->solve(res_detailed);

  EXPECT_EQ(moveit_msgs::msg::MoveItErrorCodes::SUCCESS, res.error_code.val)
      << testutils::demangle(typeid(TypeParam).name());
}

/**
 * @brief Solve a valid request. Expect a detailed response.
 */
TYPED_TEST(PlanningContextTest, SolveValidRequestDetailedResponse)
{
  planning_interface::MotionPlanDetailedResponse res;  //<-- Detailed!
  planning_interface::MotionPlanRequest req = this->getValidRequest(testutils::demangle(typeid(TypeParam).name()));

  this->planning_context_->setMotionPlanRequest(req);
  this->planning_context_->solve(res);

  EXPECT_EQ(moveit_msgs::msg::MoveItErrorCodes::SUCCESS, res.error_code.val)
      << testutils::demangle(typeid(TypeParam).name());
}

/**
 * @brief Call solve on a terminated context.
 */
TYPED_TEST(PlanningContextTest, SolveOnTerminated)
{
  planning_interface::MotionPlanResponse res;
  planning_interface::MotionPlanRequest req = this->getValidRequest(testutils::demangle(typeid(TypeParam).name()));

  this->planning_context_->setMotionPlanRequest(req);

  bool result_termination = this->planning_context_->terminate();
  EXPECT_TRUE(result_termination) << testutils::demangle(typeid(TypeParam).name());

  this->planning_context_->solve(res);

  EXPECT_EQ(moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED, res.error_code.val)
      << testutils::demangle(typeid(TypeParam).name());
}

/**
 * @brief Check if clear can be called. So far only stability is expected.
 */
TYPED_TEST(PlanningContextTest, Clear)
{
  EXPECT_NO_THROW(this->planning_context_->clear()) << testutils::demangle(typeid(TypeParam).name());
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
