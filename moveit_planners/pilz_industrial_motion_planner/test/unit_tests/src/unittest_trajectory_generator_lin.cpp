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

#include <pilz_industrial_motion_planner/joint_limits_aggregator.hpp>
#include <pilz_industrial_motion_planner/trajectory_generator_lin.hpp>
#include <pilz_industrial_motion_planner_testutils/command_types_typedef.hpp>
#include <pilz_industrial_motion_planner_testutils/xml_testdata_loader.hpp>
#include "test_utils.hpp"

#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include <rclcpp/rclcpp.hpp>

using namespace pilz_industrial_motion_planner;
using namespace pilz_industrial_motion_planner_testutils;

static const std::string PARAM_NAMESPACE_LIMITS = "robot_description_planning";

// Parameter names
const std::string TEST_DATA_FILE_NAME("testdata_file_name");
const std::string PARAM_PLANNING_GROUP_NAME("planning_group");
const std::string TARGET_LINK_HCD("target_link_hand_computed_data");
const std::string RANDOM_TEST_TRIAL_NUM("random_trial_number");
const std::string JOINT_POSITION_TOLERANCE("joint_position_tolerance");
const std::string JOINT_VELOCITY_TOLERANCE("joint_velocity_tolerance");
const std::string POSE_TRANSFORM_MATRIX_NORM_TOLERANCE("pose_norm_tolerance");
const std::string ROTATION_AXIS_NORM_TOLERANCE("rot_axis_norm_tolerance");
const std::string VELOCITY_SCALING_FACTOR("velocity_scaling_factor");
const std::string OTHER_TOLERANCE("other_tolerance");

/**
 * @brief Parameterized unittest of trajectory generator LIN to enable tests
 * against
 * different robot models.The parameter is the name of robot model parameter on
 * the
 * ros parameter server.
 */
class TrajectoryGeneratorLINTest : public testing::Test
{
protected:
  /**
   * @brief Create test scenario for lin trajectory generator
   *
   */
  void SetUp() override
  {
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node_ = rclcpp::Node::make_shared("unittest_trajectory_generator_lin", node_options);

    // load robot model
    rm_loader_ = std::make_unique<robot_model_loader::RobotModelLoader>(node_);
    robot_model_ = rm_loader_->getModel();
    ASSERT_TRUE(bool(robot_model_)) << "Failed to load robot model";
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);

    // get parameters
    ASSERT_TRUE(node_->get_parameter(TEST_DATA_FILE_NAME, test_data_file_name_));
    ASSERT_TRUE(node_->get_parameter(PARAM_PLANNING_GROUP_NAME, planning_group_));
    ASSERT_TRUE(node_->get_parameter(TARGET_LINK_HCD, target_link_hcd_));
    ASSERT_TRUE(node_->get_parameter(RANDOM_TEST_TRIAL_NUM, random_trial_num_));
    ASSERT_TRUE(node_->get_parameter(JOINT_POSITION_TOLERANCE, joint_position_tolerance_));
    ASSERT_TRUE(node_->get_parameter(JOINT_VELOCITY_TOLERANCE, joint_velocity_tolerance_));
    ASSERT_TRUE(node_->get_parameter(POSE_TRANSFORM_MATRIX_NORM_TOLERANCE, pose_norm_tolerance_));
    ASSERT_TRUE(node_->get_parameter(ROTATION_AXIS_NORM_TOLERANCE, rot_axis_norm_tolerance_));
    ASSERT_TRUE(node_->get_parameter(VELOCITY_SCALING_FACTOR, velocity_scaling_factor_));
    ASSERT_TRUE(node_->get_parameter(OTHER_TOLERANCE, other_tolerance_));

    testutils::checkRobotModel(robot_model_, planning_group_, target_link_hcd_);

    // load the test data provider
    tdp_ = std::make_unique<pilz_industrial_motion_planner_testutils::XmlTestdataLoader>(test_data_file_name_);
    ASSERT_NE(nullptr, tdp_) << "Failed to load test data by provider.";

    tdp_->setRobotModel(robot_model_);

    // create the limits container
    // TODO, move this also into test data set
    pilz_industrial_motion_planner::JointLimitsContainer joint_limits =
        pilz_industrial_motion_planner::JointLimitsAggregator::getAggregatedLimits(
            node_, PARAM_NAMESPACE_LIMITS, robot_model_->getActiveJointModels());

    cartesian_limits::Params cart_limits;
    cart_limits.max_rot_vel = 0.5 * M_PI;
    cart_limits.max_trans_acc = 2;
    cart_limits.max_trans_dec = 2;
    cart_limits.max_trans_vel = 1;

    planner_limits_.setJointLimits(joint_limits);
    planner_limits_.setCartesianLimits(cart_limits);

    // initialize the LIN trajectory generator
    lin_ = std::make_unique<TrajectoryGeneratorLIN>(robot_model_, planner_limits_, planning_group_);
    ASSERT_NE(nullptr, lin_) << "Failed to create LIN trajectory generator.";
  }

  void TearDown() override
  {
    robot_model_.reset();
  }

  bool checkLinResponse(const planning_interface::MotionPlanRequest& req,
                        const planning_interface::MotionPlanResponse& res)
  {
    moveit_msgs::msg::MotionPlanResponse res_msg;
    res.getMessage(res_msg);
    if (!testutils::isGoalReached(robot_model_, res_msg.trajectory.joint_trajectory, req, pose_norm_tolerance_))
    {
      return false;
    }

    if (!testutils::checkCartesianLinearity(robot_model_, res_msg.trajectory.joint_trajectory, req,
                                            pose_norm_tolerance_, rot_axis_norm_tolerance_))
    {
      return false;
    }

    if (!testutils::checkJointTrajectory(res_msg.trajectory.joint_trajectory, planner_limits_.getJointLimitContainer()))
    {
      return false;
    }

    return true;
  }

protected:
  // ros stuff
  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelConstPtr robot_model_;
  std::unique_ptr<robot_model_loader::RobotModelLoader> rm_loader_;
  planning_scene::PlanningSceneConstPtr planning_scene_;

  // lin trajectory generator using model without gripper
  std::unique_ptr<TrajectoryGenerator> lin_;
  // test data provider
  std::unique_ptr<pilz_industrial_motion_planner_testutils::TestdataLoader> tdp_;

  // test parameters from parameter server
  std::string planning_group_, target_link_hcd_, test_data_file_name_;
  int random_trial_num_;
  double joint_position_tolerance_, joint_velocity_tolerance_, pose_norm_tolerance_, rot_axis_norm_tolerance_,
      velocity_scaling_factor_, other_tolerance_;
  LimitsContainer planner_limits_;
};

/**
 * @brief Checks that each derived MoveItErrorCodeException contains the correct
 * error code.
 */
TEST_F(TrajectoryGeneratorLINTest, TestExceptionErrorCodeMapping)
{
  {
    auto ltcf_ex = std::make_shared<LinTrajectoryConversionFailure>("");
    EXPECT_EQ(ltcf_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::FAILURE);
  }

  {
    auto jnm_ex = std::make_shared<JointNumberMismatch>("");
    EXPECT_EQ(jnm_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }

  {
    auto lifgi_ex = std::make_shared<LinInverseForGoalIncalculable>("");
    EXPECT_EQ(lifgi_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION);
  }
}

/**
 * @brief test the lin planner with invalid motion plan request which has non
 * zero start velocity
 */
TEST_F(TrajectoryGeneratorLINTest, nonZeroStartVelocity)
{
  planning_interface::MotionPlanRequest req{ tdp_->getLinJoint("lin2").toRequest() };

  // add non-zero velocity in the start state
  req.start_state.joint_state.velocity.push_back(1.0);

  // try to generate the result
  planning_interface::MotionPlanResponse res;
  lin_->generate(planning_scene_, req, res);
  EXPECT_EQ(res.error_code.val, moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE);
}

/**
 * @brief test the lin planner with joint space goal
 */
TEST_F(TrajectoryGeneratorLINTest, jointSpaceGoal)
{
  planning_interface::MotionPlanRequest lin_joint_req{ tdp_->getLinJoint("lin2").toRequest() };

  // generate the LIN trajectory
  planning_interface::MotionPlanResponse res;
  lin_->generate(planning_scene_, lin_joint_req, res);
  EXPECT_TRUE(res.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);

  // check the resulted trajectory
  EXPECT_TRUE(checkLinResponse(lin_joint_req, res));
}

/**
 * @brief test the lin planner with joint space goal with start velocity almost
 * zero
 */
TEST_F(TrajectoryGeneratorLINTest, jointSpaceGoalNearZeroStartVelocity)
{
  planning_interface::MotionPlanRequest lin_joint_req{ tdp_->getLinJoint("lin2").toRequest() };

  // Set velocity near zero
  lin_joint_req.start_state.joint_state.velocity =
      std::vector<double>(lin_joint_req.start_state.joint_state.position.size(), 1e-16);

  // generate the LIN trajectory
  planning_interface::MotionPlanResponse res;
  lin_->generate(planning_scene_, lin_joint_req, res);
  EXPECT_TRUE(res.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);

  // check the resulted trajectory
  EXPECT_TRUE(checkLinResponse(lin_joint_req, res));
}

/**
 * @brief test the lin planner with Cartesian goal
 */
TEST_F(TrajectoryGeneratorLINTest, cartesianSpaceGoal)
{
  // construct motion plan request
  moveit_msgs::msg::MotionPlanRequest lin_cart_req{ tdp_->getLinCart("lin2").toRequest() };

  // generate lin trajectory
  planning_interface::MotionPlanResponse res;
  lin_->generate(planning_scene_, lin_cart_req, res);
  EXPECT_TRUE(res.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);

  // check the resulted trajectory
  EXPECT_TRUE(checkLinResponse(lin_cart_req, res));
}

/**
 * @brief test the trapezoid shape of the planning trajectory in Cartesian space
 *
 * The test checks translational path for a trapezoid velocity profile.
 * Due to the way the acceleration is calculated 1 or 2 intermediate points
 * occur that are neither
 * acceleration, constant or deceleration.
 */
TEST_F(TrajectoryGeneratorLINTest, cartesianTrapezoidProfile)
{
  // construct motion plan request
  moveit_msgs::msg::MotionPlanRequest lin_joint_req{ tdp_->getLinJoint("lin2").toRequest() };

  /// +++++++++++++++++++++++
  /// + plan LIN trajectory +
  /// +++++++++++++++++++++++
  planning_interface::MotionPlanResponse res;
  interpolation::Params interpolation_params;
  interpolation_params.max_sample_time = 0.01;
  lin_->generate(planning_scene_, lin_joint_req, res, interpolation_params);
  EXPECT_EQ(res.error_code.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);

  ASSERT_TRUE(testutils::checkCartesianTranslationalPath(res.trajectory, target_link_hcd_));

  // check last point for vel=acc=0
  for (size_t idx = 0; idx < res.trajectory->getLastWayPointPtr()->getVariableCount(); ++idx)
  {
    EXPECT_NEAR(0.0, res.trajectory->getLastWayPointPtr()->getVariableVelocity(idx), other_tolerance_);
    EXPECT_NEAR(0.0, res.trajectory->getLastWayPointPtr()->getVariableAcceleration(idx), other_tolerance_);
  }
}

/**
 * @brief Unit test for the TrajectoryGeneratorLIN class to verify interpolation parameters.
 *
 * This test constructs a motion plan request and sets various interpolation parameters to check
 * the behavior of the linear trajectory generation. It verifies the following:
 * - Successful trajectory generation with default parameters.
 * - Failure of interpolation when the max translation interpolation distance is set to a very small value.
 * - Failure of interpolation when the max rotation interpolation distance is set to a very small value.
 * - Successful trajectory generation and interpolation with specific test parameters.
 */
TEST_F(TrajectoryGeneratorLINTest, interpolationParameters)
{
  // construct motion plan request
  moveit_msgs::msg::MotionPlanRequest lin_joint_req{ tdp_->getLinJoint("lin2").toRequest() };

  interpolation::Params default_params;
  interpolation::Params interpolation_params;
  interpolation::Params test_params;
  test_params.max_sample_time = 0.05;
  test_params.max_translation_interpolation_distance = 0.001;
  test_params.max_rotation_interpolation_distance = 0.001;

  /// +++++++++++++++++++++++
  /// + plan LIN trajectory +
  /// +++++++++++++++++++++++
  planning_interface::MotionPlanResponse res;
  interpolation_params.max_sample_time = test_params.max_sample_time;
  lin_->generate(planning_scene_, lin_joint_req, res, interpolation_params);
  EXPECT_EQ(res.error_code.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);

  // set the max translation interpolation distance to a very small value
  // and check that the interpolation fails
  interpolation_params.max_translation_interpolation_distance = test_params.max_translation_interpolation_distance;
  ASSERT_FALSE(testutils::checkInterpolationParameters(res.trajectory, target_link_hcd_, interpolation_params));
  interpolation_params.max_translation_interpolation_distance = default_params.max_translation_interpolation_distance;

  // set the max rotation interpolation distance to a very small value
  // and check that the interpolation fails
  interpolation_params.max_rotation_interpolation_distance = test_params.max_rotation_interpolation_distance;
  ASSERT_FALSE(testutils::checkInterpolationParameters(res.trajectory, target_link_hcd_, interpolation_params));

  // recompute the trajectory with the same interpolation parameters
  // and check that the interpolation is successful
  lin_->generate(planning_scene_, lin_joint_req, res, test_params);
  ASSERT_TRUE(testutils::checkInterpolationParameters(res.trajectory, target_link_hcd_, test_params));
}

/**
 * @brief Test case for verifying the interpolation parameters in the LIN trajectory generator.
 *
 * This test constructs a motion plan request for a linear joint motion and sets the interpolation parameters.
 * It then generates a LIN trajectory and checks if the interpolation parameters are correctly applied.
 * The test verifies the following:
 * - The trajectory generation is successful with the initial interpolation parameters.
 * - The interpolation parameters are correctly checked against the generated trajectory.
 * - The trajectory generation is successful with modified interpolation parameters.
 */
// TEST_F(TrajectoryGeneratorLINTest, interpolationParametersNumericalIK)
// {
//   // construct motion plan request
//   moveit_msgs::msg::MotionPlanRequest lin_joint_req{ tdp_->getLinJoint("lin2").toRequest() };

//   interpolation::Params interpolation_params;
//   interpolation_params.max_sample_time = 0.01;
//   interpolation_params.max_translation_interpolation_distance = 0.001;
//   interpolation_params.max_rotation_interpolation_distance = 1.0;

//   /// +++++++++++++++++++++++
//   /// + plan LIN trajectory +
//   /// +++++++++++++++++++++++
//   planning_interface::MotionPlanResponse res;
//   lin_->generate(planning_scene_, lin_joint_req, res, interpolation_params);
//   EXPECT_EQ(res.error_code.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
//   ASSERT_TRUE(testutils::checkInterpolationParameters(res.trajectory, target_link_hcd_, interpolation_params));

//   interpolation_params.min_translation_interpolation_distance = 5e-4;
//   interpolation_params.min_rotation_interpolation_distance = 5e-4;
//   ASSERT_FALSE(testutils::checkInterpolationParameters(res.trajectory, target_link_hcd_, interpolation_params));

//   // recompute the trajectory with the same interpolation parameters
//   // and check that the interpolation is successful
//   lin_->generate(planning_scene_, lin_joint_req, res, interpolation_params);
//   ASSERT_TRUE(testutils::checkInterpolationParameters(res.trajectory, target_link_hcd_, interpolation_params));
// }

/**
 * @brief Check that lin planner returns 'false' if
 * calculated lin trajectory violates velocity/acceleration or deceleration
 * limits.
 *
 *
 * Test Sequence:
 *    1. Call function with lin request violating velocity/acceleration or
 * deceleration limits.
 *
 * Expected Results:
 *    1. Function returns 'false'.
 */
TEST_F(TrajectoryGeneratorLINTest, LinPlannerLimitViolation)
{
  LinJoint lin{ tdp_->getLinJoint("lin2") };
  lin.setAccelerationScale(1.01);

  planning_interface::MotionPlanResponse res;
  lin_->generate(planning_scene_, lin.toRequest(), res);
  ASSERT_FALSE(res.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
}

/**
 * @brief test joint linear movement with discontinuities in joint space
 *
 * This will violate joint velocity/acceleration limits.
 *
 * Test Sequence:
 *    1. Generate lin trajectory which is discontinuous in joint space.
 *
 * Expected Results:
 *    1. Function returns 'false'.
 */
TEST_F(TrajectoryGeneratorLINTest, LinPlannerDiscontinuousJointTraj)
{
  LinJoint lin{ tdp_->getLinJoint("lin2") };
  // Alter goal joint configuration (represents the same cartesian pose, but
  // does not fit together with start config)
  lin.getGoalConfiguration().setJoint(1, 1.63);
  lin.getGoalConfiguration().setJoint(2, 0.96);
  lin.getGoalConfiguration().setJoint(4, -2.48);
  lin.setVelocityScale(1.0);
  lin.setAccelerationScale(1.0);

  planning_interface::MotionPlanResponse res;
  lin_->generate(planning_scene_, lin.toRequest(), res);
  ASSERT_FALSE(res.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
}

/**
 * @brief test joint linear movement with equal goal and start
 *
 * Test Sequence:
 *    1. Call function with lin request start = goal
 *
 * Expected Results:
 *    1. trajectory generation is successful.
 */
TEST_F(TrajectoryGeneratorLINTest, LinStartEqualsGoal)
{
  // construct motion plan request
  moveit_msgs::msg::MotionPlanRequest lin_joint_req{ tdp_->getLinJoint("lin2").toRequest() };

  moveit::core::RobotState start_state(robot_model_);
  jointStateToRobotState(lin_joint_req.start_state.joint_state, start_state);

  for (auto& joint_constraint : lin_joint_req.goal_constraints.at(0).joint_constraints)
  {
    joint_constraint.position = start_state.getVariablePosition(joint_constraint.joint_name);
  }

  // generate the LIN trajectory
  planning_interface::MotionPlanResponse res;
  lin_->generate(planning_scene_, lin_joint_req, res);
  EXPECT_TRUE(res.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);

  // check the resulted trajectory
  EXPECT_TRUE(checkLinResponse(lin_joint_req, res));
}

/**
 * @brief Checks that generate() function returns 'false' if called with an
 * incorrect number of joints.
 *
 * Test Sequence:
 *    1. Call functions with incorrect number of joints.
 *
 * Expected Results:
 *    1. Function returns 'false'.
 */
TEST_F(TrajectoryGeneratorLINTest, IncorrectJointNumber)
{
  // construct motion plan request
  moveit_msgs::msg::MotionPlanRequest lin_joint_req{ tdp_->getLinJoint("lin2").toRequest() };

  // Ensure that request consists of an incorrect number of joints.
  lin_joint_req.goal_constraints.front().joint_constraints.pop_back();

  // generate the LIN trajectory
  planning_interface::MotionPlanResponse res;
  lin_->generate(planning_scene_, lin_joint_req, res);
  EXPECT_TRUE(res.error_code.val == moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
}

/**
 * @brief Set a frame id in goal constraint with cartesian goal on both position
 * and orientation constraints
 */
TEST_F(TrajectoryGeneratorLINTest, cartGoalFrameIdBothConstraints)
{
  // construct motion plan request
  moveit_msgs::msg::MotionPlanRequest lin_cart_req{ tdp_->getLinCart("lin2").toRequest() };

  lin_cart_req.goal_constraints.front().position_constraints.front().header.frame_id = robot_model_->getModelFrame();
  lin_cart_req.goal_constraints.front().orientation_constraints.front().header.frame_id = robot_model_->getModelFrame();

  // generate lin trajectory
  planning_interface::MotionPlanResponse res;
  lin_->generate(planning_scene_, lin_cart_req, res);
  EXPECT_TRUE(res.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);

  // check the resulted trajectory
  EXPECT_TRUE(checkLinResponse(lin_cart_req, res));
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
