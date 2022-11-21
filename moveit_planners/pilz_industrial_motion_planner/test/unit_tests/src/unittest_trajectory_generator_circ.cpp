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

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pilz_industrial_motion_planner/joint_limits_aggregator.h>
#include <pilz_industrial_motion_planner/trajectory_generator_circ.h>
#include <pilz_industrial_motion_planner_testutils/command_types_typedef.h>
#include <pilz_industrial_motion_planner_testutils/xml_testdata_loader.h>
#include "test_utils.h"

#include <rclcpp/rclcpp.hpp>

using namespace pilz_industrial_motion_planner;
using namespace pilz_industrial_motion_planner_testutils;

static const std::string PARAM_NAMESPACE_LIMITS = "robot_description_planning";

class TrajectoryGeneratorCIRCTest : public testing::Test
{
protected:
  /**
   * @brief Create test scenario for circ trajectory generator
   *
   */
  void SetUp() override
  {
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node_ = rclcpp::Node::make_shared("unittest_trajectory_generator_circ", node_options);

    // load robot model
    robot_model_loader::RobotModelLoader rm_loader(node_);
    robot_model_ = rm_loader.getModel();
    ASSERT_TRUE(bool(robot_model_)) << "Failed to load robot model";
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);

    // get parameters
    ASSERT_TRUE(node_->has_parameter("testdata_file_name"));
    node_->get_parameter<std::string>("testdata_file_name", test_data_file_name_);
    ASSERT_TRUE(node_->has_parameter("planning_group"));
    node_->get_parameter<std::string>("planning_group", planning_group_);
    ASSERT_TRUE(node_->has_parameter("target_link"));
    node_->get_parameter<std::string>("target_link", target_link_);
    ASSERT_TRUE(node_->has_parameter("cartesian_position_tolerance"));
    node_->get_parameter<double>("cartesian_position_tolerance", cartesian_position_tolerance_);
    ASSERT_TRUE(node_->has_parameter("angular_acc_tolerance"));
    node_->get_parameter<double>("angular_acc_tolerance", angular_acc_tolerance_);
    ASSERT_TRUE(node_->has_parameter("rot_axis_norm_tolerance"));
    node_->get_parameter<double>("rot_axis_norm_tolerance", rot_axis_norm_tolerance_);
    ASSERT_TRUE(node_->has_parameter("acceleration_tolerance"));
    node_->get_parameter<double>("acceleration_tolerance", acceleration_tolerance_);
    ASSERT_TRUE(node_->has_parameter("other_tolerance"));
    node_->get_parameter<double>("other_tolerance", other_tolerance_);

    // check robot model
    testutils::checkRobotModel(robot_model_, planning_group_, target_link_);

    // load the test data provider
    tdp_ = std::make_unique<pilz_industrial_motion_planner_testutils::XmlTestdataLoader>(test_data_file_name_);
    ASSERT_NE(nullptr, tdp_) << "Failed to load test data by provider.";

    tdp_->setRobotModel(robot_model_);

    // create the limits container
    pilz_industrial_motion_planner::JointLimitsContainer joint_limits =
        pilz_industrial_motion_planner::JointLimitsAggregator::getAggregatedLimits(
            node_, PARAM_NAMESPACE_LIMITS, robot_model_->getActiveJointModels());

    // Cartesian limits are chose as such values to ease the manually compute the
    // trajectory
    cartesian_limits::Params cartesian_limit;
    cartesian_limit.max_trans_vel = 1.0 * M_PI;
    cartesian_limit.max_trans_acc = 1.0 * M_PI;
    cartesian_limit.max_trans_dec = 1.0 * M_PI;
    cartesian_limit.max_rot_vel = 1.0 * M_PI;

    planner_limits_.setJointLimits(joint_limits);
    planner_limits_.setCartesianLimits(cartesian_limit);

    // initialize the LIN trajectory generator
    circ_ = std::make_unique<TrajectoryGeneratorCIRC>(robot_model_, planner_limits_, planning_group_);
    ASSERT_NE(nullptr, circ_) << "failed to create CIRC trajectory generator";
  }

  void checkCircResult(const planning_interface::MotionPlanRequest& req,
                       const planning_interface::MotionPlanResponse& res)
  {
    moveit_msgs::msg::MotionPlanResponse res_msg;
    res.getMessage(res_msg);
    EXPECT_TRUE(testutils::isGoalReached(res.trajectory_->getFirstWayPointPtr()->getRobotModel(),
                                         res_msg.trajectory.joint_trajectory, req, other_tolerance_));

    EXPECT_TRUE(
        testutils::checkJointTrajectory(res_msg.trajectory.joint_trajectory, planner_limits_.getJointLimitContainer()));

    EXPECT_EQ(req.path_constraints.position_constraints.size(), 1u);
    EXPECT_EQ(req.path_constraints.position_constraints.at(0).constraint_region.primitive_poses.size(), 1u);

    // Check that all point have the equal distance to the center
    Eigen::Vector3d circ_center;
    getCircCenter(req, res, circ_center);

    for (std::size_t i = 0; i < res.trajectory_->getWayPointCount(); ++i)
    {
      Eigen::Affine3d waypoint_pose = res.trajectory_->getWayPointPtr(i)->getFrameTransform(target_link_);
      EXPECT_NEAR(
          (res.trajectory_->getFirstWayPointPtr()->getFrameTransform(target_link_).translation() - circ_center).norm(),
          (circ_center - waypoint_pose.translation()).norm(), cartesian_position_tolerance_);
    }

    // check translational and rotational paths
    ASSERT_TRUE(testutils::checkCartesianTranslationalPath(res.trajectory_, target_link_, acceleration_tolerance_));
    ASSERT_TRUE(testutils::checkCartesianRotationalPath(res.trajectory_, target_link_, angular_acc_tolerance_,
                                                        rot_axis_norm_tolerance_));

    for (size_t idx = 0; idx < res.trajectory_->getLastWayPointPtr()->getVariableCount(); ++idx)
    {
      EXPECT_NEAR(0.0, res.trajectory_->getLastWayPointPtr()->getVariableVelocity(idx), other_tolerance_);
      EXPECT_NEAR(0.0, res.trajectory_->getLastWayPointPtr()->getVariableAcceleration(idx), other_tolerance_);
    }
  }

  void getCircCenter(const planning_interface::MotionPlanRequest& req,
                     const planning_interface::MotionPlanResponse& res, Eigen::Vector3d& circ_center)
  {
    if (req.path_constraints.name == "center")
    {
      tf2::fromMsg(req.path_constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).position,
                   circ_center);
    }
    else if (req.path_constraints.name == "interim")
    {
      Eigen::Vector3d interim;
      tf2::fromMsg(req.path_constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).position,
                   interim);
      Eigen::Vector3d start = res.trajectory_->getFirstWayPointPtr()->getFrameTransform(target_link_).translation();
      Eigen::Vector3d goal = res.trajectory_->getLastWayPointPtr()->getFrameTransform(target_link_).translation();

      const Eigen::Vector3d t = interim - start;
      const Eigen::Vector3d u = goal - start;
      const Eigen::Vector3d v = goal - interim;

      const Eigen::Vector3d w = t.cross(u);

      ASSERT_GT(w.norm(), 1e-8) << "Circle center not well defined for given start, interim and goal.";

      circ_center = start + (u * t.dot(t) * u.dot(v) - t * u.dot(u) * t.dot(v)) * 0.5 / pow(w.norm(), 2);
    }
  }

protected:
  // ros stuff
  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelConstPtr robot_model_;
  planning_scene::PlanningSceneConstPtr planning_scene_;

  std::unique_ptr<TrajectoryGeneratorCIRC> circ_;
  // test data provider
  std::unique_ptr<pilz_industrial_motion_planner_testutils::TestdataLoader> tdp_;

  // test parameters from parameter server
  std::string planning_group_, target_link_, test_data_file_name_;
  int random_trial_num_;
  double cartesian_position_tolerance_, angular_acc_tolerance_, rot_axis_norm_tolerance_, acceleration_tolerance_,
      other_tolerance_;
  LimitsContainer planner_limits_;
};

/**
 * @brief Checks that each derived MoveItErrorCodeException contains the correct
 * error code.
 */
TEST_F(TrajectoryGeneratorCIRCTest, TestExceptionErrorCodeMapping)
{
  {
    std::shared_ptr<CircleNoPlane> cnp_ex{ new CircleNoPlane("") };
    EXPECT_EQ(cnp_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN);
  }

  {
    std::shared_ptr<CircleToSmall> cts_ex{ new CircleToSmall("") };
    EXPECT_EQ(cts_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN);
  }

  {
    std::shared_ptr<CenterPointDifferentRadius> cpdr_ex{ new CenterPointDifferentRadius("") };
    EXPECT_EQ(cpdr_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN);
  }

  {
    std::shared_ptr<CircTrajectoryConversionFailure> ctcf_ex{ new CircTrajectoryConversionFailure("") };
    EXPECT_EQ(ctcf_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN);
  }

  {
    std::shared_ptr<UnknownPathConstraintName> upcn_ex{ new UnknownPathConstraintName("") };
    EXPECT_EQ(upcn_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN);
  }

  {
    std::shared_ptr<NoPositionConstraints> npc_ex{ new NoPositionConstraints("") };
    EXPECT_EQ(npc_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN);
  }

  {
    std::shared_ptr<NoPrimitivePose> npp_ex{ new NoPrimitivePose("") };
    EXPECT_EQ(npp_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN);
  }

  {
    std::shared_ptr<UnknownLinkNameOfAuxiliaryPoint> ulnoap_ex{ new UnknownLinkNameOfAuxiliaryPoint("") };
    EXPECT_EQ(ulnoap_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::INVALID_LINK_NAME);
  }

  {
    std::shared_ptr<NumberOfConstraintsMismatch> nocm_ex{ new NumberOfConstraintsMismatch("") };
    EXPECT_EQ(nocm_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }

  {
    std::shared_ptr<CircJointMissingInStartState> cjmiss_ex{ new CircJointMissingInStartState("") };
    EXPECT_EQ(cjmiss_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE);
  }

  {
    std::shared_ptr<CircInverseForGoalIncalculable> cifgi_ex{ new CircInverseForGoalIncalculable("") };
    EXPECT_EQ(cifgi_ex->getErrorCode(), moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION);
  }
}

/**
 * @brief test invalid motion plan request with incomplete start state and
 * cartesian goal
 */
TEST_F(TrajectoryGeneratorCIRCTest, incompleteStartState)
{
  auto circ{ tdp_->getCircCartCenterCart("circ1_center_2") };

  planning_interface::MotionPlanRequest req{ circ.toRequest() };
  EXPECT_GT(req.start_state.joint_state.name.size(), 1u);
  req.start_state.joint_state.name.resize(1);
  req.start_state.joint_state.position.resize(1);  // prevent failing check for equal sizes

  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(planning_scene_, req, res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE);
}

/**
 * @brief test invalid motion plan request with non zero start velocity
 */
TEST_F(TrajectoryGeneratorCIRCTest, nonZeroStartVelocity)
{
  moveit_msgs::msg::MotionPlanRequest req{ tdp_->getCircJointCenterCart("circ1_center_2").toRequest() };

  // start state has non-zero velocity
  req.start_state.joint_state.velocity.push_back(1.0);
  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(planning_scene_, req, res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE);
  req.start_state.joint_state.velocity.clear();
}

TEST_F(TrajectoryGeneratorCIRCTest, ValidCommand)
{
  auto circ{ tdp_->getCircCartCenterCart("circ1_center_2") };

  planning_interface::MotionPlanResponse res;
  EXPECT_TRUE(circ_->generate(planning_scene_, circ.toRequest(), res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
}

/**
 * @brief Generate invalid circ with to high vel scaling
 */
TEST_F(TrajectoryGeneratorCIRCTest, velScaleToHigh)
{
  auto circ{ tdp_->getCircCartCenterCart("circ1_center_2") };

  circ.setVelocityScale(1.0);
  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(planning_scene_, circ.toRequest(), res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED);
}

/**
 * @brief Generate invalid circ with to high acc scaling
 */
TEST_F(TrajectoryGeneratorCIRCTest, accScaleToHigh)
{
  auto circ{ tdp_->getCircCartCenterCart("circ1_center_2") };

  circ.setAccelerationScale(1.0);
  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(planning_scene_, circ.toRequest(), res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED);
}

/**
 * @brief Use three points (with center) with a really small distance between to
 * trigger a internal throw from KDL
 */
TEST_F(TrajectoryGeneratorCIRCTest, samePointsWithCenter)
{
  // Define auxiliary point and goal to be the same as the start
  auto circ{ tdp_->getCircCartCenterCart("circ1_center_2") };
  circ.getAuxiliaryConfiguration().getConfiguration().setPose(circ.getStartConfiguration().getPose());
  circ.getAuxiliaryConfiguration().getConfiguration().getPose().position.x += 1e-8;
  circ.getAuxiliaryConfiguration().getConfiguration().getPose().position.y += 1e-8;
  circ.getAuxiliaryConfiguration().getConfiguration().getPose().position.z += 1e-8;
  circ.getGoalConfiguration().setPose(circ.getStartConfiguration().getPose());
  circ.getGoalConfiguration().getPose().position.x -= 1e-8;
  circ.getGoalConfiguration().getPose().position.y -= 1e-8;
  circ.getGoalConfiguration().getPose().position.z -= 1e-8;

  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(planning_scene_, circ.toRequest(), res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN);
}

/**
 * @brief Use three points (with interim) with a really small distance between
 *
 * Expected: Planning should fail.
 */
TEST_F(TrajectoryGeneratorCIRCTest, samePointsWithInterim)
{
  // Define auxiliary point and goal to be the same as the start
  auto circ{ tdp_->getCircCartInterimCart("circ3_interim") };
  circ.getAuxiliaryConfiguration().getConfiguration().setPose(circ.getStartConfiguration().getPose());
  circ.getAuxiliaryConfiguration().getConfiguration().getPose().position.x += 1e-8;
  circ.getAuxiliaryConfiguration().getConfiguration().getPose().position.y += 1e-8;
  circ.getAuxiliaryConfiguration().getConfiguration().getPose().position.z += 1e-8;
  circ.getGoalConfiguration().setPose(circ.getStartConfiguration().getPose());
  circ.getGoalConfiguration().getPose().position.x -= 1e-8;
  circ.getGoalConfiguration().getPose().position.y -= 1e-8;
  circ.getGoalConfiguration().getPose().position.z -= 1e-8;

  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(planning_scene_, circ.toRequest(), res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN);
}

/**
 * @brief test invalid motion plan request with no aux point defined
 */
TEST_F(TrajectoryGeneratorCIRCTest, emptyAux)
{
  auto circ{ tdp_->getCircCartCenterCart("circ1_center_2") };

  planning_interface::MotionPlanRequest req = circ.toRequest();

  req.path_constraints.position_constraints.clear();

  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(planning_scene_, req, res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN);
}

/**
 * @brief test invalid motion plan request with no aux name defined
 */
TEST_F(TrajectoryGeneratorCIRCTest, invalidAuxName)
{
  auto circ{ tdp_->getCircCartCenterCart("circ1_center_2") };

  planning_interface::MotionPlanRequest req = circ.toRequest();

  req.path_constraints.name = "";

  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(planning_scene_, req, res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN);
}

/**
 * @brief test invalid motion plan request with invalid link name in the
 * auxiliary point
 */
TEST_F(TrajectoryGeneratorCIRCTest, invalidAuxLinkName)
{
  auto circ{ tdp_->getCircJointInterimCart("circ3_interim") };

  planning_interface::MotionPlanRequest req = circ.toRequest();

  req.path_constraints.position_constraints.front().link_name = "INVALID";

  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(planning_scene_, req, res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::msg::MoveItErrorCodes::INVALID_LINK_NAME);
}

/**
 * @brief test the circ planner with invalid center point
 */
TEST_F(TrajectoryGeneratorCIRCTest, invalidCenter)
{
  auto circ{ tdp_->getCircCartCenterCart("circ1_center_2") };
  circ.getAuxiliaryConfiguration().getConfiguration().setPose(circ.getStartConfiguration().getPose());
  circ.getAuxiliaryConfiguration().getConfiguration().getPose().position.y += 1;

  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(planning_scene_, circ.toRequest(), res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN);
}

/**
 * @brief test the circ planner with colinear start/goal/center position
 *
 * Expected: Planning should fail since the path is not uniquely defined.
 */
TEST_F(TrajectoryGeneratorCIRCTest, colinearCenter)
{
  auto circ{ tdp_->getCircCartCenterCart("circ1_center_2") };
  circ.getAuxiliaryConfiguration().getConfiguration().setPose(circ.getStartConfiguration().getPose());
  circ.getGoalConfiguration().setPose(circ.getStartConfiguration().getPose());

  // Stretch start and goal pose along line
  circ.getStartConfiguration().getPose().position.x -= 0.1;
  circ.getGoalConfiguration().getPose().position.x += 0.1;

  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(planning_scene_, circ.toRequest(), res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN);
}

/**
 * @brief test the circ planner with  colinear start/goal/interim position
 *
 * Expected: Planning should fail. These positions do not even represent a
 * circle.
 */
TEST_F(TrajectoryGeneratorCIRCTest, colinearInterim)
{
  auto circ{ tdp_->getCircCartInterimCart("circ3_interim") };
  circ.getAuxiliaryConfiguration().getConfiguration().setPose(circ.getStartConfiguration().getPose());
  circ.getGoalConfiguration().setPose(circ.getStartConfiguration().getPose());

  // Stretch start and goal pose along line
  circ.getStartConfiguration().getPose().position.x -= 0.1;
  circ.getGoalConfiguration().getPose().position.x += 0.1;

  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(planning_scene_, circ.toRequest(), res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN);
}

/**
 * @brief test the circ planner with half circle with interim point
 *
 * The request contains start/interim/goal such that
 * start, center (not explicitly given) and goal are colinear
 *
 * Expected: Planning should successfully return.
 */
TEST_F(TrajectoryGeneratorCIRCTest, colinearCenterDueToInterim)
{
  // get the test data from xml
  auto circ{ tdp_->getCircCartInterimCart("circ3_interim") };

  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(circ_->generate(planning_scene_, circ.toRequest(), res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
}

/**
 * @brief test the circ planner with colinear start/center/interim positions
 *
 * The request contains start/interim/goal such that
 * start, center (not explicitly given) and interim are colinear.
 * In case the interim is used as auxiliary point for KDL::Path_Circle this
 * should fail.
 *
 * Expected: Planning should successfully return.
 */
TEST_F(TrajectoryGeneratorCIRCTest, colinearCenterAndInterim)
{
  auto circ{ tdp_->getCircCartInterimCart("circ3_interim") };

  // alter start, interim and goal such that start/center and interim are
  // colinear
  circ.getAuxiliaryConfiguration().getConfiguration().setPose(circ.getStartConfiguration().getPose());
  circ.getGoalConfiguration().setPose(circ.getStartConfiguration().getPose());

  circ.getStartConfiguration().getPose().position.x -= 0.2;
  circ.getAuxiliaryConfiguration().getConfiguration().getPose().position.x += 0.2;
  circ.getGoalConfiguration().getPose().position.y -= 0.2;

  circ.setAccelerationScale(0.05);
  circ.setVelocityScale(0.05);

  moveit_msgs::msg::MotionPlanRequest req = circ.toRequest();

  planning_interface::MotionPlanResponse res;
  EXPECT_TRUE(circ_->generate(planning_scene_, req, res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
  checkCircResult(req, res);
}

/**
 * @brief test the circ planner with a circ path where the angle between goal
 * and interim is larger than 180 degree
 *
 * The request contains start/interim/goal such that 180 degree < interim angle
 * < goal angle.
 *
 * Expected: Planning should successfully return.
 */
TEST_F(TrajectoryGeneratorCIRCTest, interimLarger180Degree)
{
  auto circ{ tdp_->getCircCartInterimCart("circ3_interim") };

  // alter start, interim and goal such that start/center and interim are
  // colinear
  circ.getAuxiliaryConfiguration().getConfiguration().setPose(circ.getStartConfiguration().getPose());
  circ.getGoalConfiguration().setPose(circ.getStartConfiguration().getPose());

  circ.getStartConfiguration().getPose().position.x -= 0.2;
  circ.getAuxiliaryConfiguration().getConfiguration().getPose().position.x += 0.14142136;
  circ.getAuxiliaryConfiguration().getConfiguration().getPose().position.y -= 0.14142136;
  circ.getGoalConfiguration().getPose().position.y -= 0.2;

  circ.setAccelerationScale(0.05);
  circ.setVelocityScale(0.05);

  moveit_msgs::msg::MotionPlanRequest req = circ.toRequest();

  planning_interface::MotionPlanResponse res;
  EXPECT_TRUE(circ_->generate(planning_scene_, req, res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
  checkCircResult(req, res);
}

/**
 * @brief test the circ planner with center point and joint goal
 */
TEST_F(TrajectoryGeneratorCIRCTest, centerPointJointGoal)
{
  auto circ{ tdp_->getCircJointCenterCart("circ1_center_2") };
  moveit_msgs::msg::MotionPlanRequest req = circ.toRequest();

  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(circ_->generate(planning_scene_, req, res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
  checkCircResult(req, res);
}

/**
 * @brief A valid circ request contains a helping point (interim or center), in
 * this test a additional
 * point is defined as an invalid test case
 */
TEST_F(TrajectoryGeneratorCIRCTest, InvalidAdditionalPrimitivePose)
{
  auto circ{ tdp_->getCircCartCenterCart("circ1_center_2") };

  moveit_msgs::msg::MotionPlanRequest req = circ.toRequest();

  // Contains one pose (interim / center)
  ASSERT_EQ(req.path_constraints.position_constraints.back().constraint_region.primitive_poses.size(), 1u);

  // Define a additional pose here
  geometry_msgs::msg::Pose center_position;
  center_position.position.x = 0.0;
  center_position.position.y = 0.0;
  center_position.position.z = 0.65;
  req.path_constraints.position_constraints.back().constraint_region.primitive_poses.push_back(center_position);

  planning_interface::MotionPlanResponse res;
  ASSERT_FALSE(circ_->generate(planning_scene_, req, res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN);
}

/**
 * @brief Joint Goals are expected to match the start state in number and
 * joint_names
 * Here an additional joint constraints is "falsely" defined to check for the
 * error.
 */
TEST_F(TrajectoryGeneratorCIRCTest, InvalidExtraJointConstraint)
{
  auto circ{ tdp_->getCircJointCenterCart("circ1_center_2") };

  moveit_msgs::msg::MotionPlanRequest req = circ.toRequest();

  // Define the additional joint constraint
  moveit_msgs::msg::JointConstraint joint_constraint;
  joint_constraint.joint_name = req.goal_constraints.front().joint_constraints.front().joint_name;
  req.goal_constraints.front().joint_constraints.push_back(joint_constraint);  //<-- Additional constraint

  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(planning_scene_, req, res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
}

/**
 * @brief test the circ planner with center point and pose goal
 */
TEST_F(TrajectoryGeneratorCIRCTest, CenterPointPoseGoal)
{
  auto circ{ tdp_->getCircCartCenterCart("circ1_center_2") };

  moveit_msgs::msg::MotionPlanRequest req = circ.toRequest();

  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(circ_->generate(planning_scene_, req, res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
  checkCircResult(req, res);
}

/**
 * @brief Set a frame id only on the position constrainst
 */
TEST_F(TrajectoryGeneratorCIRCTest, CenterPointPoseGoalFrameIdPositionConstraints)
{
  auto circ{ tdp_->getCircCartCenterCart("circ1_center_2") };

  moveit_msgs::msg::MotionPlanRequest req = circ.toRequest();

  req.goal_constraints.front().position_constraints.front().header.frame_id = robot_model_->getModelFrame();

  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(circ_->generate(planning_scene_, req, res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
  checkCircResult(req, res);
}

/**
 * @brief Set a frame id only on the orientation constrainst
 */
TEST_F(TrajectoryGeneratorCIRCTest, CenterPointPoseGoalFrameIdOrientationConstraints)
{
  auto circ{ tdp_->getCircCartCenterCart("circ1_center_2") };

  moveit_msgs::msg::MotionPlanRequest req = circ.toRequest();
  req.goal_constraints.front().orientation_constraints.front().header.frame_id = robot_model_->getModelFrame();

  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(circ_->generate(planning_scene_, req, res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
  checkCircResult(req, res);
}

/**
 * @brief Set a frame_id on both position and orientation constraints
 */
TEST_F(TrajectoryGeneratorCIRCTest, CenterPointPoseGoalFrameIdBothConstraints)
{
  auto circ{ tdp_->getCircCartCenterCart("circ1_center_2") };

  moveit_msgs::msg::MotionPlanRequest req = circ.toRequest();

  // Both set
  req.goal_constraints.front().position_constraints.front().header.frame_id = robot_model_->getModelFrame();
  req.goal_constraints.front().orientation_constraints.front().header.frame_id = robot_model_->getModelFrame();

  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(circ_->generate(planning_scene_, req, res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
  checkCircResult(req, res);
}

/**
 * @brief test the circ planner with interim point with joint goal
 */
TEST_F(TrajectoryGeneratorCIRCTest, InterimPointJointGoal)
{
  auto circ{ tdp_->getCircJointInterimCart("circ3_interim") };

  moveit_msgs::msg::MotionPlanRequest req = circ.toRequest();

  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(circ_->generate(planning_scene_, req, res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
  checkCircResult(req, res);
}

/**
 * @brief test the circ planner with interim point with joint goal and a close
 * to zero velocity of the start state
 *
 * The generator is expected to be robust against a velocity being almost zero.
 */
TEST_F(TrajectoryGeneratorCIRCTest, InterimPointJointGoalStartVelNearZero)
{
  auto circ{ tdp_->getCircJointInterimCart("circ3_interim") };

  moveit_msgs::msg::MotionPlanRequest req = circ.toRequest();

  // Set velocity near zero
  req.start_state.joint_state.velocity = std::vector<double>(req.start_state.joint_state.position.size(), 1e-16);

  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(circ_->generate(planning_scene_, req, res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
  checkCircResult(req, res);
}

/**
 * @brief test the circ planner with interim point with pose goal
 */
TEST_F(TrajectoryGeneratorCIRCTest, InterimPointPoseGoal)
{
  auto circ{ tdp_->getCircJointInterimCart("circ3_interim") };
  moveit_msgs::msg::MotionPlanRequest req = circ.toRequest();

  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(circ_->generate(planning_scene_, req, res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
  checkCircResult(req, res);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
