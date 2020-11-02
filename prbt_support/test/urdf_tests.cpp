/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ros/ros.h"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#define _USE_MATH_DEFINES
#include <math.h>

const std::string ARM_GROUP_NAME{ "arm_group_name" };
const std::string ARM_GROUP_TIP_LINK_NAME{ "arm_tip_link" };

static constexpr double L0{ 0.2604 };  // Height of foot
static constexpr double L1{ 0.3500 };  // Height of first connector
static constexpr double L2{ 0.3070 };  // Height of second connector
static constexpr double L3{ 0.0840 };  // Distance last joint to flange
static constexpr double DELTA{ 1.0e-10 };

const std::string PARAM_MODEL{ "robot_description" };

typedef std::tuple<std::string, std::vector<double>, Eigen::Vector3d> TestDataPoint;

inline std::string getName(TestDataPoint& tdp)
{
  return std::get<0>(tdp);
}

inline std::vector<double> getJoints(TestDataPoint& tdp)
{
  return std::get<1>(tdp);
}

inline Eigen::Vector3d getPos(TestDataPoint& tdp)
{
  return std::get<2>(tdp);
}

std::string vecToString(std::vector<double>& vec)
{
  return std::string(vec.begin(), vec.end());
}

/**
 * @brief test fixture
 */
class URDFKinematicsTest : public testing::TestWithParam<std::string>
{
protected:
  void SetUp() override;

protected:
  // ros stuff
  ros::NodeHandle ph_{ "~" };
  // The following call to RobotModelLoader will throw an error message about not not finding srdf,
  // but it works just fine with just urdf.
  robot_model::RobotModelConstPtr robot_model_{ robot_model_loader::RobotModelLoader(GetParam(), false).getModel() };

  // parameters
  std::string tip_link_name_;

  // test data
  std::vector<TestDataPoint> testDataSet_;
};

void URDFKinematicsTest::SetUp()
{
  // get necessary parameters
  ASSERT_TRUE(ph_.getParam(ARM_GROUP_TIP_LINK_NAME, tip_link_name_));

  // Fill the testdata
  testDataSet_.push_back(TestDataPoint("HomePos ", { 0, 0, 0, 0, 0, 0 }, Eigen::Vector3d(0, 0, L0 + L1 + L2 + L3)));
  testDataSet_.push_back(TestDataPoint("TestPos1", { 0, M_PI_2, 0, 0, 0, 0 }, Eigen::Vector3d(L1 + L2 + L3, 0, L0)));
  testDataSet_.push_back(
      TestDataPoint("TestPos2", { 0, -M_PI_2, M_PI_2, 0, 0, 0 }, Eigen::Vector3d(-L1, 0, L0 - L2 - L3)));
  testDataSet_.push_back(
      TestDataPoint("TestPos3", { 0, -M_PI_2, -M_PI_2, 0, 0, 0 }, Eigen::Vector3d(-L1, 0, L0 + L2 + L3)));
  testDataSet_.push_back(TestDataPoint("TestPos4", { 0, 0, M_PI_2, 0, 0, 0 }, Eigen::Vector3d(-L2 - L3, 0, L0 + L1)));
  testDataSet_.push_back(
      TestDataPoint("TestPos5", { -M_PI_2, 0, -M_PI_2, 0, 0, 0 }, Eigen::Vector3d(0, -L2 - L3, L0 + L1)));
  testDataSet_.push_back(
      TestDataPoint("TestPos6", { -M_PI_2, -M_PI_2, -M_PI_2, 0, 0, 0 }, Eigen::Vector3d(0, L1, L0 + L2 + L3)));
  testDataSet_.push_back(
      TestDataPoint("TestPos7", { M_PI_2, -M_PI_2, 0, 0, 0, 0 }, Eigen::Vector3d(0, -L1 - L2 - L3, L0)));
  testDataSet_.push_back(TestDataPoint("TestPos8", { 0, 0, 0, 0, -M_PI_2, 0 }, Eigen::Vector3d(L3, 0, L0 + L1 + L2)));
  testDataSet_.push_back(
      TestDataPoint("TestPos9", { M_PI_2, 0, 0, 0, -M_PI_2, 0 }, Eigen::Vector3d(0, L3, L0 + L1 + L2)));
  testDataSet_.push_back(
      TestDataPoint("TestPos10", { 0, 0, 0, 0, 0, M_PI_2 }, Eigen::Vector3d(0, 0, L0 + L1 + L2 + L3)));
}

INSTANTIATE_TEST_CASE_P(InstantiationName, URDFKinematicsTest, ::testing::Values(PARAM_MODEL), );
/**
 * \brief test the kinematics of urdf model
 *
 * Assures that the forward kinematics behave as expected.
 * This is done by looping over multiple test data points and checking
 * the tcp position (not the pose!).
 */
TEST_P(URDFKinematicsTest, forwardKinematics)
{
  // check if the transformation is available
  robot_state::RobotState rstate(robot_model_);
  ASSERT_TRUE(rstate.knowsFrameTransform(tip_link_name_));

  // Loop over the testdata
  for (TestDataPoint& test_data : testDataSet_)
  {
    std::string name = getName(test_data);
    std::vector<double> joints = getJoints(test_data);
    Eigen::Vector3d pos_exp = getPos(test_data);

    // get frame transform
    rstate.setVariablePositions(joints);
    rstate.update();
    Eigen::Affine3d transform = rstate.getFrameTransform(tip_link_name_);

    // Test the position
    double error_norm = (transform.translation() - pos_exp).norm();
    EXPECT_NEAR(error_norm, 0, DELTA) << "TestPosition \"" << name << "\" failed\n"
                                      << "\t Joints: ["
                                      << Eigen::VectorXd::Map(joints.data(), static_cast<long>(joints.size()))
                                             .transpose()
                                      << "]\n"
                                      << "\t Expected position [" << pos_exp.transpose() << "]\n"
                                      << "\t Real position [" << transform.translation().transpose() << "]";
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "urdf_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
