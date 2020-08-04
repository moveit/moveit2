/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

/*      Title     : TODO(adamp)
 *      Project   : moveit_servo
 *      Created   : 07/24/2020
 *      Author    : Adam Pettinger
 *      Desc      : Loads a Servo instance without launching or parameters
 */

#include <gtest/gtest.h>
#include <moveit_servo/servo.h>

#include <fstream>
#include <sstream>
#include <string>
#include <boost/filesystem/path.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "test_parameter_struct.hpp"
#include "unit_test_servo_calcs.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.unit_test_servo_calcs.cpp");

void loadModelFile(std::string filename, std::string& file_content)
{
  boost::filesystem::path res_path(ament_index_cpp::get_package_share_directory("moveit_resources"));
  std::string xml_string;
  std::fstream xml_file((res_path / filename).string().c_str(), std::fstream::in);
  // EXPECT_TRUE(xml_file.is_open());
  EXPECT_TRUE(5==5);
  while (xml_file.good())
  {
    std::string line;
    std::getline(xml_file, line);
    xml_string += (line + "\n");
  }
  xml_file.close();
  file_content = xml_string;
}

FriendServoCalcs::FriendServoCalcs(const rclcpp::Node::SharedPtr& node, const moveit_servo::ServoParametersPtr& parameters,
             const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
: ServoCalcs(node, parameters, planning_scene_monitor) {}

ServoCalcsTestFixture::ServoCalcsTestFixture()
: node_(std::make_shared<rclcpp::Node>("servo_calcs_test"))
{
  // "Load" parameters from yaml as node parameters
  std::string robot_description_string, srdf_string;
  loadModelFile("panda_description/urdf/panda.urdf", robot_description_string);
  loadModelFile("panda_moveit_config/config/panda.srdf", srdf_string);
  node_->declare_parameter<std::string>("robot_description", robot_description_string);
  node_->declare_parameter<std::string>("robot_description_semantic", srdf_string);

  // Startup planning_scene_monitor
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, "robot_description", tf_buffer_, "planning_scene_monitor");

  // Get moveit parameters
  moveit_servo::ServoParametersPtr test_params = getTestParameters();

  // Set up the servo_calcs object
  servo_calcs_ = std::make_unique<FriendServoCalcs>(node_, test_params, psm_);
}

void ServoCalcsTestFixture::SetUp()
{
  // do stuff
}

void ServoCalcsTestFixture::TearDown()
{
  // Do other stuff
}

sensor_msgs::msg::JointState ServoCalcsTestFixture::getJointState(std::vector<double> pos, std::vector<double> vel)
{
  sensor_msgs::msg::JointState msg;
  std::vector<double> effort;
  effort.assign(9,0);

  msg.name = panda_joint_names_;
  msg.position = pos;
  msg.velocity = vel;
  msg.effort = effort;
  msg.header.stamp = node_->now();

  return msg;
}

TEST_F(ServoCalcsTestFixture, TestRemoveSingleDimension)
{
  int rows = 6;
  int cols = 7;

  // Create a matrix and vector, set one value so we can track the removal was correct
  Eigen::MatrixXd matrix(rows, cols);
  matrix.setZero();
  matrix(2,0) = 4.0;
  Eigen::VectorXd vector(rows);
  vector.setZero();
  vector(2) = 4.0;

  // Remove the second row
  servo_calcs_->removeDimension(matrix, vector, 1);

  // Size should be reduced, and the set values should have moved up
  EXPECT_EQ(matrix.rows(), rows-1);
  EXPECT_EQ(vector.size(), rows-1);
  EXPECT_EQ(matrix(1,0), 4.0);
  EXPECT_EQ(vector(1), 4.0);

  // Remove the last row
  servo_calcs_->removeDimension(matrix, vector, 4);

  // Size should be reduced, but the set values should not move
  EXPECT_EQ(matrix.rows(), rows-2);
  EXPECT_EQ(vector.size(), rows-2);
  EXPECT_EQ(matrix(1,0), 4.0);
  EXPECT_EQ(vector(1), 4.0);

  // Sanity check that the columns stayed the same
  EXPECT_EQ(matrix.cols(), cols);
}

TEST_F(ServoCalcsTestFixture, TestRemoveDriftDimensions)
{
  int rows = 6;
  int cols = 7;

  // Create a matrix and vector, set one value so we can track the removal was correct
  Eigen::MatrixXd matrix(rows, cols);
  matrix.setZero();
  matrix(2,0) = 4.0;
  Eigen::VectorXd vector(rows);
  vector.setZero();
  vector(2) = 4.0;

  // Remove with default drift_dimensions_ values should not change
  servo_calcs_->removeDriftDimensions(matrix, vector);
  EXPECT_EQ(matrix.rows(), rows);
  EXPECT_EQ(matrix.cols(), cols);
  EXPECT_EQ(vector.size(), rows);
  EXPECT_EQ(matrix(2,0), 4.0);
  EXPECT_EQ(vector(2), 4.0);

  // Set drift_dimensions_ to be something with True's in it
  servo_calcs_->drift_dimensions_[0] = true;
  servo_calcs_->drift_dimensions_[1] = true;
  servo_calcs_->drift_dimensions_[5] = true;

  // Now a remove should have changes
  servo_calcs_->removeDriftDimensions(matrix, vector);
  EXPECT_EQ(matrix.rows(), rows-3);
  EXPECT_EQ(matrix.cols(), cols);
  EXPECT_EQ(vector.size(), rows-3);
  EXPECT_EQ(matrix(0,0), 4.0);
  EXPECT_EQ(vector(0), 4.0);
}

TEST_F(ServoCalcsTestFixture, TestEnforceControlDimensions)
{
  // Create a TwistStamped message to test with
  geometry_msgs::msg::TwistStamped msg;
  msg.twist.linear.x = 1.0;
  msg.twist.linear.y = 2.0;
  msg.twist.linear.z = 3.0;
  msg.twist.angular.x = 4.0;
  msg.twist.angular.y = 5.0;
  msg.twist.angular.z = 6.0;
  geometry_msgs::msg::TwistStamped init_copy = msg;

  // Enforcing with the default values should not change anything
  servo_calcs_->enforceControlDimensions(msg);
  EXPECT_EQ(msg, init_copy);

  // Lets set the whole array to false and make sure each value is changed to 0
  for (size_t i=0; i<6; ++i)
  {
    servo_calcs_->control_dimensions_[i] = false;
  }
  geometry_msgs::msg::TwistStamped empty_msg;
  servo_calcs_->enforceControlDimensions(msg);
  EXPECT_EQ(msg, empty_msg);
}

TEST_F(ServoCalcsTestFixture, TestCheckValidCommand)
{
  // Create a valid JointJog message and test it
  control_msgs::msg::JointJog joint_msg;
  joint_msg.velocities.push_back(1.0);
  EXPECT_TRUE(servo_calcs_->checkValidCommand(joint_msg));

  // Now give it a NaN and check again
  joint_msg.velocities.push_back(NAN);
  EXPECT_FALSE(servo_calcs_->checkValidCommand(joint_msg));

  // Create a valid TwistStamped message and test it
  geometry_msgs::msg::TwistStamped twist_msg;
  twist_msg.twist.linear.x = 1.0;
  EXPECT_TRUE(servo_calcs_->checkValidCommand(twist_msg));

  // Now give it a NaN and check again
  twist_msg.twist.linear.y = NAN;
  EXPECT_FALSE(servo_calcs_->checkValidCommand(twist_msg));
}

TEST_F(ServoCalcsTestFixture, TestApplyJointUpdate)
{
  // Create Eigen arrays for testing
  Eigen::ArrayXd deltas(3);
  deltas[0] = 1.0;
  deltas[1] = 2.0;
  deltas[2] = 3.0;
  Eigen::ArrayXd prev_vel(3);
  prev_vel.setZero();

  // Test catching size mismatches
  sensor_msgs::msg::JointState joint_state;
  EXPECT_FALSE(servo_calcs_->applyJointUpdate(deltas, joint_state, prev_vel));

  // Now test it with properly sized arrays
  joint_state.position.assign(3, 10.0);
  joint_state.velocity.assign(3, 0);
  EXPECT_TRUE(servo_calcs_->applyJointUpdate(deltas, joint_state, prev_vel));

  // Can't test equality on the position because of filtering
  EXPECT_NE(joint_state.position[0], 10);
  EXPECT_NE(joint_state.position[2], 10);

  // Velocities should match though
  EXPECT_EQ(joint_state.velocity[0], deltas[0]/servo_calcs_->parameters_->publish_period);
  EXPECT_EQ(joint_state.velocity[2], deltas[2]/servo_calcs_->parameters_->publish_period);
  EXPECT_EQ(joint_state.velocity[0], prev_vel[0]);
}

TEST_F(ServoCalcsTestFixture, TestInsertRedundantPoints)
{
  // Create a joint trajectory message for testing
  trajectory_msgs::msg::JointTrajectory joint_traj;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions.push_back(1.0);
  point.positions.push_back(2.0);
  point.velocities.push_back(3.0);
  point.velocities.push_back(4.0);
  joint_traj.points.push_back(point);

  auto init_copy = joint_traj;

  // Test for count 0 and 1, this should leave the message unchanged
  servo_calcs_->insertRedundantPointsIntoTrajectory(joint_traj, 0);
  EXPECT_EQ(joint_traj, init_copy);
  servo_calcs_->insertRedundantPointsIntoTrajectory(joint_traj, 1);
  EXPECT_EQ(joint_traj, init_copy);

  // Test for a more reasonable count
  size_t count = 10;
  servo_calcs_->insertRedundantPointsIntoTrajectory(joint_traj, count);
  EXPECT_NE(joint_traj, init_copy);
  EXPECT_EQ(joint_traj.points.size(), count);
  EXPECT_EQ(joint_traj.points[0].positions, joint_traj.points.back().positions);
}

TEST_F(ServoCalcsTestFixture, TestSuddenHalt)
{
  // We need to set up some ServoCalcs members
  servo_calcs_->num_joints_ = 3;
  servo_calcs_->original_joint_state_.position.push_back(1.0);
  servo_calcs_->original_joint_state_.position.push_back(2.0);
  servo_calcs_->original_joint_state_.position.push_back(3.0);

  // Let's make sure to test publishing position and velocity
  servo_calcs_->parameters_->publish_joint_positions = true;
  servo_calcs_->parameters_->publish_joint_velocities = true;

  // Let's start with an empty trajectory
  trajectory_msgs::msg::JointTrajectory msg;
  servo_calcs_->suddenHalt(msg);
  EXPECT_TRUE(msg.points.size() == 1);
  EXPECT_TRUE(msg.points[0].positions.size() == 3);
  EXPECT_TRUE(msg.points[0].velocities.size() == 3);
  EXPECT_EQ(msg.points[0].positions[2], 3.0);
  EXPECT_EQ(msg.points[0].velocities[2], 0.0);

  // Now if we change those values and call it again, they should go back
  msg.points[0].positions[2] = 100.0;
  msg.points[0].velocities[2] = 100.0;
  servo_calcs_->suddenHalt(msg);
  EXPECT_EQ(msg.points[0].positions[2], 3.0);
  EXPECT_EQ(msg.points[0].velocities[2], 0.0);
}

TEST_F(ServoCalcsTestFixture, TestEnforcePosLimits)
{
  // Set the position to the upper limits
  std::vector<double> position{0,0,2.8973,1.7628,2.8973,0.0175,2.8973,3.7525,2.8973};
  std::vector<double> velocity;
  velocity.assign(9,1.0);

  // Set the position in the ServoCalcs object
  sensor_msgs::msg::JointState joint_state = getJointState(position, velocity);
  servo_calcs_->original_joint_state_ = joint_state;
  servo_calcs_->kinematic_state_->setVariableValues(joint_state);

  // Test here, expecting to be violating joint position limits
  EXPECT_FALSE(servo_calcs_->enforceSRDFPositionLimits());

  // At the upper limits with negative velocity, we should not be 'violating'
  velocity.assign(9,-1.0);
  joint_state = getJointState(position, velocity);
  servo_calcs_->original_joint_state_ = joint_state;
  servo_calcs_->kinematic_state_->setVariableValues(joint_state);
  EXPECT_TRUE(servo_calcs_->enforceSRDFPositionLimits());

  // However, if we change 1 of the joints to the bottom limit and stay negative velocity
  // We expect to violate joint position limits again
  position[2] = -2.8973;
  joint_state = getJointState(position, velocity);
  servo_calcs_->original_joint_state_ = joint_state;
  servo_calcs_->kinematic_state_->setVariableValues(joint_state);
  EXPECT_FALSE(servo_calcs_->enforceSRDFPositionLimits());

  // For completeness, we'll set the position to lower limits with positive vel and expect a pass
  std::vector<double> lower_position{0,0,-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973};
  velocity.assign(9,1.0);
  joint_state = getJointState(lower_position, velocity);
  servo_calcs_->original_joint_state_ = joint_state;
  servo_calcs_->kinematic_state_->setVariableValues(joint_state);
  EXPECT_TRUE(servo_calcs_->enforceSRDFPositionLimits());
}

TEST_F(ServoCalcsTestFixture, TestEnforceAccelVelLimits)
{
  // First, define the velocity limits (from panda URDF)
  std::vector<double> vel_limits{2.3925,2.3925,2.3925,2.3925,2.8710,2.8710,2.8710};

  // Lets test the Velocity limits first
  // Set prev_joint_velocity_ == desired_velocity, both above the limits
  // to avoid acceleration limits (accel is 0)
  Eigen::ArrayXd desired_velocity(7);
  desired_velocity << 3, 3, 3, 3, 3, 3, 3; // rad/s
  desired_velocity *= servo_calcs_->parameters_->publish_period; // rad/loop
  servo_calcs_->prev_joint_velocity_ = desired_velocity;

  // Do the enforcing and check it
  servo_calcs_->enforceSRDFAccelVelLimits(desired_velocity);
  for (size_t i=0; i<7; ++i)
  {
    // We need to check vs radians-per-loop allowable rate (not rad/s)
    EXPECT_LE(desired_velocity[i], vel_limits[i]*servo_calcs_->parameters_->publish_period);
  }

  // Let's check the negative velocities too
  desired_velocity *= -1;
  servo_calcs_->prev_joint_velocity_ = desired_velocity;
  servo_calcs_->enforceSRDFAccelVelLimits(desired_velocity);
  for (size_t i=0; i<7; ++i)
  {
    // We need to check vs radians-per-loop allowable rate (not rad/s)
    EXPECT_GE(desired_velocity[i], -1*vel_limits[i]*servo_calcs_->parameters_->publish_period);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
