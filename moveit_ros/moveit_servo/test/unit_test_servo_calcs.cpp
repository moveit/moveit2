/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
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

/*      Title     : unit_test_servo_calcs.cpp
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
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "unit_test_servo_calcs.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.unit_test_servo_calcs.cpp");

void loadModelFile(std::string package_name, std::string filename, std::string& file_content)
{
  std::filesystem::path res_path(ament_index_cpp::get_package_share_directory(package_name));
  std::string xml_string;
  std::fstream xml_file((res_path / filename).string().c_str(), std::fstream::in);
  while (xml_file.good())
  {
    std::string line;
    std::getline(xml_file, line);
    xml_string += (line + "\n");
  }
  xml_file.close();
  file_content = xml_string;
}

FriendServoCalcs::FriendServoCalcs(const rclcpp::Node::SharedPtr& node,
                                   const std::shared_ptr<const moveit_servo::ServoParameters>& parameters,
                                   const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
  : ServoCalcs(node, parameters, planning_scene_monitor)
{
}

ServoCalcsTestFixture::ServoCalcsTestFixture() : node_(TEST_NODE)
{
  servo_calcs_ = std::make_unique<FriendServoCalcs>(node_, TEST_PARAMS, TEST_PSM);
  servo_calcs_->start();
}

sensor_msgs::msg::JointState ServoCalcsTestFixture::getJointState(std::vector<double> pos, std::vector<double> vel)
{
  sensor_msgs::msg::JointState msg;
  std::vector<double> effort;
  effort.assign(9, 0);

  msg.name = PANDA_JOINT_NAMES;
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
  matrix(2, 0) = 4.0;
  Eigen::VectorXd vector(rows);
  vector.setZero();
  vector(2) = 4.0;

  // Remove the second row
  servo_calcs_->removeDimension(matrix, vector, 1);

  // Size should be reduced, and the set values should have moved up
  EXPECT_EQ(matrix.rows(), rows - 1);
  EXPECT_EQ(vector.size(), rows - 1);
  EXPECT_EQ(matrix(1, 0), 4.0);
  EXPECT_EQ(vector(1), 4.0);

  // Remove the last row
  servo_calcs_->removeDimension(matrix, vector, 4);

  // Size should be reduced, but the set values should not move
  EXPECT_EQ(matrix.rows(), rows - 2);
  EXPECT_EQ(vector.size(), rows - 2);
  EXPECT_EQ(matrix(1, 0), 4.0);
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
  matrix(2, 0) = 4.0;
  Eigen::VectorXd vector(rows);
  vector.setZero();
  vector(2) = 4.0;

  // Remove with default drift_dimensions_ values should not change
  servo_calcs_->removeDriftDimensions(matrix, vector);
  EXPECT_EQ(matrix.rows(), rows);
  EXPECT_EQ(matrix.cols(), cols);
  EXPECT_EQ(vector.size(), rows);
  EXPECT_EQ(matrix(2, 0), 4.0);
  EXPECT_EQ(vector(2), 4.0);

  // Set drift_dimensions_ to be something with True's in it
  servo_calcs_->drift_dimensions_[0] = true;
  servo_calcs_->drift_dimensions_[1] = true;
  servo_calcs_->drift_dimensions_[5] = true;

  // Now a remove should have changes
  servo_calcs_->removeDriftDimensions(matrix, vector);
  EXPECT_EQ(matrix.rows(), rows - 3);
  EXPECT_EQ(matrix.cols(), cols);
  EXPECT_EQ(vector.size(), rows - 3);
  EXPECT_EQ(matrix(0, 0), 4.0);
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
  for (size_t i = 0; i < 6; ++i)
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

  // Now set the scaling to unitless and give it a number with abs() > 1, expecting a fail
  const_cast<moveit_servo::ServoParameters*>(servo_calcs_->parameters_.get())->command_in_type = "speed_units";
  twist_msg.twist.linear.y = -10.0;
  EXPECT_TRUE(servo_calcs_->checkValidCommand(twist_msg));
  const_cast<moveit_servo::ServoParameters*>(servo_calcs_->parameters_.get())->command_in_type = "unitless";
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
  EXPECT_EQ(joint_state.velocity[0], deltas[0] / servo_calcs_->parameters_->publish_period);
  EXPECT_EQ(joint_state.velocity[2], deltas[2] / servo_calcs_->parameters_->publish_period);
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
  const_cast<moveit_servo::ServoParameters*>(servo_calcs_->parameters_.get())->publish_joint_positions = true;
  const_cast<moveit_servo::ServoParameters*>(servo_calcs_->parameters_.get())->publish_joint_velocities = true;

  // Let's start with an empty trajectory
  trajectory_msgs::msg::JointTrajectory msg;
  servo_calcs_->suddenHalt(msg);
  EXPECT_EQ(msg.points.size(), 1UL);
  EXPECT_EQ(msg.points[0].positions.size(), 3UL);
  EXPECT_EQ(msg.points[0].velocities.size(), 3UL);
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
  std::vector<double> position{ 0, 0, 2.8973, 1.9628, 2.8973, 0.0175, 2.8973, 3.7525, 2.8973 };
  std::vector<double> velocity;
  velocity.assign(9, 1.0);

  // Set the position in the ServoCalcs object
  sensor_msgs::msg::JointState joint_state = getJointState(position, velocity);
  servo_calcs_->original_joint_state_ = joint_state;
  servo_calcs_->current_state_->setVariableValues(joint_state);

  // Test here, expecting to be violating joint position limits
  EXPECT_FALSE(servo_calcs_->enforcePositionLimits());

  // At the upper limits with negative velocity, we should not be 'violating'
  velocity.assign(9, -1.0);
  joint_state = getJointState(position, velocity);
  servo_calcs_->original_joint_state_ = joint_state;
  servo_calcs_->current_state_->setVariableValues(joint_state);
  EXPECT_TRUE(servo_calcs_->enforcePositionLimits());

  // However, if we change 1 of the joints to the bottom limit and stay negative velocity
  // We expect to violate joint position limits again
  position[2] = -2.8973;
  joint_state = getJointState(position, velocity);
  servo_calcs_->original_joint_state_ = joint_state;
  servo_calcs_->current_state_->setVariableValues(joint_state);
  EXPECT_FALSE(servo_calcs_->enforcePositionLimits());

  // For completeness, we'll set the position to lower limits with positive vel and expect a pass
  std::vector<double> lower_position{ 0, 0, -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973 };
  velocity.assign(9, 1.0);
  joint_state = getJointState(lower_position, velocity);
  servo_calcs_->original_joint_state_ = joint_state;
  servo_calcs_->current_state_->setVariableValues(joint_state);
  EXPECT_TRUE(servo_calcs_->enforcePositionLimits());
}

TEST_F(ServoCalcsTestFixture, TestEnforceVelLimits)
{
  // First, define the velocity limits (from panda URDF)
  std::vector<double> vel_limits{ 2.1750, 2.1750, 2.1750, 2.1750, 2.610, 2.610, 2.610 };

  // Let's test the Velocity limits first
  // Set prev_joint_velocity_ == desired_velocity, both above the limits
  // to avoid acceleration limits (accel is 0)
  Eigen::ArrayXd desired_velocity(7);
  desired_velocity << 3, 3, 3, 3, 3, 3, 3;                        // rad/s
  desired_velocity *= servo_calcs_->parameters_->publish_period;  // rad/loop
  servo_calcs_->prev_joint_velocity_ = desired_velocity;

  Eigen::ArrayXd delta_theta = servo_calcs_->parameters_->publish_period * desired_velocity;

  // Do the enforcing and check it
  servo_calcs_->enforceVelLimits(delta_theta);
  for (size_t i = 0; i < 7; ++i)
  {
    // We need to check vs radians-per-loop allowable rate (not rad/s)
    EXPECT_LE(delta_theta[i], vel_limits[i] * servo_calcs_->parameters_->publish_period);
  }

  // Let's check negative velocity limits too
  delta_theta *= -1;
  servo_calcs_->prev_joint_velocity_ = -1 * desired_velocity;
  servo_calcs_->enforceVelLimits(delta_theta);
  for (size_t i = 0; i < 7; ++i)
  {
    // We need to check vs radians-per-loop allowable rate (not rad/s)
    EXPECT_GE(delta_theta[i], -1 * vel_limits[i] * servo_calcs_->parameters_->publish_period);
  }
}

// TODO(henningkayser): re-enable acceleration limit enforcement
// The accel/vel limit enforcement has been refactored, enforceSingleVelAccelLimit() was removed
// since all values need to be scaled by the lowest bound
// TEST_F(ServoCalcsTestFixture, TestEnforceAccelLimits)
// {
//   // The panda URDF defines no accel limits
//   // So we get the bound from joint_model_group_ and modify it
//   auto joint_model = servo_calcs_->joint_model_group_->getActiveJointModels()[3];
//   auto bounds = joint_model->getVariableBounds(joint_model->getName());
//   bounds.acceleration_bounded_ = true;
//   bounds.min_acceleration_ = -3;
//   bounds.max_acceleration_ = 3;
//
//   // Pick previous_velocity and desired_velocity to violate limits
//   double previous_velocity = -2;  // rad/s & within velocity limits
//   double desired_velocity = 2;    // rad/s & within velocity limits
//
//   // From those, calculate desired delta_theta and current acceleration
//   double delta_theta = desired_velocity * servo_calcs_->parameters_->publish_period;                         // rad
//   double acceleration = (desired_velocity - previous_velocity) / servo_calcs_->parameters_->publish_period;  // rad/s^2
//
//   // Enforce the bounds
//   double init_delta_theta = delta_theta;
//   servo_calcs_->enforceSingleVelAccelLimit(bounds, desired_velocity, previous_velocity, acceleration, delta_theta);
//
//   // The delta_theta should have dropped to be within the limit
//   EXPECT_LT(delta_theta, init_delta_theta);
//
//   // In fact we can calculate the maximum delta_theta at the limit as:
//   // delta_limit = delta_t * (accel_lim * delta_t _ + vel_prev)
//   double delta_at_limit = servo_calcs_->parameters_->publish_period *
//                           (previous_velocity + bounds.max_acceleration_ * servo_calcs_->parameters_->publish_period);
//   EXPECT_EQ(delta_theta, delta_at_limit);
//
//   // Let's test again, but with only a small change in velocity
//   desired_velocity = -1.9;
//   delta_theta = desired_velocity * servo_calcs_->parameters_->publish_period;                         // rad
//   acceleration = (desired_velocity - previous_velocity) / servo_calcs_->parameters_->publish_period;  // rad/s^2
//   init_delta_theta = delta_theta;
//   servo_calcs_->enforceSingleVelAccelLimit(bounds, desired_velocity, previous_velocity, acceleration, delta_theta);
//
//   // Now, the delta_theta should not have changed
//   EXPECT_EQ(delta_theta, init_delta_theta);
// }

TEST_F(ServoCalcsTestFixture, TestScaleCartesianCommand)
{
  // Create a twist msg to test
  geometry_msgs::msg::TwistStamped msg;
  msg.twist.linear.x = 2.0;
  msg.twist.angular.z = 6.0;

  // Lets test an invalid scaling type first
  const_cast<moveit_servo::ServoParameters*>(servo_calcs_->parameters_.get())->command_in_type = "invalid_string";
  Eigen::VectorXd result = servo_calcs_->scaleCartesianCommand(msg);
  EXPECT_TRUE(result.isZero());

  // Now let's try with unitless
  const_cast<moveit_servo::ServoParameters*>(servo_calcs_->parameters_.get())->command_in_type = "unitless";
  result = servo_calcs_->scaleCartesianCommand(msg);
  EXPECT_NEAR(result[0],
              msg.twist.linear.x * servo_calcs_->parameters_->linear_scale * servo_calcs_->parameters_->publish_period,
              0.001);
  EXPECT_NEAR(result[5],
              msg.twist.angular.z * servo_calcs_->parameters_->rotational_scale *
                  servo_calcs_->parameters_->publish_period,
              0.001);

  // And finally with speed_units
  const_cast<moveit_servo::ServoParameters*>(servo_calcs_->parameters_.get())->command_in_type = "speed_units";
  result = servo_calcs_->scaleCartesianCommand(msg);
  EXPECT_NEAR(result[0], msg.twist.linear.x * servo_calcs_->parameters_->publish_period, 0.001);
  EXPECT_NEAR(result[5], msg.twist.angular.z * servo_calcs_->parameters_->publish_period, 0.001);
}

TEST_F(ServoCalcsTestFixture, TestScaleJointCommand)
{
  // Get a JointJog msg to test
  control_msgs::msg::JointJog msg;
  msg.joint_names = PANDA_JOINT_NAMES;
  std::vector<double> vel{ 0, 0, 1, 1, 1, 1, 1, 1, 1 };
  msg.velocities = vel;

  // Test with unitless
  const_cast<moveit_servo::ServoParameters*>(servo_calcs_->parameters_.get())->command_in_type = "unitless";
  Eigen::VectorXd result = servo_calcs_->scaleJointCommand(msg);
  EXPECT_EQ(result[0], servo_calcs_->parameters_->joint_scale * servo_calcs_->parameters_->publish_period);

  // And with speed_units
  const_cast<moveit_servo::ServoParameters*>(servo_calcs_->parameters_.get())->command_in_type = "speed_units";
  result = servo_calcs_->scaleJointCommand(msg);
  EXPECT_EQ(result[0], servo_calcs_->parameters_->publish_period);

  // And for completeness, with invalid scaling type
  const_cast<moveit_servo::ServoParameters*>(servo_calcs_->parameters_.get())->command_in_type = "invalid_string";
  result = servo_calcs_->scaleJointCommand(msg);
  EXPECT_TRUE(result.isZero());
}

TEST_F(ServoCalcsTestFixture, TestComposeOutputMsg)
{
  // Create the input and output message
  trajectory_msgs::msg::JointTrajectory traj;
  sensor_msgs::msg::JointState joint_state;
  joint_state.name.push_back("some_joint");
  joint_state.position.push_back(1.0);
  joint_state.velocity.push_back(2.0);

  // Perform the compisition with all 3 modes published
  const_cast<moveit_servo::ServoParameters*>(servo_calcs_->parameters_.get())->publish_joint_positions = true;
  const_cast<moveit_servo::ServoParameters*>(servo_calcs_->parameters_.get())->publish_joint_velocities = true;
  const_cast<moveit_servo::ServoParameters*>(servo_calcs_->parameters_.get())->publish_joint_accelerations = true;
  servo_calcs_->composeJointTrajMessage(joint_state, traj);

  // Check the header info
  EXPECT_TRUE(traj.header.stamp == rclcpp::Time(0));  // Time should be set
  EXPECT_EQ(traj.header.frame_id, servo_calcs_->parameters_->planning_frame);
  EXPECT_EQ(traj.joint_names[0], "some_joint");

  // Check the trajectory info
  EXPECT_EQ(traj.points.size(), 1UL);
  EXPECT_EQ(traj.points[0].positions.size(), 1UL);  // Set to input length
  EXPECT_EQ(traj.points[0].velocities.size(), 1UL);
  EXPECT_EQ(traj.points[0].accelerations.size(), 7UL);  // Set to num joints
  EXPECT_EQ(traj.points[0].positions[0], 1.0);
  EXPECT_EQ(traj.points[0].velocities[0], 2.0);
  EXPECT_EQ(traj.points[0].accelerations[0], 0.0);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);

  // Set up the shared node
  TEST_NODE = std::make_shared<rclcpp::Node>("servo_calcs_test");

  // "Load" parameters from yaml as node parameters
  std::string robot_description_string, srdf_string;
  loadModelFile("moveit_resources_panda_description", "urdf/panda.urdf", robot_description_string);
  loadModelFile("moveit_resources_panda_moveit_config", "config/panda.srdf", srdf_string);
  TEST_NODE->declare_parameter<std::string>("robot_description", robot_description_string);
  TEST_NODE->declare_parameter<std::string>("robot_description_semantic", srdf_string);

  // Startup planning_scene_monitor
  TEST_TF_BUFFER = std::make_shared<tf2_ros::Buffer>(TEST_NODE->get_clock());
  TEST_PSM = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(TEST_NODE, "robot_description",
                                                                            TEST_TF_BUFFER, "planning_scene_monitor");
  // Initialize CurrentStateMonitor
  TEST_PSM->startStateMonitor();

  // read parameters and store them in shared pointer to constant
  TEST_PARAMS = moveit_servo::ServoParameters::makeServoParameters(TEST_NODE);
  if (TEST_PARAMS == nullptr)
  {
    RCLCPP_FATAL(LOGGER, "Failed to load the servo parameters");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(LOGGER, "Init finished, beginning test");

  // Actually run the tests
  int ret = RUN_ALL_TESTS();

  // Shut down the shared stuff
  TEST_PARAMS.reset();
  TEST_PSM.reset();
  TEST_TF_BUFFER.reset();
  TEST_NODE.reset();

  rclcpp::shutdown();
  return ret;
}
