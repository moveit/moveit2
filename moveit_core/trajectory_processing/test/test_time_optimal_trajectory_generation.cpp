/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Tobias Kunz <tobias@gatech.edu>
 * Date: 05/2012
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * Algorithm details and publications:
 * http://www.golems.org/node/1570
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit/utils/robot_model_test_utils.hpp>

using trajectory_processing::Path;
using trajectory_processing::TimeOptimalTrajectoryGeneration;
using trajectory_processing::Trajectory;

namespace
{
// The URDF used in moveit::core::loadTestingRobotModel() does not contain acceleration limits,
// so add them here.
// TODO(andyz): Function won't be needed once this issue has been addressed:
// https://github.com/ros/urdfdom/issues/177
void setAccelerationLimits(const moveit::core::RobotModelPtr& robot_model)
{
  const std::vector<moveit::core::JointModel*> joint_models = robot_model->getActiveJointModels();
  for (auto& joint_model : joint_models)
  {
    std::vector<moveit_msgs::msg::JointLimits> joint_bounds_msg(joint_model->getVariableBoundsMsg());
    // TODO: update individual bounds
    for (auto& joint_bound : joint_bounds_msg)
    {
      joint_bound.has_acceleration_limits = true;
      joint_bound.max_acceleration = 1.0;
    }
    joint_model->setVariableBounds(joint_bounds_msg);
  }
}
}  // namespace

TEST(time_optimal_trajectory_generation, test1)
{
  Eigen::VectorXd waypoint(4);
  std::vector<Eigen::VectorXd> waypoints;

  waypoint << 1424.0, 984.999694824219, 2126.0, 0.0;
  waypoints.push_back(waypoint);
  waypoint << 1423.0, 985.000244140625, 2126.0, 0.0;
  waypoints.push_back(waypoint);

  Eigen::VectorXd max_velocities(4);
  max_velocities << 1.3, 0.67, 0.67, 0.5;
  Eigen::VectorXd max_accelerations(4);
  max_accelerations << 0.00249, 0.00249, 0.00249, 0.00249;

  auto path_maybe = Path::create(waypoints, 100.0);
  ASSERT_TRUE(path_maybe.has_value());

  auto trajectory_maybe = Trajectory::create(*path_maybe, max_velocities, max_accelerations, 10.0);
  ASSERT_TRUE(trajectory_maybe.has_value());
  const Trajectory& trajectory = trajectory_maybe.value();

  EXPECT_DOUBLE_EQ(40.080256821829849, trajectory.getDuration());

  // Test start matches
  EXPECT_DOUBLE_EQ(1424.0, trajectory.getPosition(0.0)[0]);
  EXPECT_DOUBLE_EQ(984.999694824219, trajectory.getPosition(0.0)[1]);
  EXPECT_DOUBLE_EQ(2126.0, trajectory.getPosition(0.0)[2]);
  EXPECT_DOUBLE_EQ(0.0, trajectory.getPosition(0.0)[3]);

  // Test end matches
  EXPECT_DOUBLE_EQ(1423.0, trajectory.getPosition(trajectory.getDuration())[0]);
  EXPECT_DOUBLE_EQ(985.000244140625, trajectory.getPosition(trajectory.getDuration())[1]);
  EXPECT_DOUBLE_EQ(2126.0, trajectory.getPosition(trajectory.getDuration())[2]);
  EXPECT_DOUBLE_EQ(0.0, trajectory.getPosition(trajectory.getDuration())[3]);

  // Start at rest and end at rest
  const double traj_duration = trajectory.getDuration();
  EXPECT_NEAR(0.0, trajectory.getVelocity(0.0)[0], 0.1);
  EXPECT_NEAR(0.0, trajectory.getVelocity(traj_duration)[0], 0.1);
}

TEST(time_optimal_trajectory_generation, test2)
{
  Eigen::VectorXd waypoint(4);
  std::vector<Eigen::VectorXd> waypoints;

  waypoint << 1427.0, 368.0, 690.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 1427.0, 368.0, 790.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 952.499938964844, 433.0, 1051.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 452.5, 533.0, 1051.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 452.5, 533.0, 951.0, 90.0;
  waypoints.push_back(waypoint);

  Eigen::VectorXd max_velocities(4);
  max_velocities << 1.3, 0.67, 0.67, 0.5;
  Eigen::VectorXd max_accelerations(4);
  max_accelerations << 0.002, 0.002, 0.002, 0.002;

  auto path_maybe = Path::create(waypoints, 100.0);
  ASSERT_TRUE(path_maybe.has_value());

  auto trajectory_maybe = Trajectory::create(*path_maybe, max_velocities, max_accelerations, 10.0);
  ASSERT_TRUE(trajectory_maybe.has_value());
  const Trajectory& trajectory = trajectory_maybe.value();

  EXPECT_DOUBLE_EQ(1922.1418427445944, trajectory.getDuration());

  // Test start matches
  EXPECT_DOUBLE_EQ(1427.0, trajectory.getPosition(0.0)[0]);
  EXPECT_DOUBLE_EQ(368.0, trajectory.getPosition(0.0)[1]);
  EXPECT_DOUBLE_EQ(690.0, trajectory.getPosition(0.0)[2]);
  EXPECT_DOUBLE_EQ(90.0, trajectory.getPosition(0.0)[3]);

  // Test end matches
  EXPECT_DOUBLE_EQ(452.5, trajectory.getPosition(trajectory.getDuration())[0]);
  EXPECT_DOUBLE_EQ(533.0, trajectory.getPosition(trajectory.getDuration())[1]);
  EXPECT_DOUBLE_EQ(951.0, trajectory.getPosition(trajectory.getDuration())[2]);
  EXPECT_DOUBLE_EQ(90.0, trajectory.getPosition(trajectory.getDuration())[3]);

  // Start at rest and end at rest
  const double traj_duration = trajectory.getDuration();
  EXPECT_NEAR(0.0, trajectory.getVelocity(0.0)[0], 0.1);
  EXPECT_NEAR(0.0, trajectory.getVelocity(traj_duration)[0], 0.1);
}

TEST(time_optimal_trajectory_generation, test3)
{
  Eigen::VectorXd waypoint(4);
  std::vector<Eigen::VectorXd> waypoints;

  waypoint << 1427.0, 368.0, 690.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 1427.0, 368.0, 790.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 952.499938964844, 433.0, 1051.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 452.5, 533.0, 1051.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 452.5, 533.0, 951.0, 90.0;
  waypoints.push_back(waypoint);

  Eigen::VectorXd max_velocities(4);
  max_velocities << 1.3, 0.67, 0.67, 0.5;
  Eigen::VectorXd max_accelerations(4);
  max_accelerations << 0.002, 0.002, 0.002, 0.002;

  auto path_maybe = Path::create(waypoints, 100.0);
  ASSERT_TRUE(path_maybe.has_value());

  auto trajectory_maybe = Trajectory::create(*path_maybe, max_velocities, max_accelerations);
  ASSERT_TRUE(trajectory_maybe.has_value());
  const Trajectory& trajectory = trajectory_maybe.value();

  EXPECT_DOUBLE_EQ(1919.5597888812974, trajectory.getDuration());

  // Test start matches
  EXPECT_DOUBLE_EQ(1427.0, trajectory.getPosition(0.0)[0]);
  EXPECT_DOUBLE_EQ(368.0, trajectory.getPosition(0.0)[1]);
  EXPECT_DOUBLE_EQ(690.0, trajectory.getPosition(0.0)[2]);
  EXPECT_DOUBLE_EQ(90.0, trajectory.getPosition(0.0)[3]);

  // Test end matches
  EXPECT_DOUBLE_EQ(452.5, trajectory.getPosition(trajectory.getDuration())[0]);
  EXPECT_DOUBLE_EQ(533.0, trajectory.getPosition(trajectory.getDuration())[1]);
  EXPECT_DOUBLE_EQ(951.0, trajectory.getPosition(trajectory.getDuration())[2]);
  EXPECT_DOUBLE_EQ(90.0, trajectory.getPosition(trajectory.getDuration())[3]);

  // Start at rest and end at rest
  const double traj_duration = trajectory.getDuration();
  EXPECT_NEAR(0.0, trajectory.getVelocity(0.0)[0], 0.1);
  EXPECT_NEAR(0.0, trajectory.getVelocity(traj_duration)[0], 0.1);
}

// Test the version of computeTimeStamps that takes custom velocity/acceleration limits
TEST(time_optimal_trajectory_generation, testCustomLimits)
{
  constexpr auto robot_name{ "panda" };
  constexpr auto group_name{ "panda_arm" };

  auto robot_model = moveit::core::loadTestingRobotModel(robot_name);
  ASSERT_TRUE(robot_model) << "Failed to load robot model" << robot_name;
  setAccelerationLimits(robot_model);
  auto group = robot_model->getJointModelGroup(group_name);
  ASSERT_TRUE(group) << "Failed to load joint model group " << group_name;
  moveit::core::RobotState waypoint_state(robot_model);
  waypoint_state.setToDefaultValues();

  const double delta_t = 0.1;
  robot_trajectory::RobotTrajectory trajectory(robot_model, group);
  waypoint_state.setJointGroupPositions(group, std::vector<double>{ -0.5, -3.52, 1.35, -2.51, -0.88, 0.63, 0.0 });
  trajectory.addSuffixWayPoint(waypoint_state, delta_t);
  waypoint_state.setJointGroupPositions(group, std::vector<double>{ -0.45, -3.2, 1.2, -2.4, -0.8, 0.6, 0.0 });
  trajectory.addSuffixWayPoint(waypoint_state, delta_t);

  TimeOptimalTrajectoryGeneration totg;
  // Custom velocity & acceleration limits for some joints
  std::unordered_map<std::string, double> vel_limits{ { "panda_joint1", 1.3 }, { "panda_joint2", 2.3 },
                                                      { "panda_joint3", 3.3 }, { "panda_joint4", 4.3 },
                                                      { "panda_joint5", 5.3 }, { "panda_joint6", 6.3 },
                                                      { "panda_joint7", 7.3 } };
  std::unordered_map<std::string, double> accel_limits{ { "panda_joint1", 1.3 }, { "panda_joint2", 2.3 },
                                                        { "panda_joint3", 3.3 }, { "panda_joint4", 4.3 },
                                                        { "panda_joint5", 5.3 }, { "panda_joint6", 6.3 },
                                                        { "panda_joint7", 7.3 } };
  ASSERT_TRUE(totg.computeTimeStamps(trajectory, vel_limits, accel_limits)) << "Failed to compute time stamps";
}

// Test that totg algorithm doesn't give large acceleration
TEST(time_optimal_trajectory_generation, testLargeAccel)
{
  double path_tolerance = 0.1;
  double resample_dt = 0.1;
  Eigen::VectorXd waypoint(6);
  std::vector<Eigen::VectorXd> waypoints;
  Eigen::VectorXd max_velocities(6);
  Eigen::VectorXd max_accelerations(6);

  // Waypoints
  // clang-format off
  waypoint << 1.6113056281076339,
             -0.21400163389235427,
             -1.974502599739185,
              9.9653618690354051e-12,
             -1.3810916877429624,
              1.5293902838041467;
  waypoints.push_back(waypoint);

  waypoint << 1.6088016187976597,
             -0.21792862470933924,
             -1.9758628799742952,
              0.00010424017303217738,
             -1.3835690515335755,
              1.5279972853269816;
  waypoints.push_back(waypoint);

  waypoint << 1.5887695443178671,
             -0.24934455124521923,
             -1.9867451218551782,
              0.00093816147756670078,
             -1.4033879618584812,
              1.5168532975096607;
  waypoints.push_back(waypoint);

  waypoint << 1.1647412393815282,
             -0.91434018564402375,
             -2.2170946337498498,
              0.018590164397622583,
             -1.8229041212673529,
              1.2809632867583278;
  waypoints.push_back(waypoint);

  // Max velocities
  max_velocities << 0.89535390627300004,
                    0.89535390627300004,
                    0.79587013890930003,
                    0.92022484811399996,
                    0.82074108075029995,
                    1.3927727430915;
  // Max accelerations
  max_accelerations << 0.82673490883799994,
                       0.78539816339699997,
                       0.60883578557700002,
                       3.2074759432319997,
                       1.4398966328939999,
                       4.7292792634680003;
  // clang-format on

  auto path_maybe = Path::create(waypoints, path_tolerance);
  ASSERT_TRUE(path_maybe.has_value());

  auto trajectory_maybe = Trajectory::create(*path_maybe, max_velocities, max_accelerations, 0.001);
  ASSERT_TRUE(trajectory_maybe.has_value());
  const Trajectory& parameterized = trajectory_maybe.value();

  size_t sample_count = std::ceil(parameterized.getDuration() / resample_dt);
  for (size_t sample = 0; sample <= sample_count; ++sample)
  {
    // always sample the end of the trajectory as well
    double t = std::min(parameterized.getDuration(), sample * resample_dt);
    Eigen::VectorXd acceleration = parameterized.getAcceleration(t);

    ASSERT_EQ(acceleration.size(), 6);
    for (std::size_t i = 0; i < 6; ++i)
      EXPECT_NEAR(acceleration(i), 0.0, 100.0) << "Invalid acceleration at position " << sample_count << '\n';
  }
}

// Second test for large accelerations with specific waypoints
TEST(time_optimal_trajectory_generation, testLargeAccel2)
{
  double path_tolerance = 0.1;
  double resample_dt = 0.05;
  Eigen::VectorXd waypoint(7);
  std::vector<Eigen::VectorXd> waypoints;
  Eigen::VectorXd max_velocities(7);
  Eigen::VectorXd max_accelerations(7);

  // Waypoints
  // clang-format off
  waypoint <<  0x1.5db22d0e56042p-1,  0x1.322d0e5604188p-1, -0x1.cc083126e978ep+0, -0x1.9604189374bc8p-1, -0x1.0e5604189374cp-2,  0x1.2ed916872b021p+0, -0x1.b7ced916872b2p-1;
  waypoints.push_back(waypoint);
  waypoint <<  0x1.5d0744a3ea288p-1,  0x1.32da9311e916ep-1, -0x1.cb99705dd41bbp+0, -0x1.95cc2cff9407dp-1, -0x1.0f85016c80febp-2,  0x1.2ea00a3a7152bp+0, -0x1.b6594af4f0d86p-1;
  waypoints.push_back(waypoint);
  waypoint <<  0x1.5b068b64a6959p-1,  0x1.34e3214598121p-1, -0x1.ca4d2e0294042p+0, -0x1.95246a43f1e9cp-1, -0x1.1311f968499c9p-2,  0x1.2df4e5544444ap+0, -0x1.b1f8a0902de03p-1;
  waypoints.push_back(waypoint);
  waypoint <<  0x1.57b001508b4b5p-1,  0x1.3846b8f1110a1p-1, -0x1.c8236a1529322p+0, -0x1.940cd0608e626p-1, -0x1.18fcec0bed4e6p-2,  0x1.2cd7a7d4a3d7ep+0, -0x1.aaacd9e83e428p-1;
  waypoints.push_back(waypoint);
  waypoint <<  0x1.5303a6679849dp-1,  0x1.3d055a1453feep-1, -0x1.c51c249593a5dp+0, -0x1.92855f5569719p-1, -0x1.2145d9576c141p-2,  0x1.2b4851bb900c7p+0, -0x1.a075f6fd21ff5p-1;
  waypoints.push_back(waypoint);
  waypoint <<   0x1.4d017aa9cd91p-1,  0x1.431f04af60f07p-1, -0x1.c1375d83d35f2p+0, -0x1.908e172283177p-1, -0x1.2becc14ac5edbp-2,  0x1.2946e30908e24p+0, -0x1.9353f7ced916ap-1;
  waypoints.push_back(waypoint);
  waypoint <<  0x1.45a97e172b20ep-1,  0x1.4a93b8c237dedp-1,  -0x1.bc7514dfe85ep+0, -0x1.8e26f7c7db53fp-1, -0x1.38f1a3e5fadb4p-2,  0x1.26d35bbd0e596p+0, -0x1.8346dc5d63888p-1;
  waypoints.push_back(waypoint);
  waypoint <<  0x1.3cfbb0afb0f98p-1,  0x1.5363764cd8c9fp-1, -0x1.b6d54aa9d2a28p+0, -0x1.8b50014572271p-1, -0x1.485481290adcbp-2,  0x1.23edbbd7a071dp+0, -0x1.704ea4a8c154fp-1;
  waypoints.push_back(waypoint);
  waypoint <<  0x1.32f812735f1adp-1,  0x1.5d8e3d4f43b1ep-1, -0x1.b057fee1922cbp+0, -0x1.8809339b4790ep-1, -0x1.5a155913f5f21p-2,  0x1.20960358bf2b8p+0, -0x1.5a6b50b0f27bep-1;
  waypoints.push_back(waypoint);
  waypoint <<  0x1.279ea3623584dp-1,  0x1.69140dc97896ap-1, -0x1.a8fd318726fc7p+0, -0x1.84528ec95b914p-1, -0x1.6e342ba6bc1b6p-2,  0x1.1ccc32406a868p+0, -0x1.419ce075f6fd4p-1;
  waypoints.push_back(waypoint);
  waypoint <<  0x1.1aef637c34379p-1,  0x1.75f4e7bb77782p-1, -0x1.a0c4e29a9111dp+0, -0x1.802c12cfae285p-1, -0x1.84b0f8e15d589p-2,  0x1.1890488ea282dp+0, -0x1.25e353f7ced94p-1;
  waypoints.push_back(waypoint);
  waypoint <<   0x1.0cea52c15b33p-1,  0x1.8430cb2540568p-1, -0x1.97af121bd06ccp+0,  -0x1.7b95bfae3f56p-1, -0x1.9d8bc0c3d9a9cp-2,  0x1.13e2464367207p+0, -0x1.073eab367a0fcp-1;
  waypoints.push_back(waypoint);
  waypoint <<  0x1.fb1ee26354ee4p-2,  0x1.93c7b806d331ap-1, -0x1.8dbbc00ae50d6p+0, -0x1.768f95650f1a5p-1, -0x1.b8c4834e310ecp-2,  0x1.0ec22b5eb85f5p+0, -0x1.cb5dcc63f1417p-2;
  waypoints.push_back(waypoint);
  waypoint <<   0x1.d9bd7d9a4408p-2,  0x1.a4b9ae6030099p-1, -0x1.82eaec67cef3ap+0, -0x1.711993f41d754p-1, -0x1.d65b40806387cp-2,  0x1.092ff7e0963f8p+0, -0x1.826809d495186p-2;
  waypoints.push_back(waypoint);
  waypoint <<  0x1.b5b0772783b31p-2,  0x1.b706ae3156de5p-1, -0x1.773c97328e1f7p+0, -0x1.6b33bb5b6a66dp-1, -0x1.f64ff85a7114bp-2,  0x1.032babc900c0fp+0, -0x1.339c0ebedfa46p-2;
  waypoints.push_back(waypoint);
  waypoint <<  0x1.8ef7cf0b13ef8p-2,  0x1.caaeb77a47afep-1, -0x1.6ab0c06b2290ep+0,  -0x1.64de0b9af5efp-1, -0x1.0c51556e2cdacp-1,  0x1.f96a8e2fefc77p-1, -0x1.bdf3b645a1cacp-3;
  waypoints.push_back(waypoint);
  waypoint <<  0x1.65938544f4bd8p-2,  0x1.dfb1ca3b027e2p-1, -0x1.5d4768118c47fp+0, -0x1.5e1884b2c00dep-1, -0x1.1ea9ac030eb52p-1,  0x1.eb99939af74f9p-1,  -0x1.0902de00d1b7p-3;
  waypoints.push_back(waypoint);
  waypoint <<  0x1.398399d5261cdp-2,  0x1.f60fe67387495p-1, -0x1.4f008e25cb44ap+0, -0x1.56e326a2c8c36p-1, -0x1.3230ffebde198p-1,  0x1.dce467d3181a4p-1,  -0x1.219652bd3c35p-5;
  waypoints.push_back(waypoint);
  waypoint <<  0x1.0acc12f864b3bp-2,  0x1.06e3809392e2fp+0, -0x1.3fdd80778ca03p+0, -0x1.4f3e99f667ef6p-1, -0x1.46e5888d22c0ap-1,  0x1.cd4c62bb6e88bp-1,  0x1.07a51c29cd491p-4;
  waypoints.push_back(waypoint);
  waypoint <<  0x1.b8f2da4b93804p-3,  0x1.12a5730c4875cp+0, -0x1.30db2275933e4p+0, -0x1.47aa8e33eecfep-1, -0x1.5b6d5b5e94286p-1,  0x1.bdd609e2ef87dp-1,  0x1.4e51b4b526d26p-3;
  waypoints.push_back(waypoint);
  waypoint <<  0x1.61a4d1f9bc768p-3,  0x1.1db9e0c9190a1p+0, -0x1.22b64605c496cp+0, -0x1.408659993719dp-1, -0x1.6ec630dc18062p-1,  0x1.af43e23d57446p-1,  0x1.069235245a34ep-2;
  waypoints.push_back(waypoint);
  waypoint <<  0x1.0fae0cfb4449cp-3,    0x1.2820c9ca04ap+0, -0x1.156eeb2820a9ap+0, -0x1.39d1fc2640cd1p-1,  -0x1.80f00905ae5ap-1,  0x1.a195ebcaa5be6p-1,  0x1.60255767c7b5fp-2;
  waypoints.push_back(waypoint);
  waypoint <<  0x1.861d16a055f44p-4,  0x1.31da2e0f0b37ap+0, -0x1.090511dca776ep+0, -0x1.338d75db0be9bp-1, -0x1.91eae3db5723ep-1,  0x1.94cc268adaf5bp-1,  0x1.b3e24124dbebfp-2;
  waypoints.push_back(waypoint);
  waypoint <<   0x1.ef1933e1c21fp-5,  0x1.3ae60d982cd0dp+0,  -0x1.faf17446b1fdp-1, -0x1.2db8c6b7986fap-1, -0x1.a1b6c15d1263dp-1,  0x1.88e6927df6ea8p-1,  0x1.00e4792dcb6b8p-1;
  waypoints.push_back(waypoint);
  waypoint <<   0x1.ceaa8fa0a794p-6,  0x1.43446865696b9p+0,  -0x1.e593c7f86a81p-1,  -0x1.2853eebbe65fp-1, -0x1.b053a18ae019cp-1,  0x1.7de52fa3f99cbp-1,  0x1.24ecb585fc3b9p-1;
  waypoints.push_back(waypoint);
  waypoint <<  -0x1.6232de73e2cp-10,  0x1.4af53e76c107fp+0, -0x1.d1f11ece7879cp-1, -0x1.235eede7f5b7bp-1, -0x1.bdc18464c045cp-1,  0x1.73c7fdfce30c5p-1,  0x1.4609d59b00662p-1;
  waypoints.push_back(waypoint);
  waypoint <<  -0x1.d036d0d42d03p-6,  0x1.51f88fcc33a5fp+0, -0x1.c00978c8dbe75p-1, -0x1.1ed9c43bc679dp-1, -0x1.ca0069eab2e7dp-1,  0x1.6a8efd88b3395p-1,  0x1.643bd96cd7eb3p-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.afc82c9312788p-5,  0x1.584e5c65c1458p+0, -0x1.afdcd5e794c9ap-1, -0x1.1ac471b758a54p-1, -0x1.d510521cb7ffep-1,  0x1.623a2e476a23cp-1,  0x1.7f82c0fb82cadp-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.310bf1b7497d4p-4,  0x1.5df6a44369e6bp+0, -0x1.a16b362aa320cp-1, -0x1.171ef65aac3a1p-1,  -0x1.def13cfacf8ep-1,  0x1.5ac9903907cb9p-1,  0x1.97de8c470104fp-1;
  waypoints.push_back(waypoint);
  waypoint <<  -0x1.7f85467e4c04p-4,  0x1.62f167652d898p+0, -0x1.94b4999206ecap-1, -0x1.13e95225c1384p-1, -0x1.e7a32a84f9922p-1,  0x1.543d235d8c30dp-1,  0x1.ad4f3b4f52998p-1;
  waypoints.push_back(waypoint);
  waypoint <<   -0x1.c350149e90dp-4,  0x1.673ea5cb0c2dep+0, -0x1.89b9001dc02d3p-1, -0x1.11238518979fcp-1, -0x1.ef261abb360c6p-1,  0x1.4e94e7b4f7538p-1,  0x1.bfd4ce147788bp-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.fc6c5c1817e28p-4,  0x1.6ade5f7505d3ep+0, -0x1.807869cdcee29p-1, -0x1.0ecd8f332f70ap-1, -0x1.f57a0d9d84fcap-1,  0x1.49d0dd3f49339p-1,  0x1.cf6f44966fd27p-1;
  waypoints.push_back(waypoint);
  waypoint <<  -0x1.156d0e75709dp-3,  0x1.6dd094631a7b7p+0, -0x1.78f2d6a2330ccp-1, -0x1.0ce7707588aafp-1, -0x1.fa9f032be662ep-1,  0x1.45f103fc81d11p-1,  0x1.dc1e9ed53b76ap-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.274cab8b766bap-3,  0x1.701544954a24ap+0, -0x1.7328469aecabap-1, -0x1.0b7128dfa34e9p-1, -0x1.fe94fb665a3f4p-1,   0x1.42f55beca12cp-1,  0x1.e5e2dcd0da755p-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.33d5054e1d5d4p-3,  0x1.71ac700b94cf7p+0, -0x1.6f18b9b7fbbf6p-1, -0x1.0a6ab8717f5b9p-1, -0x1.00adfb267048dp+0,  0x1.40dde50fa7444p-1,  0x1.ecbbfe894cce7p-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.3b061bbd65716p-3,  0x1.729616c5fa7bdp+0, -0x1.6cc42ff96047dp-1, -0x1.09d41f2b1cd1fp-1,  -0x1.0179f9efbcadp+0,   0x1.3faa9f65941ap-1,  0x1.f0aa03fe92824p-1;
  waypoints.push_back(waypoint);
  waypoint <<  -0x1.3cf007cc4141p-3,  0x1.72d443c12b752p+0,   -0x1.6c25722065ep-1, -0x1.09ac0bf5cbf1ep-1, -0x1.01b042aa8a936p+0,  0x1.3f58db282eeaep-1,  0x1.f1b5b8fdab576p-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.3d4d897a0140ap-3,  0x1.733d53b096a52p+0, -0x1.6c31e46d62392p-1, -0x1.097fd5ba56286p-1, -0x1.01c6080109181p+0,  0x1.3eaa38452d0a6p-1,  0x1.f1913b094b87ap-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.3d94892aa1577p-3,  0x1.738f0b2620122p+0, -0x1.6c3a686a8fd73p-1, -0x1.095b259c2afb6p-1, -0x1.01d68e294e095p+0,   0x1.3e2425139855p-1,  0x1.f172c6a118445p-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.3deb7cf23b3eep-3,  0x1.73f81c9c204aap+0, -0x1.6c426fd16088ap-1,  -0x1.092635c96f7ep-1, -0x1.01eac6a49f9e1p+0,  0x1.3d7c280f46989p-1,  0x1.f1466d4f5f105p-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.3e5295d13db0bp-3,  0x1.7478872873856p+0, -0x1.6c4a1c13e087bp-1, -0x1.08e148e26774cp-1, -0x1.0202bd0aba6f6p+0,  0x1.3cb2102d99989p-1,   0x1.f10c6b0c60e4p-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.3ecb9ba9091efp-3,   0x1.75104a868a0fp+0, -0x1.6c52970913301p-1, -0x1.088ebb4ea6a78p-1, -0x1.021eda7bc4d3bp+0,  0x1.3bc411a37438fp-1,  0x1.f0c6d6fa707d2p-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.3f50d1e12df7cp-3,  0x1.75bf66a000068p+0, -0x1.6c57c197b40a1p-1, -0x1.0826dcb596e07p-1, -0x1.023db09690521p+0,  0x1.3ab81237a6597p-1,  0x1.f06e84132108bp-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.3fde92391dcfdp-3,  0x1.7685dab625854p+0, -0x1.6c56fced448c5p-1, -0x1.07a4ca09cb5d3p-1, -0x1.025e564c98983p+0,   0x1.3991d2df6da7p-1,  0x1.effee2be0cac5p-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.407252e8fb4e8p-3,  0x1.7763a5bb7c284p+0, -0x1.6c4e76c071ca6p-1, -0x1.07051e09c599dp-1, -0x1.02802987ecfaap+0,  0x1.3853f05c10ff5p-1,  0x1.ef74c7afcef97p-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.4111386827e85p-3,  0x1.7858c88ae0103p+0, -0x1.6c41c4d2edd06p-1, -0x1.064eba310fbe8p-1, -0x1.02a46b8184816p+0,  0x1.36f923fe4a69cp-1,  0x1.eed6811e51329p-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.41b2a19cb16dep-3,  0x1.796540ca766dep+0, -0x1.6c2a0fb788822p-1, -0x1.05760707ea485p-1, -0x1.02c8ca1600cd4p+0,  0x1.358a620df5dc4p-1,  0x1.ee188b19b5f08p-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.425151dcd1213p-3,  0x1.7a890c0a854c5p+0, -0x1.6c03467a61141p-1, -0x1.0473fe9b2e631p-1, -0x1.02ebe24a452a5p+0,  0x1.340d17e7c8082p-1,  0x1.ed340263ce56ep-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.42e82f92f81ddp-3,  0x1.7bc2ba9595f6bp+0, -0x1.6bca3e6bb8668p-1,  -0x1.0344181d0c75p-1, -0x1.030c631a26addp+0,   0x1.32879e4f8cb9p-1,  0x1.ec2455dce211bp-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.437e35b9dd5e6p-3,  0x1.7d11bf0eef37cp+0, -0x1.6b84944d3dc99p-1, -0x1.01f0a48e53f38p-1, -0x1.032c2171bbd72p+0,  0x1.30f31b780cccdp-1,  0x1.eaf36b3143aacp-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.440b606a1ec23p-3,  0x1.7e74b83dc2ee7p+0, -0x1.6b2c0023a2148p-1, -0x1.00716214d5cc6p-1, -0x1.0348d2c6a3e97p+0,  0x1.2f58a21559a46p-1,  0x1.e997e2fb3c30dp-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.4487960689d05p-3,  0x1.7fe6cc06d1209p+0, -0x1.6abc9926b16cap-1, -0x1.fd86bf6b747c8p-2, -0x1.03605bfef2afcp+0,  0x1.2dc4182d7284dp-1,  0x1.e80e028279c79p-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.44ecd03439baap-3,  0x1.81600f1c5a8b2p+0, -0x1.6a35f668ee88ap-1, -0x1.f9d6b4e09deeap-2, -0x1.03714ccc1662dp+0,  0x1.2c41942b1b135p-1,  0x1.e65996f9a4ff1p-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.453cad0f57ceap-3,  0x1.82c7c2200bf9cp+0,  -0x1.69a7c455c957p-1, -0x1.f6268097dc1c2p-2, -0x1.037cd332dc58cp+0,  0x1.2ae2019bdac44p-1,  0x1.e4a297feee42dp-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.4579b2f704b43p-3,  0x1.84182e9db9ffcp+0, -0x1.69171da4a725ep-1, -0x1.f2904e7f3350ap-2, -0x1.0383bc77bc2e1p+0,  0x1.29a6881c7bf67p-1,  0x1.e2f4f14ad5ebbp-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.45a052a08304cp-3,  0x1.85513fd7f306ap+0, -0x1.6881035ba4875p-1, -0x1.ef0b45e3b4914p-2, -0x1.038506efd3e86p+0,  0x1.2892ad3324e4dp-1,  0x1.e14bca2e1e5d3p-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.45b371cd72e22p-3,  0x1.8672f50321be5p+0, -0x1.67e7eeb604392p-1, -0x1.eb9ed7c953f48p-2,  -0x1.038184909243p+0,  0x1.27a39de9db4c7p-1,  0x1.dfab2e5aeb00fp-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.45b86906a868ap-3,  0x1.877d2b286b617p+0, -0x1.67508398547a1p-1, -0x1.e85929eebcd2cp-2, -0x1.037ab9c14c95bp+0,  0x1.26d43de135caep-1,  0x1.de1ac7ebcac27p-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.45b67f4fef6bbp-3,  0x1.88700b722d9c4p+0, -0x1.66c0e4875ec39p-1, -0x1.e54cf15e9d00ap-2, -0x1.0372b8328538ep+0,  0x1.261d0a3771a21p-1,  0x1.dca490d0af0afp-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.45a9a21ce6456p-3,  0x1.894b905e134d5p+0, -0x1.6635a0ec10e3bp-1, -0x1.e270624662f37p-2, -0x1.03685cd6e7df6p+0,   0x1.2581b2935afap-1,  0x1.db42f704e8752p-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.459497e05dda5p-3,  0x1.8a0fb7536b90bp+0, -0x1.65b115843b269p-1, -0x1.dfca573cbd4dep-2, -0x1.035c6edf06231p+0,  0x1.24ffab8294158p-1,  0x1.d9f9d358ab8e5p-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.457b585d936f8p-3,  0x1.8abc7fc5a190cp+0, -0x1.6536a11740ec8p-1, -0x1.dd6492c6df2b5p-2, -0x1.03500afc8451cp+0,  0x1.24934f34c4e15p-1,  0x1.d8cea0fe2a154p-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.45647e97fc248p-3,  0x1.8b51f5b0aec3fp+0, -0x1.64cbda87308dcp-1, -0x1.db4f32ee59241p-2, -0x1.03450ad2e9214p+0,  0x1.24367ceeb066ep-1,   0x1.d7ca6e614fabp-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.454e3d261e74dp-3,  0x1.8bd019e41d698p+0, -0x1.646f47b3f4971p-1, -0x1.d985fb425bc5dp-2, -0x1.033aed9dd7f2cp+0,  0x1.23eabf4d9d44cp-1,  0x1.d6eacbc40d5a8p-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.45398953c8a1dp-3,  0x1.8c36e1c08fbaep+0, -0x1.6421b9cc77cc2p-1, -0x1.d80b54fc03074p-2, -0x1.0331f8390039dp+0,  0x1.23af47be23d22p-1,  0x1.d631169aa6f35p-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.452844954d62fp-3,  0x1.8c864d943a748p+0, -0x1.63e4bef3b50b8p-1, -0x1.d6e3b7c13a8eep-2, -0x1.032ab123cc472p+0,   0x1.238270d3cbccp-1,  0x1.d59fdc597c07ap-1;
  waypoints.push_back(waypoint);
  waypoint << -0x1.4512d19bc860fp-3,  0x1.8ce8975896428p+0, -0x1.63993710f49ddp-1, -0x1.d575b1492907dp-2, -0x1.0321a9de11b82p+0,  0x1.234b049401f47p-1,  0x1.d4ec06f299cbdp-1;
  waypoints.push_back(waypoint);

  // Max velocities
  max_velocities << 1.9, 1.9, 2.85, 2.85, 2.85, 2.85, 2.85;
  // Max accelerations
  max_accelerations << 0.57, 0.57, 0.57, 0.57, 0.57, 0.57, 0.57;
  // clang-format on

  auto path_maybe = Path::create(waypoints, path_tolerance);
  ASSERT_TRUE(path_maybe.has_value());

  auto trajectory_maybe = Trajectory::create(*path_maybe, max_velocities, max_accelerations, 0.001);
  ASSERT_TRUE(trajectory_maybe.has_value());
  const Trajectory& parameterized = trajectory_maybe.value();

  size_t sample_count = std::ceil(parameterized.getDuration() / resample_dt);
  for (size_t sample = 0; sample <= sample_count; ++sample)
  {
    // always sample the end of the trajectory as well
    double t = std::min(parameterized.getDuration(), sample * resample_dt);
    Eigen::VectorXd position = parameterized.getPosition(t);
    Eigen::VectorXd velocity = parameterized.getVelocity(t);
    Eigen::VectorXd acceleration = parameterized.getAcceleration(t);

    ASSERT_EQ(acceleration.size(), 7);
    for (std::size_t i = 0; i < 7; ++i)
      EXPECT_NEAR(acceleration(i), 0.0, 100.0) << "Invalid acceleration at position " << sample_count << " (time: " << t << ")\n";
  }
}

// This test differentiates the trajectory velocities and verifies that the acceleration limit is actually respected.
// A path tolerance of 0.0 makes this test fail. Output trajectory accelerations are all within the limits, but there
// are jumps in velocity, i.e. the derivative of the velocity is not consistent with the acceleration.
// The test is here mostly to prevent us from making a path tolerance of 0.0 a valid default again in the future.
TEST(time_optimal_trajectory_generation, AccelerationLimitIsRespected)
{
  double path_tolerance = 0.001;
  double resample_dt = 0.01;

  // Waypoints.
  std::vector<Eigen::VectorXd> waypoints;
  waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
  waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
  waypoints.push_back(Eigen::Vector3d(1.0, 1.0, 0.0));

  Eigen::VectorXd max_velocities = Eigen::VectorXd::Constant(3, 0.1);
  Eigen::VectorXd max_accelerations = Eigen::VectorXd::Constant(3, 0.5);

  auto path_maybe = Path::create(waypoints, path_tolerance);
  ASSERT_TRUE(path_maybe.has_value());

  auto trajectory_maybe = Trajectory::create(*path_maybe, max_velocities, max_accelerations, 0.001);
  ASSERT_TRUE(trajectory_maybe.has_value());
  const Trajectory& parameterized = trajectory_maybe.value();

  size_t sample_count = std::ceil(parameterized.getDuration() / resample_dt);
  Eigen::Vector3d previous_velocity = Eigen::Vector3d::Zero();
  for (size_t sample = 0; sample <= sample_count; ++sample)
  {
    double t = std::min(parameterized.getDuration(), sample * resample_dt);
    Eigen::Vector3d velocity = parameterized.getVelocity(t);

    double acceleration = (velocity - previous_velocity).norm() / resample_dt;
    EXPECT_LT(acceleration, max_accelerations.norm() + 1e-3);
    previous_velocity = velocity;
  }
}

// A path that requires a full 180 degree turn at any point is not supported by the current implementation.
// Path::create() should fail.
TEST(time_optimal_trajectory_generation, PathMakes180DegreeTurn)
{
  // Waypoints.
  std::vector<Eigen::VectorXd> waypoints;
  waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
  waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
  waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));

  EXPECT_FALSE(Path::create(waypoints, /*path_tolerance=*/0.01));
}

// Test parameterizing a trajectory would always produce a trajectory with output end waypoint same as the input end
// waypoint
TEST(time_optimal_trajectory_generation, testLastWaypoint)
{
  constexpr auto robot_name{ "panda" };
  constexpr auto group_name{ "panda_arm" };

  auto robot_model = moveit::core::loadTestingRobotModel(robot_name);
  ASSERT_TRUE(robot_model) << "Failed to load robot model" << robot_name;
  setAccelerationLimits(robot_model);
  auto group = robot_model->getJointModelGroup(group_name);
  ASSERT_TRUE(group) << "Failed to load joint model group " << group_name;
  moveit::core::RobotState waypoint_state(robot_model);
  waypoint_state.setToDefaultValues();

  robot_trajectory::RobotTrajectory trajectory(robot_model, group);
  auto add_waypoint = [&](const std::vector<double>& waypoint) {
    waypoint_state.setJointGroupPositions(group, waypoint);
    trajectory.addSuffixWayPoint(waypoint_state, 0.1);
  };
  add_waypoint({ 0.000000000, 0.000000000, 0, 0, 0, 0, 0 });
  add_waypoint({ 0.009521808, 0.009521808, 0, 0, 0, 0, 0 });
  add_waypoint({ 0.011902261, 0.011902261, 0, 0, 0, 0, 0 });
  add_waypoint({ 0.016663165, 0.016663165, 0, 0, 0, 0, 0 });
  add_waypoint({ 0.026184973, 0.026184973, 0, 0, 0, 0, 0 });
  add_waypoint({ 0.034516555, 0.034516555, 0, 0, 0, 0, 0 });

  const std::vector<double> expected_last_waypoint = { 0.034516555, 0.034516555, 0, 0, 0, 0, 0 };
  add_waypoint(expected_last_waypoint);

  TimeOptimalTrajectoryGeneration totg;
  ASSERT_TRUE(totg.computeTimeStamps(trajectory)) << "Failed to compute time stamps";
  const auto robot_state = trajectory.getLastWayPoint();
  std::vector<double> actual_last_waypoint;
  robot_state.copyJointGroupPositions(group, actual_last_waypoint);
  EXPECT_TRUE(std::equal(actual_last_waypoint.cbegin(), actual_last_waypoint.cend(), expected_last_waypoint.cbegin(),
                         expected_last_waypoint.cend(), [](const double rhs, const double lhs) {
                           return std::abs(rhs - lhs) < std::numeric_limits<double>::epsilon();
                         }));
}

TEST(time_optimal_trajectory_generation, testPluginAPI)
{
  constexpr auto robot_name{ "panda" };
  constexpr auto group_name{ "panda_arm" };

  auto robot_model = moveit::core::loadTestingRobotModel(robot_name);
  ASSERT_TRUE(robot_model) << "Failed to load robot model" << robot_name;
  setAccelerationLimits(robot_model);
  auto group = robot_model->getJointModelGroup(group_name);
  ASSERT_TRUE(group) << "Failed to load joint model group " << group_name;
  moveit::core::RobotState waypoint_state(robot_model);
  waypoint_state.setToDefaultValues();

  robot_trajectory::RobotTrajectory trajectory(robot_model, group);
  waypoint_state.setJointGroupPositions(group, std::vector<double>{ -0.5, -3.52, 1.35, -2.51, -0.88, 0.63, 0.0 });
  trajectory.addSuffixWayPoint(waypoint_state, 0.1);
  waypoint_state.setJointGroupPositions(group, std::vector<double>{ -0.5, -3.52, 1.35, -2.51, -0.88, 0.63, 0.0 });
  trajectory.addSuffixWayPoint(waypoint_state, 0.1);
  waypoint_state.setJointGroupPositions(group, std::vector<double>{ 0.0, -3.5, 1.4, -1.2, -1.0, -0.2, 0.0 });
  trajectory.addSuffixWayPoint(waypoint_state, 0.1);
  waypoint_state.setJointGroupPositions(group, std::vector<double>{ -0.5, -3.00, 1.35, -2.51, -0.88, 0.63, 0.0 });
  trajectory.addSuffixWayPoint(waypoint_state, 0.1);
  waypoint_state.setJointGroupPositions(group, std::vector<double>{ 0.0, -3.5, 1.4, -1.2, -1.5, -0.2, 0.0 });
  trajectory.addSuffixWayPoint(waypoint_state, 0.1);
  waypoint_state.setJointGroupPositions(group, std::vector<double>{ -0.5, -3.52, 1.35, -2.51, -0.88, 0.63, 0.0 });
  trajectory.addSuffixWayPoint(waypoint_state, 0.1);

  // Number TOTG iterations
  constexpr size_t totg_iterations = 10;

  // Test computing the dynamics repeatedly with the same totg instance
  moveit_msgs::msg::RobotTrajectory first_trajectory_msg_start, first_trajectory_msg_end;
  {
    robot_trajectory::RobotTrajectory test_trajectory(trajectory, true /* deep copy */);

    // Test if the trajectory was copied correctly
    ASSERT_EQ(test_trajectory.getDuration(), trajectory.getDuration());
    moveit::core::JointBoundsVector test_bounds = test_trajectory.getRobotModel()->getActiveJointModelsBounds();
    moveit::core::JointBoundsVector original_bounds = trajectory.getRobotModel()->getActiveJointModelsBounds();
    ASSERT_EQ(test_bounds.size(), original_bounds.size());
    for (size_t bound_idx = 0; bound_idx < test_bounds.at(0)->size(); ++bound_idx)
    {
      ASSERT_EQ(test_bounds.at(0)->at(bound_idx).max_velocity_, original_bounds.at(0)->at(bound_idx).max_velocity_);
      ASSERT_EQ(test_bounds.at(0)->at(bound_idx).min_velocity_, original_bounds.at(0)->at(bound_idx).min_velocity_);
      ASSERT_EQ(test_bounds.at(0)->at(bound_idx).velocity_bounded_,
                original_bounds.at(0)->at(bound_idx).velocity_bounded_);

      ASSERT_EQ(test_bounds.at(0)->at(bound_idx).max_acceleration_,
                original_bounds.at(0)->at(bound_idx).max_acceleration_);
      ASSERT_EQ(test_bounds.at(0)->at(bound_idx).min_acceleration_,
                original_bounds.at(0)->at(bound_idx).min_acceleration_);
      ASSERT_EQ(test_bounds.at(0)->at(bound_idx).acceleration_bounded_,
                original_bounds.at(0)->at(bound_idx).acceleration_bounded_);
    }
    ASSERT_EQ(test_trajectory.getWayPointDurationFromPrevious(1), trajectory.getWayPointDurationFromPrevious(1));

    TimeOptimalTrajectoryGeneration totg;
    ASSERT_TRUE(totg.computeTimeStamps(test_trajectory)) << "Failed to compute time stamps";

    test_trajectory.getRobotTrajectoryMsg(first_trajectory_msg_start);

    // Iteratively recompute timestamps with same totg instance
    for (size_t i = 0; i < totg_iterations; ++i)
    {
      bool totg_success = totg.computeTimeStamps(test_trajectory);
      EXPECT_TRUE(totg_success) << "Failed to compute time stamps with a same TOTG instance in iteration " << i;
    }

    test_trajectory.getRobotTrajectoryMsg(first_trajectory_msg_end);
  }

  // Test computing the dynamics repeatedly with one TOTG instance per call
  moveit_msgs::msg::RobotTrajectory second_trajectory_msg_start, second_trajectory_msg_end;
  {
    robot_trajectory::RobotTrajectory test_trajectory(trajectory, true /* deep copy */);

    {
      TimeOptimalTrajectoryGeneration totg;
      ASSERT_TRUE(totg.computeTimeStamps(test_trajectory)) << "Failed to compute time stamps";
    }

    test_trajectory.getRobotTrajectoryMsg(second_trajectory_msg_start);

    // Iteratively recompute timestamps with new totg instances
    for (size_t i = 0; i < totg_iterations; ++i)
    {
      TimeOptimalTrajectoryGeneration totg;
      bool totg_success = totg.computeTimeStamps(test_trajectory);
      EXPECT_TRUE(totg_success) << "Failed to compute time stamps with a new TOTG instance in iteration " << i;
    }

    test_trajectory.getRobotTrajectoryMsg(second_trajectory_msg_end);
  }

  // Make sure trajectories produce equal waypoints independent of TOTG instances
  ASSERT_EQ(first_trajectory_msg_start, second_trajectory_msg_start);
  ASSERT_EQ(first_trajectory_msg_end, second_trajectory_msg_end);

  // Iterate on the original trajectory again
  moveit_msgs::msg::RobotTrajectory third_trajectory_msg_end;

  {
    TimeOptimalTrajectoryGeneration totg;
    ASSERT_TRUE(totg.computeTimeStamps(trajectory)) << "Failed to compute time stamps";
  }

  for (size_t i = 0; i < totg_iterations; ++i)
  {
    TimeOptimalTrajectoryGeneration totg;
    bool totg_success = totg.computeTimeStamps(trajectory);
    ASSERT_TRUE(totg_success) << "Failed to compute timestamps on a new TOTG instance in iteration " << i;
  }

  // Compare with previous work
  trajectory.getRobotTrajectoryMsg(third_trajectory_msg_end);

  // Make sure trajectories produce equal waypoints independent of TOTG instances
  ASSERT_EQ(first_trajectory_msg_end, third_trajectory_msg_end);
}

TEST(time_optimal_trajectory_generation, testFixedNumWaypoints)
{
  // Test the version of computeTimeStamps() that gives a fixed num waypoints
  constexpr size_t desired_num_waypoints = 42;

  constexpr auto robot_name{ "panda" };
  constexpr auto group_name{ "panda_arm" };

  auto robot_model = moveit::core::loadTestingRobotModel(robot_name);
  ASSERT_TRUE(robot_model) << "Failed to load robot model" << robot_name;
  setAccelerationLimits(robot_model);
  auto group = robot_model->getJointModelGroup(group_name);
  ASSERT_TRUE(group) << "Failed to load joint model group " << group_name;
  moveit::core::RobotState waypoint_state(robot_model);
  waypoint_state.setToDefaultValues();

  const double delta_t = 0.1;
  robot_trajectory::RobotTrajectory trajectory(robot_model, group);
  waypoint_state.setJointGroupPositions(group, std::vector<double>{ -0.5, -3.52, 1.35, -2.51, -0.88, 0.63, 0.0 });
  trajectory.addSuffixWayPoint(waypoint_state, delta_t);
  waypoint_state.setJointGroupPositions(group, std::vector<double>{ -0.45, -3.2, 1.2, -2.4, -0.8, 0.6, 0.0 });
  trajectory.addSuffixWayPoint(waypoint_state, delta_t);

  ASSERT_TRUE(trajectory_processing::totgComputeTimeStamps(desired_num_waypoints, trajectory)) << "Failed to compute "
                                                                                                  "time stamps";
  // Allow +/-1 waypoint due to floating point error
  EXPECT_NEAR(trajectory.getWayPointCount(), desired_num_waypoints, 1);
}

TEST(time_optimal_trajectory_generation, testSingleDofDiscontinuity)
{
  // Test a (prior) specific failure case
  Eigen::VectorXd waypoint(1);
  std::vector<Eigen::VectorXd> waypoints;

  const double start_position = 1.881943;
  waypoint << start_position;
  waypoints.push_back(waypoint);
  waypoint << 2.600542;
  waypoints.push_back(waypoint);

  Eigen::VectorXd max_velocities(1);
  max_velocities << 4.54;
  Eigen::VectorXd max_accelerations(1);
  max_accelerations << 28.0;

  auto path_maybe = Path::create(waypoints, 0.1 /* path tolerance */);
  ASSERT_TRUE(path_maybe.has_value());

  auto trajectory_maybe = Trajectory::create(*path_maybe, max_velocities, max_accelerations, 0.001 /* timestep */);
  ASSERT_TRUE(trajectory_maybe.has_value());
  const Trajectory& trajectory = trajectory_maybe.value();

  EXPECT_GT(trajectory.getDuration(), 0.0);
  const double traj_duration = trajectory.getDuration();
  EXPECT_NEAR(0.320681, traj_duration, 1e-3);

  // Start matches
  EXPECT_DOUBLE_EQ(start_position, trajectory.getPosition(0.0)[0]);
  // Start at rest and end at rest
  EXPECT_NEAR(0.0, trajectory.getVelocity(0.0)[0], 0.1);
  EXPECT_NEAR(0.0, trajectory.getVelocity(traj_duration)[0], 0.1);

  // Check vels and accels at all points
  for (double time = 0; time < traj_duration; time += 0.01)
  {
    // This trajectory has a single switching point
    double t_switch = 0.1603407;
    if (time < t_switch)
    {
      EXPECT_NEAR(trajectory.getAcceleration(time)[0], max_accelerations[0], 1e-3) << "Time: " << time;
    }
    else if (time > t_switch)
    {
      EXPECT_NEAR(trajectory.getAcceleration(time)[0], -max_accelerations[0], 1e-3) << "Time: " << time;
    }
  }
}

TEST(time_optimal_trajectory_generation, testRelevantZeroMaxAccelerationsInvalidateTrajectory)
{
  const Eigen::Vector2d max_velocity(1, 1);
  const Path path = *Path::create({ Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 1) });

  EXPECT_FALSE(Trajectory::create(path, max_velocity, /*max_acceleration=*/Eigen::Vector2d(0, 1)));
  EXPECT_FALSE(Trajectory::create(path, max_velocity, /*max_acceleration=*/Eigen::Vector2d(1, 0)));
  EXPECT_FALSE(Trajectory::create(path, max_velocity, /*max_acceleration=*/Eigen::Vector2d(0, 0)));
}

TEST(time_optimal_trajectory_generation, testIrrelevantZeroMaxAccelerationsDontInvalidateTrajectory)
{
  const Eigen::Vector2d max_velocity(1, 1);

  EXPECT_TRUE(Trajectory::create(*Path::create({ Eigen::Vector2d(0, 0), Eigen::Vector2d(0, 1) }), max_velocity,
                                 /*max_acceleration=*/Eigen::Vector2d(0, 1)));
  EXPECT_TRUE(Trajectory::create(*Path::create({ Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 0) }), max_velocity,
                                 /*max_acceleration=*/Eigen::Vector2d(1, 0)));
}

TEST(time_optimal_trajectory_generation, testTimeStepZeroMakesTrajectoryInvalid)
{
  EXPECT_FALSE(Trajectory::create(*Path::create({ Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 1) }),
                                  /*max_velocity=*/Eigen::Vector2d(1, 1), /*max_acceleration=*/Eigen::Vector2d(1, 1),
                                  /*time_step=*/0));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
