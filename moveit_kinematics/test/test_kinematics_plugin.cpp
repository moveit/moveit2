/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Southwest Research Institute
 *                2018, Bielefeld University
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Jorge Nicho, Robert Haschke */

#include <gtest/gtest.h>
#include <memory>
#include <functional>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

// MoveIt
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.h>

#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <moveit/utils/robot_model_test_utils.h>
#include <moveit/utils/logger.hpp>

rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.kinematics.test_kinematics_plugin");
}
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
const double DEFAULT_SEARCH_DISCRETIZATION = 0.01f;
const double EXPECTED_SUCCESS_RATE = 0.8;
static const std::string UNDEFINED = "<undefined>";

// As loading of parameters is quite slow, we share them across all tests
class SharedData
{
  friend class KinematicsTest;
  typedef pluginlib::ClassLoader<kinematics::KinematicsBase> KinematicsLoader;

  moveit::core::RobotModelPtr robot_model_;
  std::unique_ptr<KinematicsLoader> kinematics_loader_;
  std::string root_link_;
  std::string tip_link_;
  std::string group_name_;
  std::string ik_plugin_name_;
  std::vector<std::string> joints_;
  std::vector<double> seed_;
  std::vector<double> consistency_limits_;
  double timeout_;
  double tolerance_;
  int num_fk_tests_;
  int num_ik_cb_tests_;
  int num_ik_tests_;
  int num_ik_multiple_tests_;
  int num_nearest_ik_tests_;
  bool publish_trajectory_;

  SharedData(const SharedData&) = delete;  // this is a singleton
  SharedData()
  {
    initialize();
  }

  void initialize()
  {
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node_ = rclcpp::Node::make_shared("moveit_kinematics_test", node_options);

    RCLCPP_INFO_STREAM(getLogger(), "Loading robot model from " << node_->get_name() << '.' << ROBOT_DESCRIPTION_PARAM);
    // load robot model
    rdf_loader::RDFLoader rdf_loader(node_, ROBOT_DESCRIPTION_PARAM);
    robot_model_ = std::make_shared<moveit::core::RobotModel>(rdf_loader.getURDF(), rdf_loader.getSRDF());
    ASSERT_TRUE(bool(robot_model_)) << "Failed to load robot model";

    // init ClassLoader
    kinematics_loader_ = std::make_unique<KinematicsLoader>("moveit_core", "kinematics::KinematicsBase");
    ASSERT_TRUE(bool(kinematics_loader_)) << "Failed to instantiate ClassLoader";

    // load parameters
    ASSERT_TRUE(node_->get_parameter("group", group_name_));
    ASSERT_TRUE(node_->get_parameter("tip_link", tip_link_));
    ASSERT_TRUE(node_->get_parameter("root_link", root_link_));
    ASSERT_TRUE(node_->get_parameter("joint_names", joints_));
    node_->get_parameter_or("seed", seed_, { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
    ASSERT_TRUE(seed_.empty() || seed_.size() == joints_.size()) << "If set, 'seed' size must match 'joint_names' size";
    node_->get_parameter_or("consistency_limits", consistency_limits_, consistency_limits_);
    ASSERT_TRUE(consistency_limits_.empty() || consistency_limits_.size() == joints_.size())
        << "If set, 'consistency_limits' size must match 'joint_names' size";
    ASSERT_TRUE(node_->get_parameter("ik_timeout", timeout_));
    ASSERT_TRUE(timeout_ > 0.0) << "'ik_timeout' must be more than 0.0 seconds";
    ASSERT_TRUE(node_->get_parameter("tolerance", tolerance_));
    ASSERT_TRUE(tolerance_ > 0.0) << "'tolerance' must be greater than 0.0";
    ASSERT_TRUE(node_->get_parameter("num_fk_tests", num_fk_tests_));
    ASSERT_TRUE(node_->get_parameter("num_ik_cb_tests", num_ik_cb_tests_));
    ASSERT_TRUE(node_->get_parameter("num_ik_tests", num_ik_tests_));
    ASSERT_TRUE(node_->get_parameter("num_ik_multiple_tests", num_ik_multiple_tests_));
    ASSERT_TRUE(node_->get_parameter("num_nearest_ik_tests", num_nearest_ik_tests_));
    ASSERT_TRUE(node_->get_parameter("ik_plugin_name", ik_plugin_name_));
    node_->get_parameter_or("publish_trajectory", publish_trajectory_, false);

    ASSERT_TRUE(robot_model_->hasJointModelGroup(group_name_));
    ASSERT_TRUE(robot_model_->hasLinkModel(root_link_));
    ASSERT_TRUE(robot_model_->hasLinkModel(tip_link_));
  }

public:
  std::shared_ptr<rclcpp::Node> node_;

  auto createUniqueInstance(const std::string& name) const
  {
    return kinematics_loader_->createUniqueInstance(name);
  }

  static const SharedData& instance()
  {
    static SharedData instance;
    return instance;
  }
  static void release()
  {
    SharedData& shared = const_cast<SharedData&>(instance());
    shared.kinematics_loader_.reset();
  }
};

class KinematicsTest : public ::testing::Test
{
protected:
  void operator=(const SharedData& data)
  {
    node_ = data.node_;
    robot_model_ = data.robot_model_;
    jmg_ = robot_model_->getJointModelGroup(data.group_name_);
    root_link_ = data.root_link_;
    tip_link_ = data.tip_link_;
    group_name_ = data.group_name_;
    ik_plugin_name_ = data.ik_plugin_name_;
    joints_ = data.joints_;
    seed_ = data.seed_;
    consistency_limits_ = data.consistency_limits_;
    timeout_ = data.timeout_;
    tolerance_ = data.tolerance_;
    num_fk_tests_ = data.num_fk_tests_;
    num_ik_cb_tests_ = data.num_ik_cb_tests_;
    num_ik_tests_ = data.num_ik_tests_;
    num_ik_multiple_tests_ = data.num_ik_multiple_tests_;
    num_nearest_ik_tests_ = data.num_nearest_ik_tests_;
    publish_trajectory_ = data.publish_trajectory_;
  }

  void SetUp() override
  {
    *this = SharedData::instance();

    RCLCPP_INFO_STREAM(getLogger(), "Loading " << ik_plugin_name_);
    kinematics_solver_ = SharedData::instance().createUniqueInstance(ik_plugin_name_);
    ASSERT_TRUE(bool(kinematics_solver_)) << "Failed to load plugin: " << ik_plugin_name_;

    // initializing plugin
    ASSERT_TRUE(kinematics_solver_->initialize(node_, *robot_model_, group_name_, root_link_, { tip_link_ },
                                               DEFAULT_SEARCH_DISCRETIZATION))
        << "Solver failed to initialize";
    jmg_ = robot_model_->getJointModelGroup(kinematics_solver_->getGroupName());
    ASSERT_TRUE(jmg_);
    // Validate chain information
    ASSERT_EQ(root_link_, kinematics_solver_->getBaseFrame());
    ASSERT_FALSE(kinematics_solver_->getTipFrames().empty());
    ASSERT_EQ(tip_link_, kinematics_solver_->getTipFrame());

    ASSERT_EQ(joints_, kinematics_solver_->getJointNames());
  }

public:
  testing::AssertionResult isNear(const char* expr1, const char* expr2, const char* /*abs_error_expr*/,
                                  const geometry_msgs::msg::Point& val1, const geometry_msgs::msg::Point& val2,
                                  double abs_error)
  {
    // clang-format off
    if (std::fabs(val1.x - val2.x) <= abs_error &&
        std::fabs(val1.y - val2.y) <= abs_error &&
        std::fabs(val1.z - val2.z) <= abs_error)
      return testing::AssertionSuccess();

    return testing::AssertionFailure()
        << std::setprecision(std::numeric_limits<double>::digits10 + 2)
        << "Expected: " << expr1 << " [" << val1.x << ", " << val1.y << ", " << val1.z << "]\n"
        << "Actual: " << expr2 << " [" << val2.x << ", " << val2.y << ", " << val2.z << ']';
    // clang-format on
  }
  testing::AssertionResult isNear(const char* expr1, const char* expr2, const char* /*abs_error_expr*/,
                                  const geometry_msgs::msg::Quaternion& val1,
                                  const geometry_msgs::msg::Quaternion& val2, double abs_error)
  {
    if ((std::fabs(val1.x - val2.x) <= abs_error && std::fabs(val1.y - val2.y) <= abs_error &&
         std::fabs(val1.z - val2.z) <= abs_error && std::fabs(val1.w - val2.w) <= abs_error) ||
        (std::fabs(val1.x + val2.x) <= abs_error && std::fabs(val1.y + val2.y) <= abs_error &&
         std::fabs(val1.z + val2.z) <= abs_error && std::fabs(val1.w + val2.w) <= abs_error))
      return testing::AssertionSuccess();

    // clang-format off
    return testing::AssertionFailure()
        << std::setprecision(std::numeric_limits<double>::digits10 + 2)
        << "Expected: " << expr1 << " [" << val1.w << ", " << val1.x << ", " << val1.y << ", " << val1.z << "]\n"
        << "Actual: " << expr2 << " [" << val2.w << ", " << val2.x << ", " << val2.y << ", " << val2.z << ']';
    // clang-format on
  }
  testing::AssertionResult expectNearHelper(const char* expr1, const char* expr2, const char* abs_error_expr,
                                            const std::vector<geometry_msgs::msg::Pose>& val1,
                                            const std::vector<geometry_msgs::msg::Pose>& val2, double abs_error)
  {
    if (val1.size() != val2.size())
    {
      return testing::AssertionFailure() << "Different vector sizes"
                                         << "\nExpected: " << expr1 << " (" << val1.size() << ')'
                                         << "\nActual: " << expr2 << " (" << val2.size() << ')';
    }

    for (size_t i = 0; i < val1.size(); ++i)
    {
      ::std::stringstream ss;
      ss << '[' << i << "].position";
      GTEST_ASSERT_(isNear((expr1 + ss.str()).c_str(), (expr2 + ss.str()).c_str(), abs_error_expr, val1[i].position,
                           val2[i].position, abs_error),
                    GTEST_NONFATAL_FAILURE_);

      ss.str("");
      ss << '[' << i << "].orientation";
      GTEST_ASSERT_(isNear((expr1 + ss.str()).c_str(), (expr2 + ss.str()).c_str(), abs_error_expr, val1[i].orientation,
                           val2[i].orientation, abs_error),
                    GTEST_NONFATAL_FAILURE_);
    }
    return testing::AssertionSuccess();
  }

  void searchIKCallback(const std::vector<double>& joint_state, moveit_msgs::msg::MoveItErrorCodes& error_code)
  {
    std::vector<std::string> link_names = { tip_link_ };
    std::vector<geometry_msgs::msg::Pose> poses;
    if (!kinematics_solver_->getPositionFK(link_names, joint_state, poses))
    {
      error_code.val = error_code.PLANNING_FAILED;
      return;
    }

    EXPECT_GT(poses[0].position.z, 0.0f);
    if (poses[0].position.z > 0.0)
    {
      error_code.val = error_code.SUCCESS;
    }
    else
    {
      error_code.val = error_code.PLANNING_FAILED;
    }
  }

public:
  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelPtr robot_model_;
  moveit::core::JointModelGroup* jmg_;
  kinematics::KinematicsBasePtr kinematics_solver_;
  random_numbers::RandomNumberGenerator rng_{ 42 };
  std::string root_link_;
  std::string tip_link_;
  std::string group_name_;
  std::string ik_plugin_name_;
  std::vector<std::string> joints_;
  std::vector<double> seed_;
  std::vector<double> consistency_limits_;
  double timeout_;
  double tolerance_;
  unsigned int num_fk_tests_;
  unsigned int num_ik_cb_tests_;
  unsigned int num_ik_tests_;
  unsigned int num_ik_multiple_tests_;
  unsigned int num_nearest_ik_tests_;
  bool publish_trajectory_;
};

#define EXPECT_NEAR_POSES(lhs, rhs, near)                                                                              \
  SCOPED_TRACE("EXPECT_NEAR_POSES(" #lhs ", " #rhs ")");                                                               \
  GTEST_ASSERT_(expectNearHelper(#lhs, #rhs, #near, lhs, rhs, near), GTEST_NONFATAL_FAILURE_);

TEST_F(KinematicsTest, getFK)
{
  std::vector<double> joints(kinematics_solver_->getJointNames().size(), 0.0);
  const std::vector<std::string>& tip_frames = kinematics_solver_->getTipFrames();
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();

  for (unsigned int i = 0; i < num_fk_tests_; ++i)
  {
    robot_state.setToRandomPositions(jmg_, this->rng_);
    robot_state.copyJointGroupPositions(jmg_, joints);
    std::vector<geometry_msgs::msg::Pose> fk_poses;
    EXPECT_TRUE(kinematics_solver_->getPositionFK(tip_frames, joints, fk_poses));

    robot_state.updateLinkTransforms();
    std::vector<geometry_msgs::msg::Pose> model_poses;
    model_poses.reserve(tip_frames.size());
    for (const auto& tip : tip_frames)
      model_poses.emplace_back(tf2::toMsg(robot_state.getGlobalLinkTransform(tip)));
    EXPECT_NEAR_POSES(model_poses, fk_poses, tolerance_);
  }
}

// perform random walk in joint-space, reaching poses via IK
TEST_F(KinematicsTest, randomWalkIK)
{
  std::vector<double> seed, goal, solution;
  const std::vector<std::string>& tip_frames = kinematics_solver_->getTipFrames();
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();

  if (!seed_.empty())
    robot_state.setJointGroupPositions(jmg_, seed_);

  moveit_msgs::msg::DisplayTrajectory msg;
  msg.model_id = robot_model_->getName();
  moveit::core::robotStateToRobotStateMsg(robot_state, msg.trajectory_start);
  msg.trajectory.resize(1);
  robot_trajectory::RobotTrajectory traj(robot_model_, jmg_);

  unsigned int failures = 0;
  static constexpr double NEAR_JOINT = 0.1;
  const std::vector<double> consistency_limits(jmg_->getVariableCount(), 1.05 * NEAR_JOINT);
  for (unsigned int i = 0; i < num_ik_tests_; ++i)
  {
    // remember previous pose
    robot_state.copyJointGroupPositions(jmg_, seed);
    // sample a new pose nearby
    robot_state.setToRandomPositionsNearBy(jmg_, robot_state, NEAR_JOINT);
    // get joints of new pose
    robot_state.copyJointGroupPositions(jmg_, goal);
    // compute target tip_frames
    std::vector<geometry_msgs::msg::Pose> poses;
    ASSERT_TRUE(kinematics_solver_->getPositionFK(tip_frames, goal, poses));

    // compute IK
    moveit_msgs::msg::MoveItErrorCodes error_code;
    kinematics_solver_->searchPositionIK(poses[0], seed, 0.1, consistency_limits, solution, error_code);
    if (error_code.val != error_code.SUCCESS)
    {
      ++failures;
      continue;
    }

    // on success: validate reached poses
    std::vector<geometry_msgs::msg::Pose> reached_poses;
    kinematics_solver_->getPositionFK(tip_frames, solution, reached_poses);
    EXPECT_NEAR_POSES(poses, reached_poses, tolerance_);

    // validate closeness of solution pose to goal
    auto diff = Eigen::Map<Eigen::ArrayXd>(solution.data(), solution.size()) -
                Eigen::Map<Eigen::ArrayXd>(goal.data(), goal.size());
    if (!diff.isZero(1.05 * NEAR_JOINT))
    {
      ++failures;
      RCLCPP_WARN_STREAM(getLogger(), "jump in [" << i << "]: " << diff.transpose());
    }

    // update robot state to found pose
    robot_state.setJointGroupPositions(jmg_, solution);
    traj.addSuffixWayPoint(robot_state, 0.1);
  }
  EXPECT_LE(failures, (1.0 - EXPECTED_SUCCESS_RATE) * num_ik_tests_);
  if (publish_trajectory_)
  {
    auto pub = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>("display_random_walk", 1);
    traj.getRobotTrajectoryMsg(msg.trajectory[0]);
    pub->publish(msg);
    rclcpp::spin_some(node_);
  }
}

static bool parsePose(const std::vector<double>& pose_values, Eigen::Isometry3d& goal)
{
  std::vector<double> vec;
  Eigen::Quaterniond q;
  if (pose_values.size() == 6)
  {
    q = Eigen::AngleAxisd(pose_values[3], Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(pose_values[4], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(pose_values[5], Eigen::Vector3d::UnitZ());
  }
  else if (pose_values.size() == 7)
  {
    q = Eigen::Quaterniond(pose_values[3], pose_values[4], pose_values[5], pose_values[6]);
  }
  else
  {
    return false;
  }

  goal = q;
  goal.translation() = Eigen::Vector3d(pose_values[0], pose_values[1], pose_values[2]);

  return true;
}

TEST_F(KinematicsTest, unitIK)
{
  static const std::string TEST_POSES_PARAM = "unit_test_poses";
  size_t expected_test_poses = 0;
  node_->get_parameter_or(TEST_POSES_PARAM + ".size", expected_test_poses, expected_test_poses);

  std::vector<double> sol;
  const std::vector<std::string>& tip_frames = kinematics_solver_->getTipFrames();
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  robot_state.setJointGroupPositions(jmg_, seed_);

  // compute initial end-effector pose
  std::vector<geometry_msgs::msg::Pose> poses;
  ASSERT_TRUE(kinematics_solver_->getPositionFK(tip_frames, seed_, poses));
  Eigen::Isometry3d initial, goal;
  tf2::fromMsg(poses[0], initial);

  RCLCPP_DEBUG(getLogger(), "Initial: %f %f %f %f %f %f %f\n", poses[0].position.x, poses[0].position.y,
               poses[0].position.z, poses[0].orientation.x, poses[0].orientation.y, poses[0].orientation.z,
               poses[0].orientation.w);

  auto validate_ik = [&](const geometry_msgs::msg::Pose& goal, std::vector<double>& truth) {
    // compute IK
    moveit_msgs::msg::MoveItErrorCodes error_code;

    RCLCPP_DEBUG(getLogger(), "Goal %f %f %f %f %f %f %f\n", goal.position.x, goal.position.y, goal.position.z,
                 goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w);

    kinematics_solver_->searchPositionIK(goal, seed_, timeout_,
                                         const_cast<const std::vector<double>&>(consistency_limits_), sol, error_code);
    ASSERT_EQ(error_code.val, error_code.SUCCESS);

    // validate reached poses
    std::vector<geometry_msgs::msg::Pose> reached_poses;
    kinematics_solver_->getPositionFK(tip_frames, sol, reached_poses);
    EXPECT_NEAR_POSES({ goal }, reached_poses, tolerance_);

    // validate ground truth
    if (!truth.empty())
    {
      ASSERT_EQ(truth.size(), sol.size()) << "Invalid size of ground truth joints vector";
      Eigen::Map<Eigen::ArrayXd> solution(sol.data(), sol.size());
      Eigen::Map<Eigen::ArrayXd> ground_truth(truth.data(), truth.size());
      EXPECT_TRUE(solution.isApprox(ground_truth, 10 * tolerance_)) << solution.transpose() << '\n'
                                                                    << ground_truth.transpose() << '\n';
    }
  };

  std::vector<double> ground_truth, pose_values;
  constexpr char pose_type_relative[] = "relative";
  constexpr char pose_type_absolute[] = "absolute";

  // process tests definitions on parameter server of the form
  //   pose_1:
  //     pose:   [0.3, 0.2, 0.1, 0, 0, 0] // xyzrpy (position + euler angles)
  //     joints: [0, 0, 0, 0, 0, 0]  // ground truth solution
  //     type: relative  // pose applied relative to current pose
  //   pose_2:
  //     pose:   [0.1, 0.2, 0.3, 0, 0, 0, 0] // xyzwxyz (position + quaternion)
  //     joints: [1, 2, 3, 4, 5, 6]  // pose applied absolute in planning frame
  //     type: absolute

  for (size_t i = 0; i < expected_test_poses; ++i)  // NOLINT(modernize-loop-convert)
  {
    const std::string pose_name = "pose_" + std::to_string(i);
    const std::string pose_param = TEST_POSES_PARAM + "." + pose_name;  // NOLINT
    goal = initial;                                                     // reset goal to initial
    ground_truth.clear();

    node_->get_parameter_or(pose_param + ".joints", ground_truth, ground_truth);
    if (!ground_truth.empty())
    {
      ASSERT_EQ(ground_truth.size(), joints_.size())
          << "Test pose '" << pose_name << "' has invalid 'joints' vector size";
    }

    pose_values.clear();
    node_->get_parameter_or(pose_param + ".pose", pose_values, pose_values);
    ASSERT_TRUE(pose_values.size() == 6 || pose_values.size() == 7)
        << "Test pose '" << pose_name << "' has invalid 'pose' vector size";

    Eigen::Isometry3d pose;
    ASSERT_TRUE(parsePose(pose_values, pose)) << "Failed to parse 'pose' vector in: " << pose_name;
    std::string pose_type = "pose_type_relative";
    node_->get_parameter_or(pose_param + ".type", pose_type, pose_type);
    if (pose_type == pose_type_relative)
    {
      goal = goal * pose;
    }
    else if (pose_type == pose_type_absolute)
    {
      goal = pose;
    }
    else
    {
      FAIL() << "Found invalid 'type' in " << pose_name << ": should be one of '" << pose_type_relative << "' or '"
             << pose_type_absolute << '\'';
    }

    std::string desc;
    {
      SCOPED_TRACE(desc);
      validate_ik(tf2::toMsg(goal), ground_truth);
    }
  }
}

TEST_F(KinematicsTest, searchIK)
{
  std::vector<double> seed, fk_values, solution;
  moveit_msgs::msg::MoveItErrorCodes error_code;
  solution.resize(kinematics_solver_->getJointNames().size(), 0.0);
  const std::vector<std::string>& fk_names = kinematics_solver_->getTipFrames();
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();

  unsigned int success = 0;
  for (unsigned int i = 0; i < num_ik_tests_; ++i)
  {
    seed.resize(kinematics_solver_->getJointNames().size(), 0.0);
    fk_values.resize(kinematics_solver_->getJointNames().size(), 0.0);
    robot_state.setToRandomPositions(jmg_, this->rng_);
    robot_state.copyJointGroupPositions(jmg_, fk_values);
    std::vector<geometry_msgs::msg::Pose> poses;
    ASSERT_TRUE(kinematics_solver_->getPositionFK(fk_names, fk_values, poses));

    kinematics_solver_->searchPositionIK(poses[0], seed, timeout_, solution, error_code);
    if (error_code.val == error_code.SUCCESS)
    {
      success++;
    }
    else
    {
      continue;
    }

    std::vector<geometry_msgs::msg::Pose> reached_poses;
    kinematics_solver_->getPositionFK(fk_names, solution, reached_poses);
    EXPECT_NEAR_POSES(poses, reached_poses, tolerance_);
  }

  if (num_ik_cb_tests_ > 0)
  {
    RCLCPP_INFO_STREAM(getLogger(), "Success Rate: " << static_cast<double>(success) / num_ik_tests_);
  }
  EXPECT_GE(success, EXPECTED_SUCCESS_RATE * num_ik_tests_);
}

TEST_F(KinematicsTest, searchIKWithCallback)
{
  std::vector<double> seed, fk_values, solution;
  moveit_msgs::msg::MoveItErrorCodes error_code;
  solution.resize(kinematics_solver_->getJointNames().size(), 0.0);
  const std::vector<std::string>& fk_names = kinematics_solver_->getTipFrames();
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();

  unsigned int success = 0;
  for (unsigned int i = 0; i < num_ik_cb_tests_; ++i)
  {
    seed.resize(kinematics_solver_->getJointNames().size(), 0.0);
    fk_values.resize(kinematics_solver_->getJointNames().size(), 0.0);
    robot_state.setToRandomPositions(jmg_, this->rng_);
    robot_state.copyJointGroupPositions(jmg_, fk_values);
    std::vector<geometry_msgs::msg::Pose> poses;
    ASSERT_TRUE(kinematics_solver_->getPositionFK(fk_names, fk_values, poses));
    if (poses[0].position.z <= 0.0f)
    {
      --i;  // draw a new random state
      continue;
    }

    kinematics_solver_->searchPositionIK(
        poses[0], fk_values, timeout_, solution,
        [this](const geometry_msgs::msg::Pose&, const std::vector<double>& joints,
               moveit_msgs::msg::MoveItErrorCodes& error_code) { searchIKCallback(joints, error_code); },
        error_code);
    if (error_code.val == error_code.SUCCESS)
    {
      success++;
    }
    else
    {
      continue;
    }

    std::vector<geometry_msgs::msg::Pose> reached_poses;
    kinematics_solver_->getPositionFK(fk_names, solution, reached_poses);
    EXPECT_NEAR_POSES(poses, reached_poses, tolerance_);
  }

  if (num_ik_cb_tests_ > 0)
  {
    RCLCPP_INFO_STREAM(getLogger(), "Success Rate: " << static_cast<double>(success) / num_ik_cb_tests_);
  }
  EXPECT_GE(success, EXPECTED_SUCCESS_RATE * num_ik_cb_tests_);
}

TEST_F(KinematicsTest, getIK)
{
  std::vector<double> fk_values, solution;
  moveit_msgs::msg::MoveItErrorCodes error_code;
  solution.resize(kinematics_solver_->getJointNames().size(), 0.0);
  const std::vector<std::string>& fk_names = kinematics_solver_->getTipFrames();
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();

  for (unsigned int i = 0; i < num_ik_tests_; ++i)
  {
    fk_values.resize(kinematics_solver_->getJointNames().size(), 0.0);
    robot_state.setToRandomPositions(jmg_, this->rng_);
    robot_state.copyJointGroupPositions(jmg_, fk_values);
    std::vector<geometry_msgs::msg::Pose> poses;

    ASSERT_TRUE(kinematics_solver_->getPositionFK(fk_names, fk_values, poses));
    kinematics_solver_->getPositionIK(poses[0], fk_values, solution, error_code);
    // starting from the correct solution, should yield the same pose
    EXPECT_EQ(error_code.val, error_code.SUCCESS);

    Eigen::Map<Eigen::ArrayXd> sol(solution.data(), solution.size());
    Eigen::Map<Eigen::ArrayXd> truth(fk_values.data(), fk_values.size());
    EXPECT_TRUE(sol.isApprox(truth, tolerance_)) << sol.transpose() << '\n' << truth.transpose() << '\n';
  }
}

TEST_F(KinematicsTest, getIKMultipleSolutions)
{
  std::vector<double> seed, fk_values;
  std::vector<std::vector<double>> solutions;
  kinematics::KinematicsQueryOptions options;
  kinematics::KinematicsResult result;

  const std::vector<std::string>& fk_names = kinematics_solver_->getTipFrames();
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();

  unsigned int success = 0;
  for (unsigned int i = 0; i < num_ik_multiple_tests_; ++i)
  {
    seed.resize(kinematics_solver_->getJointNames().size(), 0.0);
    fk_values.resize(kinematics_solver_->getJointNames().size(), 0.0);
    robot_state.setToRandomPositions(jmg_, this->rng_);
    robot_state.copyJointGroupPositions(jmg_, fk_values);
    std::vector<geometry_msgs::msg::Pose> poses;
    ASSERT_TRUE(kinematics_solver_->getPositionFK(fk_names, fk_values, poses));

    solutions.clear();
    kinematics_solver_->getPositionIK(poses, fk_values, solutions, result, options);

    if (result.kinematic_error == kinematics::KinematicErrors::OK)
    {
      success += solutions.empty() ? 0 : 1;
    }
    else
    {
      continue;
    }

    std::vector<geometry_msgs::msg::Pose> reached_poses;
    for (const auto& s : solutions)
    {
      kinematics_solver_->getPositionFK(fk_names, s, reached_poses);
      EXPECT_NEAR_POSES(poses, reached_poses, tolerance_);
    }
  }

  if (num_ik_cb_tests_ > 0)
  {
    RCLCPP_INFO_STREAM(getLogger(), "Success Rate: " << static_cast<double>(success) / num_ik_multiple_tests_);
  }
  EXPECT_GE(success, EXPECTED_SUCCESS_RATE * num_ik_multiple_tests_);
}

// validate that getPositionIK() retrieves closest solution to seed
TEST_F(KinematicsTest, getNearestIKSolution)
{
  std::vector<std::vector<double>> solutions;
  kinematics::KinematicsQueryOptions options;
  kinematics::KinematicsResult result;

  std::vector<double> seed, fk_values, solution;
  moveit_msgs::msg::MoveItErrorCodes error_code;
  const std::vector<std::string>& fk_names = kinematics_solver_->getTipFrames();
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();

  for (unsigned int i = 0; i < num_nearest_ik_tests_; ++i)
  {
    robot_state.setToRandomPositions(jmg_, this->rng_);
    robot_state.copyJointGroupPositions(jmg_, fk_values);
    std::vector<geometry_msgs::msg::Pose> poses;
    ASSERT_TRUE(kinematics_solver_->getPositionFK(fk_names, fk_values, poses));

    // sample seed vector
    robot_state.setToRandomPositions(jmg_, this->rng_);
    robot_state.copyJointGroupPositions(jmg_, seed);

    // getPositionIK for single solution
    kinematics_solver_->getPositionIK(poses[0], seed, solution, error_code);

    // check if getPositionIK call for single solution returns solution
    if (error_code.val != error_code.SUCCESS)
      continue;

    const Eigen::Map<const Eigen::VectorXd> seed_eigen(seed.data(), seed.size());
    double error_get_ik =
        (Eigen::Map<const Eigen::VectorXd>(solution.data(), solution.size()) - seed_eigen).array().abs().sum();

    // getPositionIK for multiple solutions
    solutions.clear();
    kinematics_solver_->getPositionIK(poses, seed, solutions, result, options);

    // check if getPositionIK call for multiple solutions returns solution
    EXPECT_EQ(result.kinematic_error, kinematics::KinematicErrors::OK)
        << "Multiple solution call failed, while single solution call succeeded";
    if (result.kinematic_error != kinematics::KinematicErrors::OK)
      continue;

    double smallest_error_multiple_ik = std::numeric_limits<double>::max();
    for (const auto& s : solutions)
    {
      double error_multiple_ik =
          (Eigen::Map<const Eigen::VectorXd>(s.data(), s.size()) - seed_eigen).array().abs().sum();
      if (error_multiple_ik <= smallest_error_multiple_ik)
        smallest_error_multiple_ik = error_multiple_ik;
    }
    EXPECT_NEAR(smallest_error_multiple_ik, error_get_ik, tolerance_);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  SharedData::release();  // avoid class_loader::LibraryUnloadException
  return result;
}
