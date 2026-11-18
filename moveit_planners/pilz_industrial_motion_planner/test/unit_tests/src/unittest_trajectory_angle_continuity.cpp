// Copyright 2026 OC Labs
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>
#include <pilz_industrial_motion_planner/trajectory_functions.hpp>
#include <moveit/kinematics_base/kinematics_base.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <cmath>

namespace
{
namespace pilz = pilz_industrial_motion_planner;
constexpr double TWO_PI = 2.0 * M_PI;

// Deliberately returns a canonical-angle branch from searchPositionIK. This is
// a valid pose solution, but can differ from the seed by a complete revolution.
class CanonicalIK : public kinematics::KinematicsBase
{
public:
  CanonicalIK(const moveit::core::JointModelGroup* group, bool prismatic, double turns)
    : prismatic_(prismatic), turns_(turns)
  {
    storeValues(group->getParentModel(), group->getName(), "base", { "tool" }, 0.0);
  }

  bool getPositionIK(const geometry_msgs::msg::Pose& pose, const std::vector<double>& seed,
                     std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error,
                     const kinematics::KinematicsQueryOptions&) const override
  {
    solve(pose, solution, error, {});
    if (!prismatic_)
      solution[0] += TWO_PI * std::round((seed[0] - solution[0]) / TWO_PI);
    return true;
  }

  bool searchPositionIK(const geometry_msgs::msg::Pose& pose, const std::vector<double>&, double,
                        std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error,
                        const kinematics::KinematicsQueryOptions&) const override
  {
    return solve(pose, solution, error, {});
  }

  bool searchPositionIK(const geometry_msgs::msg::Pose& pose, const std::vector<double>&, double,
                        const std::vector<double>&, std::vector<double>& solution,
                        moveit_msgs::msg::MoveItErrorCodes& error,
                        const kinematics::KinematicsQueryOptions&) const override
  {
    return solve(pose, solution, error, {});
  }

  bool searchPositionIK(const geometry_msgs::msg::Pose& pose, const std::vector<double>&, double,
                        std::vector<double>& solution, const IKCallbackFn& callback,
                        moveit_msgs::msg::MoveItErrorCodes& error,
                        const kinematics::KinematicsQueryOptions&) const override
  {
    return solve(pose, solution, error, callback);
  }

  bool searchPositionIK(const geometry_msgs::msg::Pose& pose, const std::vector<double>&, double,
                        const std::vector<double>&, std::vector<double>& solution, const IKCallbackFn& callback,
                        moveit_msgs::msg::MoveItErrorCodes& error,
                        const kinematics::KinematicsQueryOptions&) const override
  {
    return solve(pose, solution, error, callback);
  }

  bool getPositionFK(const std::vector<std::string>& links, const std::vector<double>& joints,
                     std::vector<geometry_msgs::msg::Pose>& poses) const override
  {
    poses.resize(links.size());
    for (auto& pose : poses)
    {
      pose.orientation.w = prismatic_ ? 1.0 : std::cos(joints[0] / 2.0);
      pose.orientation.z = prismatic_ ? 0.0 : std::sin(joints[0] / 2.0);
      pose.position.x = prismatic_ ? joints[0] : 0.0;
    }
    return true;
  }

  const std::vector<std::string>& getJointNames() const override
  {
    return joints_;
  }
  const std::vector<std::string>& getLinkNames() const override
  {
    return links_;
  }

private:
  bool solve(const geometry_msgs::msg::Pose& pose, std::vector<double>& solution,
             moveit_msgs::msg::MoveItErrorCodes& error, const IKCallbackFn& callback) const
  {
    const auto& q = pose.orientation;
    solution = { prismatic_ ?
                     pose.position.x :
                     std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)) + TWO_PI * turns_ };
    error.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    if (callback)
      callback(pose, solution, error);
    return error.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }
  bool prismatic_;
  double turns_;
  const std::vector<std::string> joints_{ "joint" };
  const std::vector<std::string> links_{ "tool" };
};

class AngleContinuity : public testing::Test
{
protected:
  void model(const std::string& type = "continuous", double lower = -10, double upper = 10, bool mimic = false,
             double solver_turns = 0, bool collision_geometry = false)
  {
    prismatic_ = type == "prismatic";
    std::string urdf = "<robot name='angle_test'><link name='base'/><link name='tool'/>"
                       "<joint name='joint' type='" +
                       type +
                       "'><parent link='base'/><child link='tool'/>"
                       "<axis xyz='" +
                       (prismatic_ ? std::string("1 0 0") : std::string("0 0 1")) +
                       "'/>"
                       "<limit lower='" +
                       std::to_string(lower) + "' upper='" + std::to_string(upper) +
                       "' velocity='100' effort='100'/></joint>";
    if (mimic)
      urdf += "<link name='mimic_tool'/><joint name='mimic_joint' type='continuous'>"
              "<parent link='base'/><child link='mimic_tool'/><axis xyz='0 0 1'/>"
              "<mimic joint='joint' multiplier='0.5'/></joint>";
    if (collision_geometry)
    {
      const auto tool = urdf.find("<link name='tool'/>");
      urdf.replace(tool, std::string("<link name='tool'/>").size(),
                   "<link name='tool'><collision><origin xyz='1 0 0'/>"
                   "<geometry><sphere radius='0.1'/></geometry></collision></link>");
      urdf += "<link name='blocker'><collision><geometry><sphere radius='0.1'/></geometry>"
              "</collision></link><joint name='blocker_joint' type='fixed'><parent link='base'/>"
              "<child link='blocker'/><origin xyz='-1 0 0'/></joint>";
    }
    urdf += "</robot>";
    auto description = urdf::parseURDF(urdf);
    ASSERT_TRUE(description);
    auto srdf = std::make_shared<srdf::Model>();
    ASSERT_TRUE(srdf->initString(*description, "<robot name='angle_test'><group name='arm'>"
                                               "<chain base_link='base' tip_link='tool'/></group></robot>"));
    model_ = std::make_shared<moveit::core::RobotModel>(description, srdf);
    model_->getJointModelGroup("arm")->setSolverAllocators(
        [this, solver_turns](const moveit::core::JointModelGroup* group) {
          return std::make_shared<CanonicalIK>(group, prismatic_, solver_turns);
        });
    scene_ = std::make_shared<planning_scene::PlanningScene>(model_);
    pilz::JointLimit limit;
    limit.has_velocity_limits = true;
    limit.max_velocity = 1.0;
    limit.has_acceleration_limits = true;
    limit.max_acceleration = 10.0;
    limit.has_deceleration_limits = true;
    limit.max_deceleration = -10.0;
    ASSERT_TRUE(limits_.addLimit("joint", limit));
  }

  void positionLimits(double lower, double upper)
  {
    auto limit = limits_.getLimit("joint");
    limit.has_position_limits = true;
    limit.min_position = lower;
    limit.max_position = upper;
    limits_ = pilz::JointLimitsContainer();
    ASSERT_TRUE(limits_.addLimit("joint", limit));
  }

  pilz::CartesianTrajectory path(const std::vector<double>& positions)
  {
    pilz::CartesianTrajectory result;
    for (size_t i = 0; i < positions.size(); ++i)
    {
      pilz::CartesianTrajectoryPoint point;
      point.pose.orientation.w = prismatic_ ? 1.0 : std::cos(positions[i] / 2.0);
      point.pose.orientation.z = prismatic_ ? 0.0 : std::sin(positions[i] / 2.0);
      point.pose.position.x = prismatic_ ? positions[i] : 0.0;
      point.time_from_start = rclcpp::Duration::from_seconds(0.1 * (i + 1));
      result.points.push_back(point);
    }
    return result;
  }

  bool generate(double initial, const std::vector<double>& positions, bool collision = false)
  {
    return pilz::generateJointTrajectory(scene_, limits_, path(positions), "arm", "tool", { { "joint", initial } },
                                         { { "joint", 0.0 } }, output_, error_, collision);
  }

  void expectPositionsAndFK(const std::vector<double>& expected)
  {
    ASSERT_EQ(output_.points.size(), expected.size());
    moveit::core::RobotState actual(model_), desired(model_);
    actual.setToDefaultValues();
    desired.setToDefaultValues();
    for (size_t i = 0; i < expected.size(); ++i)
    {
      ASSERT_EQ(output_.points[i].positions.size(), 1u);
      EXPECT_NEAR(output_.points[i].positions[0], expected[i], 1e-10);
      actual.setVariablePosition("joint", output_.points[i].positions[0]);
      desired.setVariablePosition("joint", expected[i]);
      EXPECT_TRUE(actual.getGlobalLinkTransform("tool").matrix().isApprox(
          desired.getGlobalLinkTransform("tool").matrix(), 1e-10));
      EXPECT_LE(std::abs(output_.points[i].velocities[0]), 1.0);
    }
  }

  bool prismatic_ = false;
  moveit::core::RobotModelPtr model_;
  planning_scene::PlanningScenePtr scene_;
  pilz::JointLimitsContainer limits_;
  trajectory_msgs::msg::JointTrajectory output_;
  moveit_msgs::msg::MoveItErrorCodes error_;
};

TEST_F(AngleContinuity, PositiveWrapPreservesPose)
{
  model();
  ASSERT_TRUE(generate(3.11, { 3.13, 3.15, 3.17 }, true));
  expectPositionsAndFK({ 3.13, 3.15, 3.17 });
}

TEST_F(AngleContinuity, NegativeWrapPreservesPose)
{
  model();
  ASSERT_TRUE(generate(-3.11, { -3.13, -3.15, -3.17 }));
  expectPositionsAndFK({ -3.13, -3.15, -3.17 });
}

TEST_F(AngleContinuity, SelfCollisionStillRejectsIK)
{
  model("continuous", -10, 10, false, 0, true);
  EXPECT_FALSE(generate(3.11, { 3.13, 3.15, 3.17 }, true));
  EXPECT_EQ(error_.val, moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION);
  EXPECT_TRUE(output_.points.empty());
  ASSERT_TRUE(generate(3.11, { 3.13, 3.15, 3.17 }, false));
  expectPositionsAndFK({ 3.13, 3.15, 3.17 });
}

TEST_F(AngleContinuity, MultiTurnSeed)
{
  model();
  ASSERT_TRUE(generate(3.11 + 2 * TWO_PI, { 3.13, 3.15, 3.17 }));
  expectPositionsAndFK({ 3.13 + 2 * TWO_PI, 3.15 + 2 * TWO_PI, 3.17 + 2 * TWO_PI });
}

TEST_F(AngleContinuity, WideBoundedJoint)
{
  model("revolute", -TWO_PI, TWO_PI);
  ASSERT_TRUE(generate(3.11, { 3.13, 3.15, 3.17 }));
  expectPositionsAndFK({ 3.13, 3.15, 3.17 });
}

TEST_F(AngleContinuity, MechanicalStopIsNotCrossed)
{
  model("revolute", -M_PI, M_PI);
  EXPECT_FALSE(generate(3.11, { 3.13, 3.15, 3.17 }));
  EXPECT_EQ(error_.val, moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED);
  EXPECT_TRUE(output_.points.empty());
}

TEST_F(AngleContinuity, PlannerLimitsConstrainContinuousJoint)
{
  model();
  positionLimits(-M_PI, M_PI);
  EXPECT_FALSE(generate(3.11, { 3.13, 3.15, 3.17 }));
  EXPECT_TRUE(output_.points.empty());
}

TEST_F(AngleContinuity, PlannerLimitsConstrainWideBoundedJoint)
{
  model("revolute", -TWO_PI, TWO_PI);
  positionLimits(-M_PI, M_PI);
  EXPECT_FALSE(generate(3.11, { 3.13, 3.15, 3.17 }));
}

TEST_F(AngleContinuity, GenuineVelocityExcessIsRejected)
{
  model();
  EXPECT_FALSE(generate(0.0, { 0.2 }));
  EXPECT_EQ(error_.val, moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED);
}

TEST_F(AngleContinuity, PrismaticMotionIsNotWrapped)
{
  model("prismatic", -10, 10);
  ASSERT_TRUE(generate(3.11, { 3.13, 3.15, 3.17 }));
  expectPositionsAndFK({ 3.13, 3.15, 3.17 });
  EXPECT_FALSE(generate(3.13, { 3.13 - TWO_PI }));
}

TEST_F(AngleContinuity, NoEquivalentDoesNotClampPose)
{
  model("revolute", -1, 1);
  // Existing position validation is outside this function. If no equivalent
  // exists, preserve the raw solution rather than manufacture a different pose.
  ASSERT_TRUE(generate(2.0, { 2.02 }));
  expectPositionsAndFK({ 2.02 });
}

TEST_F(AngleContinuity, MimicSourceIsNotWrapped)
{
  model("continuous", -10, 10, true);
  moveit::core::RobotState a(model_), b(model_);
  a.setToDefaultValues();
  b.setToDefaultValues();
  a.setVariablePosition("joint", 3.13);
  b.setVariablePosition("joint", 3.13 + TWO_PI);
  EXPECT_TRUE(a.getGlobalLinkTransform("tool").matrix().isApprox(b.getGlobalLinkTransform("tool").matrix()));
  EXPECT_FALSE(
      a.getGlobalLinkTransform("mimic_tool").matrix().isApprox(b.getGlobalLinkTransform("mimic_tool").matrix()));
  EXPECT_FALSE(generate(3.11, { 3.13, 3.15, 3.17 }));
}

TEST_F(AngleContinuity, KDLTrajectoryWrap)
{
  model();
  auto* line = new KDL::Path_Line(KDL::Frame(KDL::Rotation::RotZ(3.11)), KDL::Frame(KDL::Rotation::RotZ(3.19)),
                                  new KDL::RotationalInterpolation_SingleAxis(), 1.0);
  auto* velocity = new KDL::VelocityProfile_Trap(1.0, 10.0);
  velocity->SetProfileDuration(0.0, line->PathLength(), 0.5);
  KDL::Trajectory_Segment trajectory(line, velocity);
  ASSERT_TRUE(pilz::generateJointTrajectory(scene_, limits_, trajectory, "arm", "tool", { { "joint", 3.11 } }, 0.1,
                                            output_, error_));
  ASSERT_GT(output_.points.size(), 2u);
  moveit::core::RobotState state(model_);
  state.setToDefaultValues();
  for (size_t i = 0; i < output_.points.size(); ++i)
  {
    const auto& point = output_.points[i];
    state.setVariablePosition("joint", point.positions[0]);
    const auto& fk = state.getGlobalLinkTransform("tool");
    const auto expected = trajectory.Pos(rclcpp::Duration(point.time_from_start).seconds());
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 3; ++col)
        EXPECT_NEAR(fk.linear()(row, col), expected.M(row, col), 1e-10);
    if (i > 0)
    {
      EXPECT_LT(std::abs(point.positions[0] - output_.points[i - 1].positions[0]), 0.1);
    }
  }
  EXPECT_NEAR(output_.points.back().positions[0], 3.19, 1e-10);
}
}  // namespace
