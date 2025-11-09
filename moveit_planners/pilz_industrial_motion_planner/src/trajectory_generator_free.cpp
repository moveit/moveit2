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

#include <pilz_industrial_motion_planner/trajectory_generator_free.hpp>

#include <pilz_industrial_motion_planner/tip_frame_getter.hpp>

#include <cassert>
#include <sstream>
#include <time.h>
#include <moveit/robot_state/conversions.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/utilities/error.h>
// TODO: Remove conditional include when released to all active distros.
#if __has_include(<tf2/convert.hpp>)
#include <tf2/convert.hpp>
#else
#include <tf2/convert.h>
#endif
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_eigen_kdl/tf2_eigen_kdl.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/utils/logger.hpp>

namespace pilz_industrial_motion_planner
{
namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.planners.pilz.trajectory_generator.lin");
}
}  // namespace
TrajectoryGeneratorFree::TrajectoryGeneratorFree(const moveit::core::RobotModelConstPtr& robot_model,
                                                 const LimitsContainer& planner_limits,
                                                 const std::string& /*group_name*/)
  : TrajectoryGenerator::TrajectoryGenerator(robot_model, planner_limits)
{
  planner_limits_.printCartesianLimits();
}

void TrajectoryGeneratorFree::extractMotionPlanInfo(const planning_scene::PlanningSceneConstPtr& scene,
                                                    const planning_interface::MotionPlanRequest& req,
                                                    TrajectoryGenerator::MotionPlanInfo& info) const
{
  RCLCPP_DEBUG(getLogger(), "Extract necessary information from motion plan request.");

  info.group_name = req.group_name;
  moveit::core::RobotState robot_state = scene->getCurrentState();

  // goal given in joint space
  if (!req.goal_constraints.front().joint_constraints.empty())
  {
    info.link_name = getSolverTipFrame(robot_model_->getJointModelGroup(req.group_name));

    if (req.goal_constraints.front().joint_constraints.size() !=
        robot_model_->getJointModelGroup(req.group_name)->getActiveJointModelNames().size())
    {
      std::ostringstream os;
      os << "Number of joints in goal does not match number of joints of group "
            "(Number joints goal: "
         << req.goal_constraints.front().joint_constraints.size() << " | Number of joints of group: "
         << robot_model_->getJointModelGroup(req.group_name)->getActiveJointModelNames().size() << ')';
      throw JointNumberMismatch(os.str());
    }

    for (const auto& joint_item : req.goal_constraints.front().joint_constraints)
    {
      info.goal_joint_position[joint_item.joint_name] = joint_item.position;
    }

    computeLinkFK(robot_state, info.link_name, info.goal_joint_position, info.goal_pose);
  }
  // goal given in Cartesian space
  else
  {
    std::string frame_id;

    info.link_name = req.goal_constraints.front().position_constraints.front().link_name;
    if (req.goal_constraints.front().position_constraints.front().header.frame_id.empty() ||
        req.goal_constraints.front().orientation_constraints.front().header.frame_id.empty())
    {
      RCLCPP_WARN(getLogger(), "Frame id is not set in position/orientation constraints of "
                               "goal. Use model frame as default");
      frame_id = robot_model_->getModelFrame();
    }
    else
    {
      frame_id = req.goal_constraints.front().position_constraints.front().header.frame_id;
    }

    // goal pose with optional offset wrt. the planning frame
    for (const auto& pc : req.path_constraints.position_constraints)
    {
      Eigen::Isometry3d waypoint;
      tf2::fromMsg(pc.constraint_region.primitive_poses.front(), waypoint);
      waypoint = scene->getFrameTransform(frame_id) * waypoint;
      RCLCPP_INFO_STREAM(getLogger(), "Added waypoint at position: " << waypoint.translation().transpose());
      info.waypoints.push_back(waypoint);
    }
    info.goal_pose = scene->getFrameTransform(frame_id) * getConstraintPose(req.goal_constraints.front());
    frame_id = robot_model_->getModelFrame();

    // check goal pose ik before Cartesian motion plan starts
    std::map<std::string, double> ik_solution;
    if (!computePoseIK(scene, info.group_name, info.link_name, info.goal_pose, frame_id, info.start_joint_position,
                       ik_solution))
    {
      std::ostringstream os;
      os << "Failed to compute inverse kinematics for link: " << info.link_name << " of goal pose";
      throw LinInverseForGoalIncalculable(os.str());
    }
  }

  // Ignored return value because at this point the function should always
  // return 'true'.
  computeLinkFK(robot_state, info.link_name, info.start_joint_position, info.start_pose);
}

void TrajectoryGeneratorFree::plan(const planning_scene::PlanningSceneConstPtr& scene,
                                   const planning_interface::MotionPlanRequest& req, const MotionPlanInfo& plan_info,
                                   double sampling_time, trajectory_msgs::msg::JointTrajectory& joint_trajectory)
{
  // set pilz cartesian limits for each item
  setMaxCartesianSpeed(req);
  // create Cartesian path for free
  std::unique_ptr<KDL::Path> path(setPathFree(plan_info.start_pose, plan_info.waypoints, req.smoothness_level));
  // create velocity profile
  std::unique_ptr<KDL::VelocityProfile> vp(
      cartesianTrapVelocityProfile(req.max_velocity_scaling_factor, req.max_acceleration_scaling_factor, path));

  // combine path and velocity profile into Cartesian trajectory
  // with the third parameter set to false, KDL::Trajectory_Segment does not
  // take
  // the ownship of Path and Velocity Profile
  KDL::Trajectory_Segment cart_trajectory(path.get(), vp.get(), false);

  moveit_msgs::msg::MoveItErrorCodes error_code;
  // sample the Cartesian trajectory and compute joint trajectory using inverse
  // kinematics
  if (!generateJointTrajectory(scene, planner_limits_.getJointLimitContainer(), cart_trajectory, plan_info.group_name,
                               plan_info.link_name, plan_info.start_joint_position, sampling_time, joint_trajectory,
                               error_code))
  {
    std::ostringstream os;
    os << "Failed to generate valid joint trajectory from the Cartesian path";
    throw LinTrajectoryConversionFailure(os.str(), error_code.val);
  }
}

std::unique_ptr<KDL::Path> TrajectoryGeneratorFree::setPathFree(const Eigen::Affine3d& start_pose,
                                                                const std::vector<Eigen::Isometry3d>& waypoints,
                                                                double smoothness_level) const
{
  RCLCPP_DEBUG(getLogger(), "Set Cartesian path for FREE command.");

  KDL::Frame kdl_start_pose;
  tf2::transformEigenToKDL(start_pose, kdl_start_pose);
  // transform waypoints to KDL frames
  std::vector<KDL::Frame> kdl_waypoints;
  for (const auto& waypoint : waypoints)
  {
    KDL::Frame kdl_waypoint;
    tf2::transformEigenToKDL(waypoint, kdl_waypoint);
    kdl_waypoints.push_back(kdl_waypoint);
  }
  // distance between start pose and first waypoint
  double dist = (kdl_start_pose.p - kdl_waypoints.front().p).Norm();
  RCLCPP_INFO(getLogger(), "distance: %f", dist);

  RCLCPP_INFO_STREAM(getLogger(), "Transformed waypoints number: " << kdl_waypoints.size());
  double eqradius = max_cartesian_speed_ / planner_limits_.getCartesianLimits().max_rot_vel;
  KDL::RotationalInterpolation* rot_interpo = new KDL::RotationalInterpolation_SingleAxis();
  // get the largest possible blend radius based on waypoints and smoothness level
  double blend_radius = computeBlendRadius(kdl_waypoints, smoothness_level);
  RCLCPP_INFO(getLogger(), "Computed blend radius: %f", blend_radius);
  KDL::Path_RoundedComposite* composite_path = new KDL::Path_RoundedComposite(blend_radius, eqradius, rot_interpo);
  // make sure the start pose is the same as the first pose in waypoints with tolerance
  if (dist > 1.0e-3)
  {
    composite_path->Add(kdl_start_pose);
  }
  else
    RCLCPP_WARN_STREAM(getLogger(), "Close start point. start: " << kdl_start_pose.p
                                                                 << "fisrt_waypoint: " << kdl_waypoints.front().p);

  for (const auto& kdl_waypoint : kdl_waypoints)
  {
    composite_path->Add(kdl_waypoint);
  }
  return std::unique_ptr<KDL::Path>(composite_path);
}

void TrajectoryGeneratorFree::cmdSpecificRequestValidation(const planning_interface::MotionPlanRequest& req) const
{
  if (req.path_constraints.position_constraints.size() < 2)
  {
    std::ostringstream os;
    os << "waypoints specified in path constraints is less than 2 for FREE motion.";
    throw NoWaypointsSpecified(os.str());
  }
}

double TrajectoryGeneratorFree::computeBlendRadius(const std::vector<KDL::Frame>& waypoints_, double smoothness) const
{
  double max_allowed_radius = std::numeric_limits<double>::infinity();

  auto pose_distance = [](const KDL::Frame& p1, const KDL::Frame& p2) { return (p1.p - p2.p).Norm(); };

  // to calculate the angle between two segments
  auto segment_angle = [](const KDL::Frame& p1, const KDL::Frame& p2, const KDL::Frame& p3) {
    KDL::Vector v1 = p2.p - p1.p;
    KDL::Vector v2 = p2.p - p3.p;

    double norm_product = v1.Norm() * v2.Norm();
    if (norm_product < 0.25e-6)
      return 0.0;  // avoid division by zero

    double cos_theta = KDL::dot(v1, v2) / norm_product;
    cos_theta = std::clamp(cos_theta, -1.0, 1.0);

    return std::acos(cos_theta);
  };

  for (size_t i = 1; i + 1 < waypoints_.size(); ++i)
  {
    double dist1 = pose_distance(waypoints_[i], waypoints_[i - 1]);
    double dist2 = pose_distance(waypoints_[i + 1], waypoints_[i]);

    if (dist1 < 1e-4 || dist2 < 1e-4)
    {
      RCLCPP_WARN(getLogger(), "Waypoint %zu is too close to neighbors (%.6f, %.6f).", i, dist1, dist2);
      continue;
    }

    // The maximum feasible radius for this junction
    double local_max_radius = std::tan(segment_angle(waypoints_[i - 1], waypoints_[i], waypoints_[i + 1]) / 2.0) *
                              std::min(dist1 / 2.0, dist2 / 2.0);

    // Keep track of the tightest constraint
    // due to roundedcomposite don't support changing radius
    if (local_max_radius < max_allowed_radius)
      max_allowed_radius = local_max_radius;
  }

  // Apply the smoothness scaling factor
  double max_radius = max_allowed_radius * std::clamp(smoothness, 0.01, 0.99);

  RCLCPP_DEBUG(getLogger(), "Validated smoothness scaling %.2f → blend radius %.6f", smoothness, max_radius);

  return max_radius;
}
}  // namespace pilz_industrial_motion_planner
