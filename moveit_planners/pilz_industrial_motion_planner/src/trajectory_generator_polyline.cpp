/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
 *  Copyright (c) 2025 Aiman Haidar
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

#include <pilz_industrial_motion_planner/trajectory_generator_polyline.hpp>
#include <pilz_industrial_motion_planner/path_polyline_generator.hpp>

#include <pilz_industrial_motion_planner/tip_frame_getter.hpp>

#include <cassert>
#include <sstream>
#include <time.h>
#include <moveit/robot_state/conversions.hpp>
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
  return moveit::getLogger("moveit.planners.pilz.trajectory_generator.polyline");
}
}  // namespace
TrajectoryGeneratorPolyline::TrajectoryGeneratorPolyline(const moveit::core::RobotModelConstPtr& robot_model,
                                                         const LimitsContainer& planner_limits,
                                                         const std::string& /*group_name*/)
  : TrajectoryGenerator::TrajectoryGenerator(robot_model, planner_limits)
{
  planner_limits_.printCartesianLimits();
}

void TrajectoryGeneratorPolyline::extractMotionPlanInfo(const planning_scene::PlanningSceneConstPtr& scene,
                                                        const planning_interface::MotionPlanRequest& req,
                                                        TrajectoryGenerator::MotionPlanInfo& info) const
{
  RCLCPP_DEBUG(getLogger(), "Extract necessary information from motion plan request.");

  info.group_name = req.group_name;
  moveit::core::RobotState robot_state = scene->getCurrentState();

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

  // Add the path waypoints
  for (const auto& pc : req.path_constraints.position_constraints)
  {
    Eigen::Isometry3d waypoint;
    waypoint = getConstraintPose(pc.constraint_region.primitive_poses.front().position,
                                 pc.constraint_region.primitive_poses.front().orientation, pc.target_point_offset);
    waypoint = scene->getFrameTransform(frame_id) * waypoint;
    info.waypoints.push_back(waypoint);
  }
  // goal constraint is just the final pose
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

  // Ignored return value because at this point the function should always
  // return 'true'.
  computeLinkFK(robot_state, info.link_name, info.start_joint_position, info.start_pose);
}

void TrajectoryGeneratorPolyline::plan(const planning_scene::PlanningSceneConstPtr& scene,
                                       const planning_interface::MotionPlanRequest& req,
                                       const MotionPlanInfo& plan_info, double sampling_time,
                                       trajectory_msgs::msg::JointTrajectory& joint_trajectory)
{
  // set pilz cartesian limits for each item
  setMaxCartesianSpeed(req);
  // create Cartesian POLYLINE path
  std::unique_ptr<KDL::Path> path;
  try
  {
    path = setPathPolyline(plan_info.start_pose, plan_info.waypoints, req.smoothness_level);
  }
  catch (const KDL::Error_MotionPlanning& e)
  {
    RCLCPP_ERROR(getLogger(), "Motion planning error: %s", e.Description());
    int code = e.GetType();
    std::ostringstream os;
    if (code == 3102 || code == 3103)
    {
      os << "zero distance between two points";
    }
    else if (code == 3104)
    {
      os << "waypoints specified in path constraints have three consicutive colinear points";
    }
    else if (code == 3105 || code == 3106)
    {
      os << "rounding circle of a point is bigger than the distance with one of the neighbor points";
    }
    else if (code == 3001 || code == 3002)
    {
      os << "the rounding radius is lower than KDL::epsilon. use bigger smoothness or resample your waypoints";
    }
    throw ConsicutiveColinearWaypoints(os.str());
  }
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

std::unique_ptr<KDL::Path> TrajectoryGeneratorPolyline::setPathPolyline(const Eigen::Affine3d& start_pose,
                                                                        const std::vector<Eigen::Isometry3d>& waypoints,
                                                                        double smoothness_level) const
{
  RCLCPP_DEBUG(getLogger(), "Set Cartesian path for POLYLINE command.");

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

  RCLCPP_INFO_STREAM(getLogger(), "Transformed waypoints number: " << kdl_waypoints.size());

  double eqradius = max_cartesian_speed_ / planner_limits_.getCartesianLimits().max_rot_vel;
  KDL::RotationalInterpolation* rot_interpo = new KDL::RotationalInterpolation_SingleAxis();

  return PathPolylineGenerator::polylineFromWaypoints(kdl_start_pose, kdl_waypoints, rot_interpo, smoothness_level,
                                                      eqradius);
}

void TrajectoryGeneratorPolyline::cmdSpecificRequestValidation(const planning_interface::MotionPlanRequest& req) const
{
  if (req.path_constraints.position_constraints.size() < 2)
  {
    std::ostringstream os;
    os << "waypoints specified in path constraints is less than 2 for POLYLINE motion.";
    throw NoWaypointsSpecified(os.str());
  }
}
}  // namespace pilz_industrial_motion_planner
