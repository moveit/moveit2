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

#include <pilz_industrial_motion_planner/trajectory_blender_transition_window.hpp>

#include <algorithm>
#include <memory>
#include <math.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/utils/logger.hpp>

namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.planners.pilz.trajectory_blender_transition_window");
}
}  // namespace

bool pilz_industrial_motion_planner::TrajectoryBlenderTransitionWindow::blend(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const pilz_industrial_motion_planner::TrajectoryBlendRequest& req,
    pilz_industrial_motion_planner::TrajectoryBlendResponse& res)
{
  RCLCPP_INFO(getLogger(), "Start trajectory blending using transition window.");

  if (!validateRequest(req, res.error_code))
  {
    RCLCPP_ERROR(getLogger(), "Trajectory blend request is not valid.");
    return false;
  }

  // search for intersection points of the two trajectories with the blending
  // sphere
  // intersection points belongs to blend trajectory after blending
  std::size_t first_intersection_index;
  std::size_t second_intersection_index;
  if (!searchIntersectionPoints(req, first_intersection_index, second_intersection_index))
  {
    RCLCPP_ERROR(getLogger(), "Blend radius too large.");
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  // Select blending period and adjust the start and end point of the blend
  // phase
  std::size_t blend_align_index;
  determineTrajectoryAlignment(req, first_intersection_index, second_intersection_index, blend_align_index);

  // blend the trajectories in Cartesian space
  pilz_industrial_motion_planner::CartesianTrajectory blend_trajectory_cartesian;
  blendTrajectoryCartesian(req, first_intersection_index, second_intersection_index, blend_align_index,
                           blend_trajectory_cartesian);

  // generate the blending trajectory in joint space
  std::map<std::string, double> initial_joint_position, initial_joint_velocity;
  for (const std::string& joint_name :
       req.first_trajectory->getFirstWayPointPtr()->getJointModelGroup(req.group_name)->getActiveJointModelNames())
  {
    initial_joint_position[joint_name] =
        req.first_trajectory->getWayPoint(first_intersection_index - 1).getVariablePosition(joint_name);
    initial_joint_velocity[joint_name] =
        req.first_trajectory->getWayPoint(first_intersection_index - 1).getVariableVelocity(joint_name);
  }
  trajectory_msgs::msg::JointTrajectory blend_joint_trajectory;
  moveit_msgs::msg::MoveItErrorCodes error_code;

  if (!generateJointTrajectory(planning_scene, limits_.getJointLimitContainer(), blend_trajectory_cartesian,
                               req.group_name, req.link_name, initial_joint_position, initial_joint_velocity,
                               blend_joint_trajectory, error_code))
  {
    // LCOV_EXCL_START
    RCLCPP_INFO(getLogger(), "Failed to generate joint trajectory for blending trajectory.");
    res.error_code.val = error_code.val;
    return false;
    // LCOV_EXCL_STOP
  }

  res.first_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(req.first_trajectory->getRobotModel(),
                                                                             req.first_trajectory->getGroup());
  res.blend_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(req.first_trajectory->getRobotModel(),
                                                                             req.first_trajectory->getGroup());
  res.second_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(req.first_trajectory->getRobotModel(),
                                                                              req.first_trajectory->getGroup());

  // set the three trajectories after blending in response
  // erase the points [first_intersection_index, back()] from the first
  // trajectory
  for (size_t i = 0; i < first_intersection_index; ++i)
  {
    res.first_trajectory->insertWayPoint(i, req.first_trajectory->getWayPoint(i),
                                         req.first_trajectory->getWayPointDurationFromPrevious(i));
  }

  // append the blend trajectory
  res.blend_trajectory->setRobotTrajectoryMsg(req.first_trajectory->getFirstWayPoint(), blend_joint_trajectory);

  // copy the points [second_intersection_index, len] from the second trajectory
  for (size_t i = second_intersection_index + 1; i < req.second_trajectory->getWayPointCount(); ++i)
  {
    res.second_trajectory->insertWayPoint(i - (second_intersection_index + 1), req.second_trajectory->getWayPoint(i),
                                          req.second_trajectory->getWayPointDurationFromPrevious(i));
  }

  res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return true;
}

bool pilz_industrial_motion_planner::TrajectoryBlenderTransitionWindow::validateRequest(
    const pilz_industrial_motion_planner::TrajectoryBlendRequest& req,
    moveit_msgs::msg::MoveItErrorCodes& error_code) const
{
  RCLCPP_DEBUG(getLogger(), "Validate the trajectory blend request.");

  // check planning group
  if (!req.first_trajectory->getRobotModel()->hasJointModelGroup(req.group_name))
  {
    RCLCPP_ERROR_STREAM(getLogger(), "Unknown planning group: " << req.group_name);
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
    return false;
  }

  // check link exists
  if (!req.first_trajectory->getRobotModel()->hasLinkModel(req.link_name) &&
      !req.first_trajectory->getLastWayPoint().hasAttachedBody(req.link_name))
  {
    RCLCPP_ERROR_STREAM(getLogger(), "Unknown link name: " << req.link_name);
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_LINK_NAME;
    return false;
  }

  if (req.blend_radius <= 0)
  {
    RCLCPP_ERROR(getLogger(), "Blending radius must be positive");
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  // end position of the first trajectory and start position of second
  // trajectory must be the same
  if (!pilz_industrial_motion_planner::isRobotStateEqual(
          req.first_trajectory->getLastWayPoint(), req.second_trajectory->getFirstWayPoint(), req.group_name, EPSILON))
  {
    RCLCPP_ERROR_STREAM(getLogger(), "During blending the last point of the preceding and the first point of the "
                                     "succeeding trajectory");
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  // end position of the first trajectory and start position of second
  // trajectory must have zero
  // velocities/accelerations
  if (!pilz_industrial_motion_planner::isRobotStateStationary(req.first_trajectory->getLastWayPoint(), req.group_name,
                                                              EPSILON) ||
      !pilz_industrial_motion_planner::isRobotStateStationary(req.second_trajectory->getFirstWayPoint(), req.group_name,
                                                              EPSILON))
  {
    RCLCPP_ERROR(getLogger(), "Intersection point of the blending trajectories has non-zero velocities/accelerations.");
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  return true;
}

Eigen::Isometry3d interpolatePose(const robot_trajectory::RobotTrajectoryPtr& trajectory, const std::string& link_name,
                                  const double& time, size_t& found_index, const size_t& start_index = 0)
{
  found_index = 0;
  for (std::size_t i = start_index; i < trajectory->getWayPointCount() - 1; ++i)
  {
    if (trajectory->getWayPointDurationFromStart(i + 1) >= time)
    {
      found_index = i;
      break;
    }
  }

  // If time is outside known waypoints, return the closest available pose
  if (found_index == 0)
  {
    return trajectory->getWayPoint(0).getFrameTransform(link_name);
  }
  if (found_index == trajectory->getWayPointCount() - 1)
  {
    return trajectory->getWayPoint(found_index).getFrameTransform(link_name);
  }

  // Get timestamps and transformations
  double t1 = trajectory->getWayPointDurationFromStart(found_index);
  double t2 = trajectory->getWayPointDurationFromStart(found_index + 1);
  Eigen::Isometry3d pose1 = trajectory->getWayPoint(found_index).getFrameTransform(link_name);
  Eigen::Isometry3d pose2 = trajectory->getWayPoint(found_index + 1).getFrameTransform(link_name);

  // Compute interpolation factor
  double interpolation_factor = (time - t1) / (t2 - t1);

  // Linear interpolation for position
  Eigen::Isometry3d interpolated_pose;
  pilz_industrial_motion_planner::interpolate(pose1, pose2, interpolation_factor, interpolated_pose);
  return interpolated_pose;
}

void pilz_industrial_motion_planner::TrajectoryBlenderTransitionWindow::blendTrajectoryCartesian(
    const pilz_industrial_motion_planner::TrajectoryBlendRequest& req, const std::size_t first_interse_index,
    const std::size_t second_interse_index, const std::size_t /* blend_align_index */,
    pilz_industrial_motion_planner::CartesianTrajectory& trajectory) const
{
  // other fields of the trajectory
  trajectory.group_name = req.group_name;
  trajectory.link_name = req.link_name;

  // Start time of the blending phase
  double t_start = req.first_trajectory->getWayPointDurationFromStart(first_interse_index);
  // Duration of the blending phase on the first trajectory
  double d1 = req.first_trajectory->getDuration() - t_start;
  // Duration of the blending phase on the second trajectory
  double d2 = req.second_trajectory->getWayPointDurationFromStart(second_interse_index);
  // Time to align the two trajectories
  double align_time = (d1 > d2) ? req.first_trajectory->getDuration() - d2 : t_start;

  // Pose on first trajectory
  Eigen::Isometry3d blend_sample_pose1 =
      req.first_trajectory->getWayPoint(first_interse_index).getFrameTransform(req.link_name);

  // Pose on second trajectory
  Eigen::Isometry3d blend_sample_pose2 =
      req.second_trajectory->getWayPoint(second_interse_index).getFrameTransform(req.link_name);

  // blend the trajectory
  double blend_duration = d2 + align_time - t_start;
  pilz_industrial_motion_planner::CartesianTrajectoryPoint waypoint;
  blend_sample_pose2 = req.second_trajectory->getFirstWayPoint().getFrameTransform(req.link_name);

  // Pose on blending trajectory
  Eigen::Isometry3d blend_sample_pose;

  // Define an arbitrary small sample time to sample the blending trajectory
  double sampling_time = 0.001;
  size_t blend_sample_pose1_index, blend_sample_pose2_index = 0;

  int num_samples = std::floor(blend_duration / sampling_time);
  sampling_time = blend_duration / num_samples;

  double blend_time = 0.0;
  Eigen::Isometry3d last_blend_sample_pose = blend_sample_pose1;

  // Add the first point
  double time_offset = req.first_trajectory->getWayPointDurationFromPrevious(first_interse_index);
  waypoint.pose = tf2::toMsg(blend_sample_pose1);
  waypoint.time_from_start = rclcpp::Duration::from_seconds(time_offset);
  trajectory.points.push_back(waypoint);
  while (blend_time <= blend_duration + EPSILON)
  {
    // if the first trajectory does not reach the last sample, update
    if ((t_start + blend_time) <= req.first_trajectory->getDuration())
    {
      blend_sample_pose1 = interpolatePose(req.first_trajectory, req.link_name, t_start + blend_time,
                                           blend_sample_pose1_index, blend_sample_pose1_index);
    }

    // if after the alignment, the second trajectory starts, update
    if ((t_start + blend_time) >= align_time)
    {
      blend_sample_pose2 = interpolatePose(req.second_trajectory, req.link_name, (t_start + blend_time) - align_time,
                                           blend_sample_pose2_index, blend_sample_pose2_index);
    }

    double s = (blend_time + sampling_time) / blend_duration;
    double alpha = 6 * std::pow(s, 5) - 15 * std::pow(s, 4) + 10 * std::pow(s, 3);

    interpolate(blend_sample_pose1, blend_sample_pose2, alpha, blend_sample_pose);

    blend_time += sampling_time;
    // Ensures samples are far enough apart to avoid numerical issues in numerical inverse kinematics
    if (((blend_sample_pose.translation() - last_blend_sample_pose.translation()).norm() < 1e-3) &&
        (blend_sample_pose.rotation().isApprox(last_blend_sample_pose.rotation(), 1e-3)) &&
        (blend_time < blend_duration))  // Force the addition of the last point
    {
      continue;
    }

    // Store the last insert pose
    last_blend_sample_pose = blend_sample_pose;

    // push to the trajectory
    waypoint.pose = tf2::toMsg(blend_sample_pose);
    waypoint.time_from_start = rclcpp::Duration::from_seconds(time_offset + blend_time);
    trajectory.points.push_back(waypoint);
  }
}

bool pilz_industrial_motion_planner::TrajectoryBlenderTransitionWindow::searchIntersectionPoints(
    const pilz_industrial_motion_planner::TrajectoryBlendRequest& req, std::size_t& first_interse_index,
    std::size_t& second_interse_index) const
{
  RCLCPP_INFO(getLogger(), "Search for start and end point of blending trajectory.");

  // compute the position of the center of the blend sphere
  // (last point of the first trajectory, first point of the second trajectory)
  Eigen::Isometry3d circ_pose = req.first_trajectory->getLastWayPoint().getFrameTransform(req.link_name);

  // Search for intersection points according to distance
  if (!linearSearchIntersectionPoint(req.link_name, circ_pose.translation(), req.blend_radius, req.first_trajectory,
                                     true, first_interse_index))
  {
    RCLCPP_ERROR_STREAM(getLogger(), "Intersection point of first trajectory not found.");
    return false;
  }
  RCLCPP_INFO_STREAM(getLogger(), "Intersection point of first trajectory found, index: " << first_interse_index);

  if (!linearSearchIntersectionPoint(req.link_name, circ_pose.translation(), req.blend_radius, req.second_trajectory,
                                     false, second_interse_index))
  {
    RCLCPP_ERROR_STREAM(getLogger(), "Intersection point of second trajectory not found.");
    return false;
  }

  RCLCPP_INFO_STREAM(getLogger(), "Intersection point of second trajectory found, index: " << second_interse_index);
  return true;
}

void pilz_industrial_motion_planner::TrajectoryBlenderTransitionWindow::determineTrajectoryAlignment(
    const pilz_industrial_motion_planner::TrajectoryBlendRequest& req, std::size_t first_interse_index,
    std::size_t second_interse_index, std::size_t& blend_align_index) const
{
  size_t way_point_count_1 = req.first_trajectory->getWayPointCount() - first_interse_index;
  size_t way_point_count_2 = second_interse_index + 1;

  if (way_point_count_1 > way_point_count_2)
  {
    blend_align_index = req.first_trajectory->getWayPointCount() - second_interse_index - 1;
  }
  else
  {
    blend_align_index = first_interse_index;
  }
}
