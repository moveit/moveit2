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

#include <pilz_industrial_motion_planner/trajectory_generator_lin.hpp>

#include <pilz_industrial_motion_planner/tip_frame_getter.hpp>

#include <cassert>
#include <sstream>
#include <time.h>
#include <moveit/robot_state/conversions.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_composite.hpp>
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
      info.waypoints.push_back(waypoint);
    }
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
  // create Cartesian path for free
  std::unique_ptr<KDL::Path> path(setPathFree(plan_info.start_pose, plan_info.waypoints));

  // set pilz cartesian limits for each item
  setMaxCartesianSpeed(req);
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
                                                                const std::vector<Eigen::Affine3d>& waypoints) const
{
  RCLCPP_DEBUG(getLogger(), "Set Cartesian path for LIN command.");

  KDL::Frame kdl_start_pose;
  tf2::transformEigenToKDL(start_pose, kdl_start_pose);
  std::vector<KDL::Frame> kdl_waypoints;
  for (const auto& waypoint : waypoints)
  {
    KDL::Frame kdl_waypoint;
    tf2::transformEigenToKDL(waypoint, kdl_waypoint);
    kdl_waypoints.push_back(kdl_waypoint);
  }
  double eqradius = max_cartesian_speed_ / planner_limits_.getCartesianLimits().max_rot_vel;
  KDL::RotationalInterpolation* rot_interpo = new KDL::RotationalInterpolation_SingleAxis();
  KDL::Path_Composite* composite_path = new KDL::Path_Composite();
  KDL::Frame previous_pose = kdl_start_pose;

  for (const auto& kdl_waypoint : kdl_waypoints)
  {
    composite_path->Add(new KDL::Path_Line(previous_pose, kdl_waypoint, rot_interpo, eqradius, true));
    previous_pose = kdl_waypoint;
  }
  return std::unique_ptr<KDL::Path>(composite_path);
}

}  // namespace pilz_industrial_motion_planner
