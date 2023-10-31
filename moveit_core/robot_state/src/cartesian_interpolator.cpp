/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ioan A. Sucan
 *  Copyright (c) 2013, Willow Garage, Inc.
 *  Copyright (c) 2019, PickNik Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Ioan Sucan, Sachin Chitta, Acorn Pooley, Mario Prats, Dave Coleman */

#include <memory>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <geometric_shapes/check_isometry.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rcpputils/asserts.hpp>
#include <moveit/utils/logger.hpp>

namespace moveit::core
{

// Minimum amount of path waypoints recommended to reliably compute a joint-space increment average.
// If relative jump detection is selected and the path is shorter than `MIN_STEPS_FOR_JUMP_THRESH`, a warning message
// will be printed out.
static const std::size_t MIN_STEPS_FOR_JUMP_THRESH = 10;

namespace
{

rclcpp::Logger getLogger()
{
  static auto logger = moveit::makeChildLogger("cartesian_interpolator");
  return logger;
}

std::optional<int> hasRelativeJointSpaceJump(const std::vector<moveit::core::RobotStatePtr>& waypoints,
                                             const moveit::core::JointModelGroup& group, double jump_threshold_factor)
{
  if (waypoints.size() < MIN_STEPS_FOR_JUMP_THRESH)
  {
    RCLCPP_WARN(getLogger(),
                "The computed path is too short to detect jumps in joint-space. "
                "Need at least %zu steps, only got %zu. Try a lower max_step.",
                MIN_STEPS_FOR_JUMP_THRESH, waypoints.size());
  }

  std::vector<double> dist_vector;
  dist_vector.reserve(waypoints.size() - 1);
  double total_dist = 0.0;
  for (std::size_t i = 1; i < waypoints.size(); ++i)
  {
    const double dist_prev_point = waypoints[i]->distance(*waypoints[i - 1], &group);
    dist_vector.push_back(dist_prev_point);
    total_dist += dist_prev_point;
  }

  // compute the average distance between the states we looked at.
  double thres = jump_threshold_factor * (total_dist / static_cast<double>(dist_vector.size()));
  for (std::size_t i = 0; i < dist_vector.size(); ++i)
  {
    if (dist_vector[i] > thres)
    {
      return i + 1;
    }
  }

  return std::nullopt;
}

std::optional<int> hasAbsoluteJointSpaceJump(const std::vector<moveit::core::RobotStatePtr>& waypoints,
                                             const moveit::core::JointModelGroup& group, double revolute_threshold,
                                             double prismatic_threshold)
{
  const bool check_revolute = revolute_threshold > 0.0;
  const bool check_prismatic = prismatic_threshold > 0.0;

  const std::vector<const moveit::core::JointModel*>& joints = group.getActiveJointModels();
  for (std::size_t i = 1; i < waypoints.size(); ++i)
  {
    for (const auto& joint : joints)
    {
      const double distance = waypoints[i]->distance(*waypoints[i - 1], joint);
      switch (joint->getType())
      {
        case moveit::core::JointModel::REVOLUTE:
          if (check_revolute && distance > revolute_threshold)
          {
            return i;
          }
          break;
        case moveit::core::JointModel::PRISMATIC:
          if (check_prismatic && distance > prismatic_threshold)
          {
            return i;
          }
          break;
        default:
          RCLCPP_WARN(getLogger(),
                      "Joint %s has not supported type %s. \n"
                      "hasAbsoluteJointSpaceJump only supports prismatic and revolute joints. Skipping joint jump "
                      "check for this joint.",
                      joint->getName().c_str(), joint->getTypeName().c_str());
          continue;
      }
    }
  }

  return std::nullopt;
}
}  // namespace

JumpThreshold JumpThreshold::disabled()
{
  return JumpThreshold();
}

JumpThreshold JumpThreshold::relative(double relative_factor)
{
  rcpputils::require_true(relative_factor > 1.0);

  JumpThreshold threshold;
  threshold.relative_factor = relative_factor;
  return threshold;
}

JumpThreshold JumpThreshold::absolute(double revolute, double prismatic)
{
  rcpputils::require_true(revolute > 0.0);
  rcpputils::require_true(prismatic > 0.0);

  JumpThreshold threshold;
  threshold.revolute = revolute;
  threshold.prismatic = prismatic;
  return threshold;
}

JumpThreshold::JumpThreshold(double relative_factor)
{
  this->relative_factor = relative_factor;
}
JumpThreshold::JumpThreshold(double revolute, double prismatic)
{
  this->revolute = revolute;
  this->prismatic = prismatic;
}

CartesianInterpolator::Distance CartesianInterpolator::computeCartesianPath(
    RobotState* start_state, const JointModelGroup* group, std::vector<RobotStatePtr>& path, const LinkModel* link,
    const Eigen::Vector3d& translation, bool global_reference_frame, const MaxEEFStep& max_step,
    const JumpThreshold& jump_threshold, const GroupStateValidityCallbackFn& validCallback,
    const kinematics::KinematicsQueryOptions& options, const kinematics::KinematicsBase::IKCostFn& cost_function)
{
  const double distance = translation.norm();
  // The target pose is obtained by adding the translation vector to the link's current pose
  Eigen::Isometry3d pose = start_state->getGlobalLinkTransform(link);

  // the translation direction can be specified w.r.t. the local link frame (then rotate into global frame)
  pose.translation() += global_reference_frame ? translation : pose.linear() * translation;

  // call computeCartesianPath for the computed target pose in the global reference frame
  return CartesianInterpolator::Distance(distance) * computeCartesianPath(start_state, group, path, link, pose, true,
                                                                          max_step, jump_threshold, validCallback,
                                                                          options, cost_function);
}

CartesianInterpolator::Percentage CartesianInterpolator::computeCartesianPath(
    RobotState* start_state, const JointModelGroup* group, std::vector<RobotStatePtr>& path, const LinkModel* link,
    const Eigen::Isometry3d& target, bool global_reference_frame, const MaxEEFStep& max_step,
    const JumpThreshold& jump_threshold, const GroupStateValidityCallbackFn& validCallback,
    const kinematics::KinematicsQueryOptions& options, const kinematics::KinematicsBase::IKCostFn& cost_function,
    const Eigen::Isometry3d& link_offset)
{
  // check unsanitized inputs for non-isometry
  ASSERT_ISOMETRY(target)
  ASSERT_ISOMETRY(link_offset)

  const std::vector<const JointModel*>& cjnt = group->getContinuousJointModels();
  // make sure that continuous joints wrap
  for (const JointModel* joint : cjnt)
    start_state->enforceBounds(joint);

  // Cartesian pose we start from
  Eigen::Isometry3d start_pose = start_state->getGlobalLinkTransform(link) * link_offset;
  Eigen::Isometry3d offset = link_offset.inverse();

  // the target can be in the local reference frame (in which case we rotate it)
  Eigen::Isometry3d rotated_target = global_reference_frame ? target : start_pose * target;

  Eigen::Quaterniond start_quaternion(start_pose.linear());
  Eigen::Quaterniond target_quaternion(rotated_target.linear());

  if (max_step.translation <= 0.0 && max_step.rotation <= 0.0)
  {
    RCLCPP_ERROR(getLogger(),
                 "Invalid MaxEEFStep passed into computeCartesianPath. Both the MaxEEFStep.rotation and "
                 "MaxEEFStep.translation components must be non-negative and at least one component must be "
                 "greater than zero");
    return 0.0;
  }

  double rotation_distance = start_quaternion.angularDistance(target_quaternion);
  double translation_distance = (rotated_target.translation() - start_pose.translation()).norm();

  // decide how many steps we will need for this path
  std::size_t translation_steps = 0;
  if (max_step.translation > 0.0)
    translation_steps = floor(translation_distance / max_step.translation);

  std::size_t rotation_steps = 0;
  if (max_step.rotation > 0.0)
    rotation_steps = floor(rotation_distance / max_step.rotation);

  // If we are testing for relative jumps, we always want at least MIN_STEPS_FOR_JUMP_THRESH steps
  std::size_t steps = std::max(translation_steps, rotation_steps) + 1;
  if (jump_threshold.relative_factor > 0 && steps < MIN_STEPS_FOR_JUMP_THRESH)
    steps = MIN_STEPS_FOR_JUMP_THRESH;

  // To limit absolute joint-space jumps, we pass consistency limits to the IK solver
  std::vector<double> consistency_limits;
  if (jump_threshold.prismatic > 0 || jump_threshold.revolute > 0)
  {
    for (const JointModel* jm : group->getActiveJointModels())
    {
      double limit;
      switch (jm->getType())
      {
        case JointModel::REVOLUTE:
          limit = jump_threshold.revolute;
          break;
        case JointModel::PRISMATIC:
          limit = jump_threshold.prismatic;
          break;
        default:
          limit = 0.0;
      }
      if (limit == 0.0)
        limit = jm->getMaximumExtent();
      consistency_limits.push_back(limit);
    }
  }

  path.clear();
  path.push_back(std::make_shared<moveit::core::RobotState>(*start_state));

  double last_valid_percentage = 0.0;
  for (std::size_t i = 1; i <= steps; ++i)
  {
    double percentage = static_cast<double>(i) / static_cast<double>(steps);

    Eigen::Isometry3d pose(start_quaternion.slerp(percentage, target_quaternion));
    pose.translation() = percentage * rotated_target.translation() + (1 - percentage) * start_pose.translation();

    // Explicitly use a single IK attempt only (by setting a timeout of 0.0), using the current state as the seed.
    // Random seeding (of additional attempts) would create large joint-space jumps.
    if (start_state->setFromIK(group, pose * offset, link->getName(), consistency_limits, 0.0, validCallback, options,
                               cost_function))
    {
      path.push_back(std::make_shared<moveit::core::RobotState>(*start_state));
    }
    else
    {
      break;
    }

    last_valid_percentage = percentage;
  }

  last_valid_percentage *= checkJointSpaceJump(group, path, jump_threshold);

  return CartesianInterpolator::Percentage(last_valid_percentage);
}

CartesianInterpolator::Percentage CartesianInterpolator::computeCartesianPath(
    RobotState* start_state, const JointModelGroup* group, std::vector<RobotStatePtr>& path, const LinkModel* link,
    const EigenSTL::vector_Isometry3d& waypoints, bool global_reference_frame, const MaxEEFStep& max_step,
    const JumpThreshold& jump_threshold, const GroupStateValidityCallbackFn& validCallback,
    const kinematics::KinematicsQueryOptions& options, const kinematics::KinematicsBase::IKCostFn& cost_function,
    const Eigen::Isometry3d& link_offset)
{
  double percentage_solved = 0.0;
  for (std::size_t i = 0; i < waypoints.size(); ++i)
  {
    // Don't test joint space jumps for every waypoint, test them later on the whole path.
    static const JumpThreshold NO_JOINT_SPACE_JUMP_TEST = JumpThreshold::disabled();
    std::vector<RobotStatePtr> waypoint_path;
    double wp_percentage_solved =
        computeCartesianPath(start_state, group, waypoint_path, link, waypoints[i], global_reference_frame, max_step,
                             NO_JOINT_SPACE_JUMP_TEST, validCallback, options, cost_function, link_offset);
    if (fabs(wp_percentage_solved - 1.0) < std::numeric_limits<double>::epsilon())
    {
      percentage_solved = static_cast<double>((i + 1)) / static_cast<double>(waypoints.size());
      std::vector<RobotStatePtr>::iterator start = waypoint_path.begin();
      if (i > 0 && !waypoint_path.empty())
        std::advance(start, 1);
      path.insert(path.end(), start, waypoint_path.end());
    }
    else
    {
      percentage_solved += wp_percentage_solved / static_cast<double>(waypoints.size());
      std::vector<RobotStatePtr>::iterator start = waypoint_path.begin();
      if (i > 0 && !waypoint_path.empty())
        std::advance(start, 1);
      path.insert(path.end(), start, waypoint_path.end());
      break;
    }
  }

  percentage_solved *= checkJointSpaceJump(group, path, jump_threshold);

  return CartesianInterpolator::Percentage(percentage_solved);
}

CartesianInterpolator::Percentage CartesianInterpolator::checkJointSpaceJump(const JointModelGroup* group,
                                                                             std::vector<RobotStatePtr>& path,
                                                                             const JumpThreshold& jump_threshold)
{
  std::optional<int> jump_index = hasJointSpaceJump(path, *group, jump_threshold);

  double percentage_solved = 1.0;
  if (jump_index.has_value())
  {
    percentage_solved = static_cast<double>(*jump_index) / static_cast<double>(path.size());
    // Truncate the path at the index right before the jump.
    path.resize(jump_index.value());
  }

  return CartesianInterpolator::Percentage(percentage_solved);
}

std::optional<int> hasJointSpaceJump(const std::vector<moveit::core::RobotStatePtr>& waypoints,
                                     const moveit::core::JointModelGroup& group,
                                     const moveit::core::JumpThreshold& jump_threshold)
{
  if (waypoints.size() <= 1)
  {
    return std::nullopt;
  }

  if (jump_threshold.relative_factor > 0.0)
  {
    return hasRelativeJointSpaceJump(waypoints, group, jump_threshold.relative_factor);
  }

  if (jump_threshold.revolute > 0.0 || jump_threshold.prismatic > 0.0)
  {
    return hasAbsoluteJointSpaceJump(waypoints, group, jump_threshold.revolute, jump_threshold.prismatic);
  }

  return std::nullopt;
}

}  // end of namespace moveit::core
