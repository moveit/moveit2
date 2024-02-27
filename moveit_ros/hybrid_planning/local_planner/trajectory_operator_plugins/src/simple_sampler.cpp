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

#include <moveit/trajectory_operator_plugins/simple_sampler.h>

#include <moveit/kinematic_constraints/utils.h>

namespace moveit::hybrid_planning
{
namespace
{
constexpr double WAYPOINT_RADIAN_TOLERANCE = 0.2;  // rad: L1-norm sum for all joints
}  // namespace

bool SimpleSampler::initialize([[maybe_unused]] const rclcpp::Node::SharedPtr& node,
                               const moveit::core::RobotModelConstPtr& robot_model, const std::string& group_name)
{
  reference_trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, group_name);
  next_waypoint_index_ = 0;
  joint_group_ = robot_model->getJointModelGroup(group_name);
  return true;
}

moveit_msgs::action::LocalPlanner::Feedback
SimpleSampler::addTrajectorySegment(const robot_trajectory::RobotTrajectory& new_trajectory)
{
  // Reset trajectory operator to delete old reference trajectory
  reset();

  // Throw away old reference trajectory and use trajectory update
  reference_trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(new_trajectory);

  // Parametrize trajectory and calculate velocity and accelerations
  time_parametrization_.computeTimeStamps(*reference_trajectory_);

  // Return empty feedback
  return feedback_;
}

bool SimpleSampler::reset()
{
  // Reset index
  next_waypoint_index_ = 0;
  reference_trajectory_->clear();
  return true;
}
moveit_msgs::action::LocalPlanner::Feedback
SimpleSampler::getLocalTrajectory(const moveit::core::RobotState& current_state,
                                  robot_trajectory::RobotTrajectory& local_trajectory)
{
  if (reference_trajectory_->getWayPointCount() == 0)
  {
    feedback_.feedback = "unhandled_exception";
    return feedback_;
  }

  // Delete previous local trajectory
  local_trajectory.clear();

  // Get next desired robot state
  const moveit::core::RobotState next_desired_goal_state = reference_trajectory_->getWayPoint(next_waypoint_index_);

  // Check if state is reached
  if (next_desired_goal_state.distance(current_state, joint_group_) <= WAYPOINT_RADIAN_TOLERANCE)
  {
    // Update index (and thus desired robot state)
    next_waypoint_index_ = std::min(next_waypoint_index_ + 1, reference_trajectory_->getWayPointCount() - 1);
  }

  // Construct local trajectory containing the next global trajectory waypoint
  local_trajectory.addSuffixWayPoint(reference_trajectory_->getWayPoint(next_waypoint_index_),
                                     reference_trajectory_->getWayPointDurationFromPrevious(next_waypoint_index_));

  // Return empty feedback
  return feedback_;
}

double SimpleSampler::getTrajectoryProgress([[maybe_unused]] const moveit::core::RobotState& current_state)
{
  // Check if trajectory is unwinded
  if (next_waypoint_index_ >= reference_trajectory_->getWayPointCount() - 1)
  {
    return 1.0;
  }
  return 0.0;
}
}  // namespace moveit::hybrid_planning

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(moveit::hybrid_planning::SimpleSampler, moveit::hybrid_planning::TrajectoryOperatorInterface);
