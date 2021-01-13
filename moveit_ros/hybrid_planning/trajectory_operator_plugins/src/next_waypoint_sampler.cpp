/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

/* Author: Sebastian Jahr
 */

#include <moveit/trajectory_operator_plugins/next_waypoint_sampler.h>

#include <moveit/kinematic_constraints/utils.h>

namespace moveit_hybrid_planning
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("local_planner_component");

bool NextWaypointSampler::initialize(const rclcpp::Node::SharedPtr& node, moveit::core::RobotModelConstPtr robot_model,
                                     std::string group_name)
{
  reference_trajectory_.reset(new robot_trajectory::RobotTrajectory(robot_model, group_name));
  index_ = 0;
  return true;
}

bool NextWaypointSampler::addTrajectorySegment(const robot_trajectory::RobotTrajectory& new_trajectory)
{
  if (new_trajectory.getGroupName() != reference_trajectory_->getGroupName())
  {
    RCLCPP_ERROR_STREAM(
        LOGGER,
        "NextWaypointSampler: Group names don't match, reset reference_trajectory!");  // TODO Review reference
                                                                                       // trajectory initialization
    reference_trajectory_.reset(
        new robot_trajectory::RobotTrajectory(new_trajectory.getRobotModel(), new_trajectory.getGroupName()));
  }
  reference_trajectory_->append(
      new_trajectory,
      0.01);  // TODO Remove magic number by interpolation between both end and start point of the trajectories
  return true;
}

std::vector<moveit_msgs::msg::Constraints> NextWaypointSampler::getLocalProblem(moveit::core::RobotState current_state)
{
  moveit::core::RobotState next_desired_goal_state = reference_trajectory_->getWayPoint(index_);
  if (next_desired_goal_state.distance(current_state) <= 0.01)
  {
    index_ += 1;
    if (index_ < reference_trajectory_->getWayPointCount())
    {
      next_desired_goal_state = reference_trajectory_->getWayPoint(index_);
    }
  }
  std::vector<moveit_msgs::msg::Constraints> local_goal_constraints;
  // Construct local trajectory
  for (auto i = 0; i < 3; i++)
  {  // TODO Use param to config window width
    if ((index_ + i) < reference_trajectory_->getWayPointCount())
    {
      moveit::core::RobotState local_robot_state = reference_trajectory_->getWayPoint(index_ + i);
      local_goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(
          local_robot_state, local_robot_state.getJointModelGroup(reference_trajectory_->getGroupName()),
          0.1));  // TODO Remove magic number!
    }
  }
  return local_goal_constraints;
}

double NextWaypointSampler::getTrajectoryProgress(moveit::core::RobotState current_state)
{
  if (index_ >= reference_trajectory_->getWayPointCount() - 1)
  {
    return 1.0;
  }
  return 0.0;
}
}  // namespace moveit_hybrid_planning

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(moveit_hybrid_planning::NextWaypointSampler, moveit_hybrid_planning::TrajectoryOperatorInterface);
