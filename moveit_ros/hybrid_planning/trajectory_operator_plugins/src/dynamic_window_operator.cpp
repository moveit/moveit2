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

#include <moveit/trajectory_operator_plugins/dynamic_window_operator.h>

#include <moveit/kinematic_constraints/utils.h>

namespace moveit_hybrid_planning
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("local_planner_component");
const double DWA_SCALING_FACTOR = 1.0;

bool DynamicWindowOperator::initialize(const rclcpp::Node::SharedPtr& node,
                                       const moveit::core::RobotModelConstPtr& robot_model,
                                       const std::string& group_name)
{
  // Initialize cycle time based on local planner frequency
  double local_planner_frequency = 0.0;
  node->get_parameter<double>("local_planning_frequency", local_planner_frequency);
  if (local_planner_frequency != 0.0)
  {
    cycle_time_ = 1 / local_planner_frequency;
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Unable to configure Dynamic Window operator because local planner frequency is 0");
    return false;
  }

  reference_trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, group_name);
  index_ = 0;
  return true;
}

bool DynamicWindowOperator::addTrajectorySegment(const robot_trajectory::RobotTrajectory& new_trajectory)
{
  // Throw away old reference trajectory and use trajectory update
  reference_trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(new_trajectory);

  // Parametrize trajectory and calculate velocity and accelerations
  time_parametrization_.computeTimeStamps(*reference_trajectory_);

  // Reset index
  index_ = 0;
  return true;
}

robot_trajectory::RobotTrajectory
DynamicWindowOperator::getLocalTrajectory(const moveit::core::RobotState& current_state)
{
  // Get next desired robot state
  moveit::core::RobotState local_trajectroy_seed = reference_trajectory_->getWayPoint(index_);

  // Check if state reached
  if (local_trajectroy_seed.distance(current_state) <= 0.01)
  {
    // Update index (and thus desired robot state)
    index_ += 1;
  }

  // Calculate worst case stopping time
  double worst_case_stop_time = calculateWorstCaseStopTime(current_state);

  // Construct local trajectory
  robot_trajectory::RobotTrajectory local_trajectory(reference_trajectory_->getRobotModel(),
                                                     reference_trajectory_->getGroupName());

  // Upper limit of the dynamic time window
  double max_duration_from_start =
      (reference_trajectory_->getWayPointDurationFromStart(index_)) + DWA_SCALING_FACTOR * worst_case_stop_time;

  // Add all global trajectory waypoints within the dynamic window to the local trajectory
  for (auto i = index_; i < reference_trajectory_->getWayPointCount(); i++)
  {
    // Check if waypoint is within dynamic time window
    if (reference_trajectory_->getWayPointDurationFromStart(i) > max_duration_from_start)
    {
      break;
    }
    moveit::core::RobotState local_robot_state = reference_trajectory_->getWayPoint(i);
    local_trajectory.addSuffixWayPoint(reference_trajectory_->getWayPoint(i),
                                       reference_trajectory_->getWayPointDurationFromPrevious(i));
  }
  return local_trajectory;
}

double DynamicWindowOperator::getTrajectoryProgress(const moveit::core::RobotState& current_state)
{
  // Check if trajectory is unwinded
  if (index_ >= reference_trajectory_->getWayPointCount() - 1)
  {
    return 1.0;
  }
  return 0.0;
}

double DynamicWindowOperator::calculateWorstCaseStopTime(const moveit::core::RobotState& state)
{
  moveit::core::JointModel::Bounds kinematic_bounds;
  double acceleration_limit = 0;
  double joint_velocity = 0;
  double worst_case_stop_time = 0;

  for (auto joint_model : state.getRobotModel()->getActiveJointModels())
  {
    // Skip virtual joint
    if (joint_model->getName() == "virtual_joint")
    {
      continue;
    }

    // Get bounds
    kinematic_bounds = joint_model->getVariableBounds();

    // Some joints do not have acceleration limits
    if (kinematic_bounds[0].acceleration_bounded_)
    {
      // Be conservative when calculating overall acceleration limit from min and max limits
      acceleration_limit =
          std::min(fabs(kinematic_bounds[0].min_acceleration_), fabs(kinematic_bounds[0].max_acceleration_));
    }
    // Use velocity bounds if no acceleration bounds are available (TODO sjahr: Good idea?)
    else if (kinematic_bounds[0].velocity_bounded_)
    {
      acceleration_limit =
          (std::min(fabs(kinematic_bounds[0].min_velocity_), fabs(kinematic_bounds[0].max_velocity_))) / cycle_time_;
    }
    else
    {
      // Worst stopping time based local trajectory matching is pointless without bounds
      RCLCPP_ERROR(LOGGER, "No acceleration or velocity bounds found joint %s!");
      throw;
    }

    // Get the current joint velocity
    joint_velocity = state.getVariableVelocity(joint_model->getFirstVariableIndex());

    // Calculate worst case stop time
    worst_case_stop_time = std::max(worst_case_stop_time, fabs(joint_velocity / acceleration_limit));
  }
  return worst_case_stop_time;
}
}  // namespace moveit_hybrid_planning

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(moveit_hybrid_planning::DynamicWindowOperator,
                       moveit_hybrid_planning::TrajectoryOperatorInterface);
