/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#include <moveit/trajectory_processing/trajectory_tools.h>
#include <sstream>

namespace trajectory_processing
{
bool isTrajectoryEmpty(const moveit_msgs::msg::RobotTrajectory& trajectory)
{
  return trajectory.joint_trajectory.points.empty() && trajectory.multi_dof_joint_trajectory.points.empty();
}

std::size_t trajectoryWaypointCount(const moveit_msgs::msg::RobotTrajectory& trajectory)
{
  return std::max(trajectory.joint_trajectory.points.size(), trajectory.multi_dof_joint_trajectory.points.size());
}

std::string toString(const robot_trajectory::RobotTrajectory& trajectory, bool print_jerk)
{
  const moveit::core::JointModelGroup* group = trajectory.getGroup();
  const std::vector<int>& idx = group->getVariableIndexList();
  unsigned int count = trajectory.getWayPointCount();

  if (count == 0)
  {
    return "Empty trajectory.";
  }

  std::stringstream ss;
  ss << "Trajectory has " << count << " points over " << trajectory.getWayPointDurationFromStart(count - 1)
     << " seconds.\n";
  for (unsigned i = 0; i < count; i++)
  {
    const moveit::core::RobotState& point = trajectory.getWayPoint(i);
    ss << "  waypoint " << i << "\t";
    ss << "time " << trajectory.getWayPointDurationFromStart(i);
    ss << " pos ";
    for (int index : idx)
    {
      ss << point.getVariablePosition(index) << " ";
    }
    ss << "vel ";
    for (int index : idx)
    {
      ss << point.getVariableVelocity(index) << " ";
    }
    ss << "acc ";
    for (int index : idx)
    {
      ss << point.getVariableAcceleration(index) << " ";
    }
    if (print_jerk && i > 0)
    {
      const moveit::core::RobotState& prev = trajectory.getWayPoint(i - 1);
      ss << "jrk ";
      double dt = trajectory.getWayPointDurationFromStart(i) - trajectory.getWayPointDurationFromStart(i - 1);
      for (int index : idx)
      {
        ss << (point.getVariableAcceleration(index) - prev.getVariableAcceleration(index)) / dt << " ";
      }
    }
    ss << "\n";
  }
  return ss.str();
}

}  // namespace trajectory_processing
