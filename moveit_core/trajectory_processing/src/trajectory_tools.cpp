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

// Takes the time differences, and updates the timestamps, velocities and accelerations
// in the trajectory.
void updateTrajectory(robot_trajectory::RobotTrajectory& rob_trajectory, const std::vector<double>& time_diff)
{
  // Error check
  if (time_diff.empty())
    return;

  double time_sum = 0.0;

  moveit::core::RobotStatePtr prev_waypoint;
  moveit::core::RobotStatePtr curr_waypoint;
  moveit::core::RobotStatePtr next_waypoint;

  const moveit::core::JointModelGroup* group = rob_trajectory.getGroup();
  const std::vector<std::string>& vars = group->getVariableNames();
  const std::vector<int>& idx = group->getVariableIndexList();

  int num_points = rob_trajectory.getWayPointCount();

  rob_trajectory.setWayPointDurationFromPrevious(0, time_sum);

  // Times
  for (int i = 1; i < num_points; ++i)
    // Update the time between the waypoints in the robot_trajectory.
    rob_trajectory.setWayPointDurationFromPrevious(i, time_diff[i - 1]);

  // Return if there is only one point in the trajectory!
  if (num_points <= 1)
    return;

  // Accelerations
  for (int i = 0; i < num_points; ++i)
  {
    curr_waypoint = rob_trajectory.getWayPointPtr(i);

    if (i > 0)
      prev_waypoint = rob_trajectory.getWayPointPtr(i - 1);

    if (i < num_points - 1)
      next_waypoint = rob_trajectory.getWayPointPtr(i + 1);

    for (std::size_t j = 0; j < vars.size(); ++j)
    {
      double q1;
      double q2;
      double q3;
      double dt1;
      double dt2;

      if (i == 0)
      {
        // First point
        q1 = next_waypoint->getVariablePosition(idx[j]);
        q2 = curr_waypoint->getVariablePosition(idx[j]);
        q3 = q1;

        dt1 = dt2 = time_diff[i];
      }
      else if (i < num_points - 1)
      {
        // middle points
        q1 = prev_waypoint->getVariablePosition(idx[j]);
        q2 = curr_waypoint->getVariablePosition(idx[j]);
        q3 = next_waypoint->getVariablePosition(idx[j]);

        dt1 = time_diff[i - 1];
        dt2 = time_diff[i];
      }
      else
      {
        // last point
        q1 = prev_waypoint->getVariablePosition(idx[j]);
        q2 = curr_waypoint->getVariablePosition(idx[j]);
        q3 = q1;

        dt1 = dt2 = time_diff[i - 1];
      }

      double v1, v2, a;

      bool start_velocity = false;
      if (dt1 == 0.0 || dt2 == 0.0)
      {
        v1 = 0.0;
        v2 = 0.0;
        a = 0.0;
      }
      else
      {
        if (i == 0)
        {
          if (curr_waypoint->hasVelocities())
          {
            start_velocity = true;
            v1 = curr_waypoint->getVariableVelocity(idx[j]);
          }
        }
        v1 = start_velocity ? v1 : (q2 - q1) / dt1;
        // v2 = (q3-q2)/dt2;
        v2 = start_velocity ? v1 : (q3 - q2) / dt2;  // Needed to ensure continuous velocity for first point
        a = 2.0 * (v2 - v1) / (dt1 + dt2);
      }

      curr_waypoint->setVariableVelocity(idx[j], (v2 + v1) / 2.0);
      curr_waypoint->setVariableAcceleration(idx[j], a);
    }
  }
}
}  // namespace trajectory_processing
