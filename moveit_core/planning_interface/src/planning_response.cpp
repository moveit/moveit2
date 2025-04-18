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

#include <moveit/planning_interface/planning_response.hpp>
#include <moveit/robot_state/conversions.hpp>

void planning_interface::MotionPlanResponse::getMessage(moveit_msgs::msg::MotionPlanResponse& msg) const
{
  msg.error_code = error_code;
  msg.planning_time = planning_time;
  if (trajectory && !trajectory->empty())
  {
    moveit::core::robotStateToRobotStateMsg(trajectory->getFirstWayPoint(), msg.trajectory_start);
    trajectory->getRobotTrajectoryMsg(msg.trajectory);
    msg.group_name = trajectory->getGroupName();
  }
}

void planning_interface::MotionPlanDetailedResponse::getMessage(moveit_msgs::msg::MotionPlanDetailedResponse& msg) const
{
  msg.error_code = error_code;

  msg.trajectory.clear();
  msg.description.clear();
  msg.processing_time.clear();

  bool first = true;

  for (std::size_t i = 0; i < trajectory.size(); ++i)
  {
    if (trajectory[i]->empty())
      continue;
    if (first)
    {
      first = false;
      moveit::core::robotStateToRobotStateMsg(trajectory[i]->getFirstWayPoint(), msg.trajectory_start);
      msg.group_name = trajectory[i]->getGroupName();
    }
    msg.trajectory.resize(msg.trajectory.size() + 1);
    trajectory[i]->getRobotTrajectoryMsg(msg.trajectory.back());
    if (description.size() > i)
      msg.description.push_back(description[i]);
    if (processing_time.size() > i)
      msg.processing_time.push_back(processing_time[i]);
  }
}
