/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, PickNik Inc.
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

/* Author: Andy Zelenak */

#include "moveit/planning_scene/utilities.hpp"

namespace planning_scene
{

namespace
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_planning_scene.utilities");
}

void poseMsgToEigen(const geometry_msgs::msg::Pose& msg, Eigen::Isometry3d& out)
{
  Eigen::Translation3d translation(msg.position.x, msg.position.y, msg.position.z);
  Eigen::Quaterniond quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  quaternion.normalize();
  out = translation * quaternion;
}

bool readPoseFromText(std::istream& in, Eigen::Isometry3d& pose)
{
  double x, y, z, rx, ry, rz, rw;
  if (!(in >> x >> y >> z))
  {
    RCLCPP_ERROR(LOGGER, "Improperly formatted translation in scene geometry file");
    return false;
  }
  if (!(in >> rx >> ry >> rz >> rw))
  {
    RCLCPP_ERROR(LOGGER, "Improperly formatted rotation in scene geometry file");
    return false;
  }
  pose = Eigen::Translation3d(x, y, z) * Eigen::Quaterniond(rw, rx, ry, rz);
  return true;
}

void writePoseToText(std::ostream& out, const Eigen::Isometry3d& pose)
{
  out << pose.translation().x() << ' ' << pose.translation().y() << ' ' << pose.translation().z() << '\n';
  Eigen::Quaterniond r(pose.linear());
  out << r.x() << ' ' << r.y() << ' ' << r.z() << ' ' << r.w() << '\n';
}

}  // namespace planning_scene
