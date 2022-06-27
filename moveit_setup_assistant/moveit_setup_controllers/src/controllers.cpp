/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik Robotics
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
 *   * Neither the name of PickNik Robotics nor the names of its
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

/* Author: David V. Lu!! */

#include <moveit_setup_controllers/controllers.hpp>

namespace moveit_setup
{
namespace controllers
{
// ******************************************************************************************
// Add a Follow Joint Trajectory action Controller for each Planning Group
// ******************************************************************************************
bool Controllers::addDefaultControllers()
{
  std::vector<std::string> group_names = getGroupNames();
  if (group_names.empty())
  {
    return false;
  }

  // Loop through groups
  bool success = true;
  for (const std::string& group_name : group_names)
  {
    // Get list of associated joints
    std::vector<std::string> joint_names = srdf_config_->getJointNames(group_name, true, false);  // exclude passive
    if (joint_names.empty())
    {
      continue;
    }
    bool ret = controllers_config_->addController(group_name + "_controller", getDefaultType(), joint_names);
    success &= ret;
  }

  return success;
}

std::vector<std::string> Controllers::getJointsFromGroups(const std::vector<std::string>& group_names) const
{
  std::vector<std::string> joint_names;
  for (const std::string& group_name : group_names)
  {
    for (const std::string& joint_name : srdf_config_->getJointNames(group_name, true, false))  // exclude passive
    {
      joint_names.push_back(joint_name);
    }
  }
  return joint_names;
}

}  // namespace controllers
}  // namespace moveit_setup
