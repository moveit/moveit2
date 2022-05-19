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

#include <moveit_setup_srdf_plugins/virtual_joints.hpp>

namespace moveit_setup_srdf_plugins
{
void VirtualJoints::onInit()
{
  srdf_config_ = config_data_->get<moveit_setup_framework::SRDFConfig>("srdf");
  urdf_config_ = config_data_->get<moveit_setup_framework::URDFConfig>("urdf");
}

// ******************************************************************************************
// Find the associated data by name
// ******************************************************************************************
srdf::Model::VirtualJoint* VirtualJoints::findVJointByName(const std::string& name)
{
  // Find the group state we are editing based on the vjoint name
  srdf::Model::VirtualJoint* searched_group = nullptr;  // used for holding our search results

  for (srdf::Model::VirtualJoint& virtual_joint : srdf_config_->getVirtualJoints())
  {
    if (virtual_joint.name_ == name)  // string match
    {
      searched_group = &virtual_joint;  // convert to pointer from iterator
      break;                            // we are done searching
    }
  }

  return searched_group;
}

bool VirtualJoints::deleteByName(const std::string& name)
{
  auto vjs = srdf_config_->getVirtualJoints();

  for (std::vector<srdf::Model::VirtualJoint>::iterator vjoint_it = vjs.begin(); vjoint_it != vjs.end(); ++vjoint_it)
  {
    // check if this is the group we want to delete
    if (vjoint_it->name_ == name)  // string match
    {
      vjs.erase(vjoint_it);
      srdf_config_->updateRobotModel(true);
      return true;
    }
  }
  return false;
}

void VirtualJoints::create(const std::string& old_name, const std::string& joint_name, const std::string& parent_name,
                           const std::string& child_name, const std::string& joint_type)
{
  srdf::Model::VirtualJoint* vj = nullptr;
  std::vector<srdf::Model::VirtualJoint>& vjs = srdf_config_->getVirtualJoints();
  if (old_name.empty())
  {
    // Create new
    vjs.push_back(srdf::Model::VirtualJoint());
    vj = &vjs.back();
  }
  else
  {
    // Find the group we are editing based on the old name string
    vj = findVJointByName(old_name);
    if (old_name != joint_name && findVJointByName(joint_name))
    {
      throw std::runtime_error("A virtual joint already exists with that name!");
    }
  }
  // Copy name data ----------------------------------------------------
  vj->name_ = joint_name;
  vj->parent_frame_ = parent_name;
  vj->child_link_ = child_name;
  vj->type_ = joint_type;
  srdf_config_->updateRobotModel(true);
}
}  // namespace moveit_setup_srdf_plugins
