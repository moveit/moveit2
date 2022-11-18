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

#include <moveit_setup_srdf_plugins/passive_joints.hpp>

namespace moveit_setup
{
namespace srdf_setup
{
std::vector<std::string> PassiveJoints::getActiveJoints() const
{
  std::vector<std::string> active_joints;

  // Retrieve pointer to the shared kinematic model
  const moveit::core::RobotModelConstPtr& model = srdf_config_->getRobotModel();

  // Get the names of the all joints
  for (const std::string& joint : model->getJointModelNames())
  {
    if (model->getJointModel(joint)->getVariableCount() > 0)
    {
      active_joints.push_back(joint);
    }
  }
  return active_joints;
}

std::vector<std::string> PassiveJoints::getPassiveJoints() const
{
  std::vector<std::string> passive_joints;
  for (const srdf::Model::PassiveJoint& passive_joint : srdf_config_->getPassiveJoints())
  {
    passive_joints.push_back(passive_joint.name_);
  }
  return passive_joints;
}

void PassiveJoints::setPassiveJoints(const std::vector<std::string>& passive_joint_names)
{
  std::vector<srdf::Model::PassiveJoint>& passive_joints = srdf_config_->getPassiveJoints();
  passive_joints.clear();
  for (const std::string& passive_joint : passive_joint_names)
  {
    srdf::Model::PassiveJoint pj;
    pj.name_ = passive_joint;
    passive_joints.push_back(pj);
  }
  srdf_config_->updateRobotModel(PASSIVE_JOINTS);
}

}  // namespace srdf_setup
}  // namespace moveit_setup
