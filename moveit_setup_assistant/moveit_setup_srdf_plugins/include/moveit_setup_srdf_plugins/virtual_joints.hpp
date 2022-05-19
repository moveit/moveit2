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
#pragma once

#include <moveit_setup_srdf_plugins/srdf_step.hpp>
#include <moveit_setup_framework/data/urdf_config.hpp>

namespace moveit_setup
{
namespace srdf_setup
{
class VirtualJoints : public SuperSRDFStep<srdf::Model::VirtualJoint>
{
public:
  std::string getName() const override
  {
    return "Virtual Joints";
  }

  std::vector<srdf::Model::VirtualJoint>& getContainer() override
  {
    return srdf_config_->getVirtualJoints();
  }

  InformationFields getInfoField() const override
  {
    return VIRTUAL_JOINTS;
  }

  void onInit() override;

  void setProperties(srdf::Model::VirtualJoint* vj, const std::string& parent_name, const std::string& child_name,
                     const std::string& joint_type);

  std::vector<std::string> getLinkNames() const
  {
    return srdf_config_->getLinkNames();
  }

protected:
  std::shared_ptr<URDFConfig> urdf_config_;
};
}  // namespace srdf_setup
}  // namespace moveit_setup
