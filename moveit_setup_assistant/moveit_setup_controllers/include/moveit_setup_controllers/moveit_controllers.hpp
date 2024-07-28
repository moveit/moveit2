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

#include <moveit_setup_controllers/controllers.hpp>
#include <moveit_setup_controllers/moveit_controllers_config.hpp>

namespace moveit_setup
{
namespace controllers
{
class MoveItControllers : public Controllers
{
public:
  std::string getName() const override
  {
    return "MoveIt Controllers";
  }

  void onInit() override
  {
    config_data_->registerType("moveit_controllers", "moveit_setup::controllers::MoveItControllersConfig");
    srdf_config_ = config_data_->get<SRDFConfig>("srdf");
    controllers_config_ = config_data_->get<MoveItControllersConfig>("moveit_controllers");
  }

  std::string getInstructions() const override
  {
    return "Configure controllers to be used in executing trajectories with MoveIt (hardware or simulation).";
  }

  std::string getButtonText() const override
  {
    return "Auto Add &FollowJointsTrajectory \n Controllers For Each Planning Group";
  }

  std::vector<std::string> getAvailableTypes() const override
  {
    return { "FollowJointTrajectory", "GripperCommand" };
  }

  std::string getDefaultType() const override
  {
    return "FollowJointTrajectory";
  }

  class ActionNamespaceField : public AdditionalControllerField
  {
  public:
    ActionNamespaceField() : AdditionalControllerField("Action Namespace", "action_ns")
    {
    }
    std::string getDefaultValue(const std::string& controller_type) const override
    {
      if (controller_type == "FollowJointTrajectory")
      {
        return "follow_joint_trajectory";
      }
      else if (controller_type == "GripperCommand")
      {
        return "gripper_cmd";
      }
      else
      {
        return "";
      }
    }
  };

  class DefaultField : public AdditionalControllerField
  {
  public:
    DefaultField() : AdditionalControllerField("Default", "default")
    {
    }
    std::string getDefaultValue(const std::string& /*controller_type*/) const override
    {
      return "true";
    }
  };

  FieldPointers getAdditionalControllerFields() const override
  {
    FieldPointers fields;
    fields.push_back(std::make_shared<ActionNamespaceField>());
    fields.push_back(std::make_shared<DefaultField>());
    return fields;
  }
};
}  // namespace controllers
}  // namespace moveit_setup
