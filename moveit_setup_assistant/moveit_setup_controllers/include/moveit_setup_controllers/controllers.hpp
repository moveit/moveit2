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

#include <moveit_setup_framework/setup_step.hpp>
#include <moveit_setup_framework/data/srdf_config.hpp>
#include <moveit_setup_controllers/controllers_config.hpp>

namespace moveit_setup
{
namespace controllers
{
/**
 * @brief Structure for containing information about types of additional parameters
 */
class AdditionalControllerField
{
public:
  AdditionalControllerField(const std::string& display_name, const std::string& parameter_name)
    : display_name_(display_name), parameter_name_(parameter_name)
  {
  }

  AdditionalControllerField() = default;
  AdditionalControllerField(const AdditionalControllerField&) = default;
  AdditionalControllerField(AdditionalControllerField&&) = default;
  AdditionalControllerField& operator=(const AdditionalControllerField&) = default;
  AdditionalControllerField& operator=(AdditionalControllerField&&) = default;
  virtual ~AdditionalControllerField() = default;

  /**
   * @brief Overridable method for changing the default value based on the controller_type
   */
  virtual std::string getDefaultValue(const std::string& /*controller_type*/) const
  {
    return "";
  }

  std::string display_name_;
  std::string parameter_name_;
};

/// Convenience alias
using FieldPointers = std::vector<std::shared_ptr<AdditionalControllerField>>;

class Controllers : public SetupStep
{
public:
  virtual std::string getInstructions() const = 0;

  virtual std::string getButtonText() const = 0;

  virtual std::vector<std::string> getAvailableTypes() const = 0;

  virtual std::string getDefaultType() const = 0;

  /**
   * @brief Define the types of controller fields for the specific types of controllers
   */
  virtual FieldPointers getAdditionalControllerFields() const
  {
    return FieldPointers();
  }

  bool isReady() const override
  {
    return !srdf_config_->getGroups().empty();
  }

  const std::vector<std::string>& getJointNames() const
  {
    return srdf_config_->getRobotModel()->getJointModelNames();
  }

  std::vector<std::string> getGroupNames() const
  {
    return srdf_config_->getGroupNames();
  }

  std::string getChildOfJoint(const std::string& joint_name) const
  {
    return srdf_config_->getChildOfJoint(joint_name);
  }

  std::vector<ControllerInfo>& getControllers() const
  {
    return controllers_config_->getControllers();
  }

  bool addController(const ControllerInfo& new_controller)
  {
    return controllers_config_->addController(new_controller);
  }

  ControllerInfo* findControllerByName(const std::string& controller_name)
  {
    return controllers_config_->findControllerByName(controller_name);
  }

  bool deleteController(const std::string& controller_name)
  {
    return controllers_config_->deleteController(controller_name);
  }

  bool addDefaultControllers();

  std::vector<std::string> getJointsFromGroups(const std::vector<std::string>& group_names) const;

protected:
  std::shared_ptr<SRDFConfig> srdf_config_;
  std::shared_ptr<ControllersConfig> controllers_config_;
};
}  // namespace controllers
}  // namespace moveit_setup
