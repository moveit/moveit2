/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Metro Robots
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
 *   * Neither the name of Metro Robots nor the names of its
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

#include <moveit_setup_framework/config.hpp>
#include <moveit_setup_framework/data_warehouse.hpp>
#include <moveit_setup_framework/templates.hpp>
#include <moveit_setup_framework/utilities.hpp>
#include <moveit_setup_framework/data/srdf_config.hpp>

namespace moveit_setup
{
namespace controllers
{
/**
 * Single controller "instance" configuration
 */
struct ControllerInfo
{
  std::string name_;                 // controller name
  std::string type_;                 // controller type
  std::vector<std::string> joints_;  // joints controlled by this controller
  std::map<std::string, std::string> parameters_;
};

/**
 * @brief All the controller configurations
 */
class ControllersConfig : public SetupConfig
{
public:
  bool isConfigured() const override
  {
    return !controllers_.empty();
  }

  /**
   * \brief Gets controllers_ vector
   */
  std::vector<ControllerInfo>& getControllers()
  {
    return controllers_;
  }

  /**
   * \brief Adds a controller to controllers_ vector
   * \param name Name of the controller
   * \param type type of the controller
   * \param joint_names vector of the joint names
   * \return true if inserted correctly
   */
  bool addController(const std::string& name, const std::string& type, const std::vector<std::string>& joint_names);

  /**
   * \brief Adds a controller to controllers_ vector
   * \param new_controller a new Controller to add
   * \return true if inserted correctly
   */
  bool addController(const ControllerInfo& new_controller);

  /**
   * Find the associated controller by name
   *
   * @param controller_name - name of controller to find in datastructure
   * @return pointer to data in datastructure
   */
  ControllerInfo* findControllerByName(const std::string& controller_name);

  /**
   * Delete controller by name
   *
   * @param controller_name - name of controller to delete
   * @return true if deleted, false if not found
   */
  bool deleteController(const std::string& controller_name);

  bool hasChangedGroups() const
  {
    auto srdf_config = config_data_->get<SRDFConfig>("srdf");
    return srdf_config->getChangeMask() & GROUPS;
  }

protected:
  /// Controllers config data
  std::vector<ControllerInfo> controllers_;
  bool changed_;
};
}  // namespace controllers
}  // namespace moveit_setup
