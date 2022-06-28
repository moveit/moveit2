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

#include <moveit_setup_controllers/controllers_config.hpp>
#include <moveit_setup_framework/data/srdf_config.hpp>
#include <moveit_setup_framework/data_warehouse.hpp>

namespace moveit_setup
{
namespace controllers
{
bool ControllersConfig::addController(const std::string& name, const std::string& type,
                                      const std::vector<std::string>& joint_names)
{
  ControllerInfo controller;
  controller.name_ = name;
  controller.type_ = type;
  controller.joints_ = joint_names;
  return addController(controller);
}

bool ControllersConfig::addController(const ControllerInfo& new_controller)
{
  // Used for holding our search results
  ControllerInfo* searched_ros_controller = nullptr;

  // Find if there is an existing controller with the same name
  searched_ros_controller = findControllerByName(new_controller.name_);

  if (searched_ros_controller && searched_ros_controller->type_ == new_controller.type_)
    return false;

  controllers_.push_back(new_controller);
  return true;
}

ControllerInfo* ControllersConfig::findControllerByName(const std::string& controller_name)
{
  // Find the Controller we are editing based on the Controller name string
  ControllerInfo* searched_ros_controller = nullptr;  // used for holding our search results

  for (ControllerInfo& ros_control_config : controllers_)
  {
    if (ros_control_config.name_ == controller_name)  // string match
    {
      searched_ros_controller = &ros_control_config;  // convert to pointer from iterator
      break;                                          // we are done searching
    }
  }

  return searched_ros_controller;
}

bool ControllersConfig::deleteController(const std::string& controller_name)
{
  for (std::vector<ControllerInfo>::iterator controller_it = controllers_.begin(); controller_it != controllers_.end();
       ++controller_it)
  {
    if (controller_it->name_ == controller_name)  // string match
    {
      controllers_.erase(controller_it);
      // we are done searching
      return true;
    }
  }
  return false;
}
}  // namespace controllers
}  // namespace moveit_setup
