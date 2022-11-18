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

#include <moveit_setup_controllers/controllers.hpp>
#include <moveit_setup_controllers/ros2_controllers_config.hpp>

namespace moveit_setup
{
namespace controllers
{
class ROS2Controllers : public Controllers
{
public:
  std::string getName() const override
  {
    return "ROS 2 Controllers";
  }

  void onInit() override
  {
    config_data_->registerType("ros2_controllers", "moveit_setup::controllers::ROS2ControllersConfig");
    config_data_->registerType("control_xacro", "moveit_setup::controllers::ControlXacroConfig");
    srdf_config_ = config_data_->get<SRDFConfig>("srdf");
    controllers_config_ = config_data_->get<ROS2ControllersConfig>("ros2_controllers");
  }

  std::string getInstructions() const override
  {
    return "Configure ros2_controllers. By default, ros2_control fake_components are used to create a simple "
           "simulation.";
  }

  std::string getButtonText() const override
  {
    return "Auto Add &JointTrajectoryController \n Controllers For Each Planning Group";
  }

  std::vector<std::string> getAvailableTypes() const override
  {
    return { "joint_trajectory_controller/JointTrajectoryController", "position_controllers/GripperActionController" };
  }

  std::string getDefaultType() const override
  {
    return "joint_trajectory_controller/JointTrajectoryController";
  }
};
}  // namespace controllers
}  // namespace moveit_setup
