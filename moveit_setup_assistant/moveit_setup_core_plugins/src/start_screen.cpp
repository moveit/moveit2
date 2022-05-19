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

#include <moveit_setup_core_plugins/start_screen.hpp>

namespace moveit_setup
{
namespace core
{
void StartScreen::onInit()
{
  package_settings_ = config_data_->get<PackageSettingsConfig>("package_settings");
  srdf_config_ = config_data_->get<SRDFConfig>("srdf");
  urdf_config_ = config_data_->get<URDFConfig>("urdf");
}

void StartScreen::loadURDFFile(const std::filesystem::path& urdf_file_path, const std::string& xacro_args)
{
  urdf_config_->loadFromPath(urdf_file_path, xacro_args);
  srdf_config_->updateRobotModel();
}

std::filesystem::path StartScreen::getURDFPath()
{
  return urdf_config_->getURDFPath();
}

std::string StartScreen::getXacroArgs()
{
  return urdf_config_->getXacroArgs();
}

std::filesystem::path StartScreen::getPackagePath()
{
  return package_settings_->getPackagePath();
}

void StartScreen::loadExisting(const std::filesystem::path& package_path)
{
  package_settings_->loadExisting(package_path);
}

bool StartScreen::isXacroFile()
{
  return urdf_config_->isXacroFile();
}
}  // namespace core
}  // namespace moveit_setup
