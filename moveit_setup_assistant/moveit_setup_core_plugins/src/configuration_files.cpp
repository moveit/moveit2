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

#include <moveit_setup_core_plugins/configuration_files.hpp>
#include <moveit_setup_framework/templates.hpp>
#include <moveit_setup_framework/data/srdf_config.hpp>

namespace moveit_setup
{
namespace core
{
void ConfigurationFiles::onInit()
{
  package_settings_ = config_data_->get<PackageSettingsConfig>("package_settings");
}

void ConfigurationFiles::loadTemplateVariables()
{
  auto& variables = TemplatedGeneratedFile::variables;
  variables.clear();
  for (const auto& config : config_data_->getConfigured())
  {
    config->collectVariables(variables);
  }
}

void ConfigurationFiles::loadFiles()
{
  package_settings_->loadDependencies();
  loadTemplateVariables();
  std::filesystem::path package_path = package_settings_->getPackagePath();
  auto gen_time = package_settings_->getGenerationTime();

  // Check if we are 'editing' a prev generated config pkg
  if (package_path.empty())
  {
    return;  // this is not configured yet
  }

  gen_files_.clear();

  for (const auto& config : config_data_->getConfigured())
  {
    config->collectFiles(package_path, gen_time, gen_files_);
  }
}

bool ConfigurationFiles::hasMatchingFileStatus(FileStatus status) const
{
  for (const auto& gen_file : gen_files_)
  {
    if (gen_file->getStatus() == status)
    {
      return true;
    }
  }
  return false;
}

void ConfigurationFiles::setShouldGenerate(const std::string& rel_path, bool should_generate)
{
  should_generate_[rel_path] = should_generate;
}

bool ConfigurationFiles::isExistingConfig()
{
  // if the folder doesn't exist or is empty
  std::filesystem::path package_path(getPackagePath());
  return std::filesystem::is_directory(package_path) && !std::filesystem::is_empty(package_path);
}

bool ConfigurationFiles::hasSetupAssistantFile()
{
  if (!isExistingConfig())
  {
    return true;
  }
  return std::filesystem::is_regular_file(getPackagePath() / SETUP_ASSISTANT_FILE);
}

std::vector<std::string> ConfigurationFiles::getIncompleteWarnings() const
{
  // There may be a better way to generalize this, but for now, we add some manual checks of the srdf/author info
  std::vector<std::string> warnings;

  auto srdf_config = config_data_->get<SRDFConfig>("srdf");
  // Check that at least 1 planning group exists
  if (srdf_config->getGroups().empty())
  {
    warnings.push_back("No robot model planning groups have been created");
  }

  // Check that at least 1 link pair is disabled from collision checking
  if (srdf_config->getDisabledCollisions().empty())
  {
    warnings.push_back("No self-collisions have been disabled");
  }

  // Check that there is at least 1 end effector added
  if (srdf_config->getEndEffectors().empty())
  {
    warnings.push_back("No end effectors have been added");
  }

  // Check that there is at least 1 virtual joint added
  if (srdf_config->getVirtualJoints().empty())
  {
    warnings.push_back("No virtual joints have been added");
  }

  // Check that there is a author name
  if (!package_settings_->hasValidName())
  {
    warnings.push_back("<b>No author name added</b>");
  }

  // Check that email information is filled
  if (!package_settings_->hasValidEmail())
  {
    warnings.push_back("<b>No valid email address added</b>");
  }

  return warnings;
}

std::string ConfigurationFiles::getInvalidGroupName() const
{
  auto srdf_config = config_data_->get<SRDFConfig>("srdf");
  for (const auto& group : srdf_config->getGroups())
  {
    // Whenever 1 of the 4 component types are found, stop checking this group
    if (!group.joints_.empty())
      continue;
    if (!group.links_.empty())
      continue;
    if (!group.chains_.empty())
      continue;
    if (!group.subgroups_.empty())
      continue;
    return group.name_;
  }
  return "";
}

}  // namespace core
}  // namespace moveit_setup
