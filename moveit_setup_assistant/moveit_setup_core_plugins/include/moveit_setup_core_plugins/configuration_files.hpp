/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

#pragma once
#include <moveit_setup_framework/setup_step.hpp>
#include <moveit_setup_framework/data/package_settings_config.hpp>

namespace moveit_setup
{
namespace core
{
class ConfigurationFiles : public SetupStep
{
public:
  std::string getName() const override
  {
    return "Configuration Files";
  }

  void onInit() override;

  const std::filesystem::path& getPackagePath()
  {
    return package_settings_->getPackagePath();
  }

  void setPackagePath(const std::filesystem::path& package_path)
  {
    package_settings_->setPackagePath(package_path);
  }

  void setPackageName(const std::string& package_name)
  {
    package_settings_->setPackageName(package_name);
  }

  /// Populate the 'Files to be generated' list
  void loadFiles();

  const std::vector<GeneratedFilePtr> getGeneratedFiles() const
  {
    return gen_files_;
  }

  unsigned int getNumFiles() const
  {
    return gen_files_.size();
  }

  bool shouldGenerate(const GeneratedFilePtr& file) const
  {
    std::string rel_path = file->getRelativePath();
    auto it = should_generate_.find(rel_path);
    if (it == should_generate_.end())
    {
      return true;
    }
    return it->second;
  }

  bool hasModifiedFiles() const
  {
    return hasMatchingFileStatus(FileStatus::EXTERNALLY_MODIFIED);
  }

  bool hasConflictingFiles() const
  {
    return hasMatchingFileStatus(FileStatus::CONFLICTED);
  }

  void setShouldGenerate(const std::string& rel_path, bool should_generate);

  bool isExistingConfig();
  bool hasSetupAssistantFile();

  void loadTemplateVariables();

  std::vector<std::string> getIncompleteWarnings() const;

  std::string getInvalidGroupName() const;

  void setGenerationTime()
  {
    package_settings_->setGenerationTime();
  }

protected:
  bool hasMatchingFileStatus(FileStatus status) const;

  // ******************************************************************************************
  // Variables
  // ******************************************************************************************
  /// Vector of all files to be generated
  std::vector<GeneratedFilePtr> gen_files_;

  std::unordered_map<std::string, bool> should_generate_;

  std::shared_ptr<PackageSettingsConfig> package_settings_;
};
}  // namespace core
}  // namespace moveit_setup
