/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik Robotics, Inc.
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

#pragma once

#include <moveit_setup_framework/config.hpp>
#include <moveit_setup_framework/data_warehouse.hpp>

namespace moveit_setup
{
static const std::string SETUP_ASSISTANT_FILE = ".setup_assistant";

class PackageSettingsConfig : public SetupConfig
{
public:
  /**
   * @brief Overridden method to load THIS config's data variables.
   *
   * Not to be confused with loadExisting which is a PackageSettingsConfig-specific
   * method for loading ALL configs and their data.
   */
  void loadPrevious(const std::filesystem::path& package_path, const YAML::Node& node) override;
  YAML::Node saveToYaml() const override;

  /**
   * @brief Method for loading the contents of the .setup_assistant file into all the configs
   *
   * @param package_path_or_name Either the path to the MoveIt config package folder OR the name of the package.
   */
  void loadExisting(const std::string& package_path_or_name);

  const std::filesystem::path& getPackagePath() const
  {
    return config_pkg_path_;
  }

  const std::string& getPackageName() const
  {
    return new_package_name_;
  }

  void setPackagePath(const std::filesystem::path& package_path);

  void setPackageName(const std::string& package_name);

  const GeneratedTime& getGenerationTime() const
  {
    return config_pkg_generated_timestamp_;
  }

  bool isConfigured() const override
  {
    return !config_pkg_path_.empty();
  }

  bool hasValidName() const;

  bool hasValidEmail() const;

  class GeneratedSettings : public YamlGeneratedFile
  {
  public:
    GeneratedSettings(const std::filesystem::path& package_path, const GeneratedTime& last_gen_time,
                      PackageSettingsConfig& parent)
      : YamlGeneratedFile(package_path, last_gen_time), parent_(parent)
    {
    }

    std::filesystem::path getRelativePath() const override
    {
      return SETUP_ASSISTANT_FILE;
    }

    std::string getDescription() const override
    {
      return "MoveIt Setup Assistant's hidden settings file. You should not need to edit this file.";
    }

    bool hasChanges() const override
    {
      // We write this file on any changes
      return true;
    }

    bool writeYaml(YAML::Emitter& emitter) override;

  protected:
    PackageSettingsConfig& parent_;
  };

  class GeneratedPackageXML : public TemplatedGeneratedFile
  {
  public:
    GeneratedPackageXML(const std::filesystem::path& package_path, const GeneratedTime& last_gen_time,
                        PackageSettingsConfig& parent)
      : TemplatedGeneratedFile(package_path, last_gen_time), parent_(parent)
    {
    }

    bool hasChanges() const override
    {
      return parent_.author_info_changed_;
    }

    std::filesystem::path getRelativePath() const override
    {
      return "package.xml";
    }

    std::filesystem::path getTemplatePath() const override
    {
      // Note: we call the file package.xml.template so that it isn't automatically indexed by rosprofile
      return getSharePath("moveit_setup_framework") / "templates" / "package.xml.template";
    }

    std::string getDescription() const override
    {
      return "Defines a ROS package";
    }

  protected:
    PackageSettingsConfig& parent_;
  };

  class GeneratedCMake : public TemplatedGeneratedFile
  {
  public:
    using TemplatedGeneratedFile::TemplatedGeneratedFile;

    bool hasChanges() const override
    {
      return false;  // Generally doesn't change
    }

    std::filesystem::path getRelativePath() const override
    {
      return "CMakeLists.txt";
    }

    std::filesystem::path getTemplatePath() const override
    {
      return getSharePath("moveit_setup_framework") / "templates" / "CMakeLists.txt";
    }

    std::string getDescription() const override
    {
      return "CMake build system configuration file";
    }
  };

  void collectFiles(const std::filesystem::path& package_path, const GeneratedTime& last_gen_time,
                    std::vector<GeneratedFilePtr>& files) override
  {
    files.push_back(std::make_shared<GeneratedSettings>(package_path, last_gen_time, *this));
    files.push_back(std::make_shared<GeneratedPackageXML>(package_path, last_gen_time, *this));
    files.push_back(std::make_shared<GeneratedCMake>(package_path, last_gen_time));
  }

  void collectVariables(std::vector<TemplateVariable>& variables) override;

  void loadDependencies();

  std::string getAuthorName() const
  {
    return author_name_;
  }
  std::string getAuthorEmail() const
  {
    return author_email_;
  }
  void setAuthorName(const std::string& name)
  {
    author_name_ = name;
  }
  void setAuthorEmail(const std::string& email)
  {
    author_email_ = email;
  }

  void setGenerationTime();

protected:
  /// Loaded configuration package path - if an existing package was loaded, holds that path
  std::filesystem::path config_pkg_path_;

  /// Name of the new package that is being (or going) to be generated, based on user specified save path
  std::string new_package_name_{ "unnamed_moveit_config" };

  /// Name of the author of this config
  std::string author_name_;

  /// Email of the author of this config
  std::string author_email_;

  bool author_info_changed_{ false };

  /// Timestamp when configuration package was generated, if it was previously generated
  GeneratedTime config_pkg_generated_timestamp_;

  std::set<std::string> package_dependencies_;
};
}  // namespace moveit_setup
