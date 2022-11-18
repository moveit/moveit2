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

#include <moveit_setup_controllers/included_xacro_config.hpp>
#include <moveit_setup_framework/templates.hpp>

namespace moveit_setup
{
/**
 * @brief A configuration that stores info about modifications to the URDF
 *
 * The modifications are primarily made in an included xacro file (which can be configured dynamically)
 */
class ModifiedUrdfConfig : public SetupConfig
{
public:
  void onInit() override;
  bool isConfigured() const override;

  void loadPrevious(const std::filesystem::path& /*config_package_path*/, const YAML::Node& node) override;
  YAML::Node saveToYaml() const override;

  /**
   * @brief Returns true if this or any of the included xacros have changes, requiring the URDF to be regenerated
   */
  bool hasChanges() const;

  class GeneratedModifiedURDF : public TemplatedGeneratedFile
  {
  public:
    GeneratedModifiedURDF(const std::filesystem::path& package_path, const GeneratedTime& last_gen_time,
                          ModifiedUrdfConfig& parent)
      : TemplatedGeneratedFile(package_path, last_gen_time), parent_(parent)
    {
    }

    std::filesystem::path getRelativePath() const override
    {
      return std::filesystem::path("config") / (parent_.urdf_config_->getRobotName() + ".urdf.xacro");
    }

    std::filesystem::path getTemplatePath() const override
    {
      return getSharePath("moveit_setup_controllers") / "templates" / "config" / "modified.urdf.xacro";
    }

    std::string getDescription() const override
    {
      return "A modified version of the original URDF file with additional interfaces (e.g. ros2_control, simulation)";
    }

    bool hasChanges() const override
    {
      return parent_.hasChanges();
    }

  protected:
    ModifiedUrdfConfig& parent_;
  };

  void collectFiles(const std::filesystem::path& package_path, const GeneratedTime& last_gen_time,
                    std::vector<GeneratedFilePtr>& files) override;
  void collectDependencies(std::set<std::string>& packages) const override;
  void collectVariables(std::vector<TemplateVariable>& variables) override;

protected:
  std::unordered_map<std::string, IncludedXacroConfig::Ptr> getIncludedXacroMap() const
  {
    return config_data_->getAll<IncludedXacroConfig>();
  }

  std::vector<IncludedXacroConfig::Ptr> getIncludedXacros() const
  {
    std::vector<IncludedXacroConfig::Ptr> xacros;
    for (auto& pair : getIncludedXacroMap())
    {
      if (pair.second->isConfigured())
        xacros.push_back(pair.second);
    }
    return xacros;
  }

  std::vector<std::string> getIncludedXacroNames() const
  {
    std::vector<std::string> names;
    for (auto& pair : getIncludedXacroMap())
    {
      if (pair.second->isConfigured())
        names.push_back(pair.first);
    }

    return names;
  }

  std::shared_ptr<URDFConfig> urdf_config_;
  std::set<std::string> cached_xacro_names_;
};
}  // namespace moveit_setup
