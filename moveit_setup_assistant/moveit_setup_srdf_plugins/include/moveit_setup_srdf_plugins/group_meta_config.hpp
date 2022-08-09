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
#include <map>
#include <string>

namespace moveit_setup
{
namespace srdf_setup
{
// Default kin solver values
static const double DEFAULT_KIN_SOLVER_SEARCH_RESOLUTION = 0.005;
static const double DEFAULT_KIN_SOLVER_TIMEOUT = 0.005;

/**
 * Planning groups extra data not found in srdf but used in config files
 */
struct GroupMetaData
{
  std::string kinematics_solver_;  // Name of kinematics plugin to use
  double kinematics_solver_search_resolution_{ DEFAULT_KIN_SOLVER_SEARCH_RESOLUTION };  // resolution to use with solver
  double kinematics_solver_timeout_{ DEFAULT_KIN_SOLVER_TIMEOUT };                      // solver timeout
  std::string kinematics_parameters_file_;  // file for additional kinematics parameters
  std::string default_planner_;             // Name of the default planner to use
};

static const std::string KINEMATICS_FILE = "config/kinematics.yaml";

class GroupMetaConfig : public SetupConfig
{
public:
  bool isConfigured() const override;
  void loadPrevious(const std::filesystem::path& package_path, const YAML::Node& node) override;

  void deleteGroup(const std::string& group_name);
  void renameGroup(const std::string& old_group_name, const std::string& new_group_name);

  const GroupMetaData& getMetaData(const std::string& group_name) const;
  void setMetaData(const std::string& group_name, const GroupMetaData& meta_data);

  class GeneratedGroupMetaConfig : public YamlGeneratedFile
  {
  public:
    GeneratedGroupMetaConfig(const std::filesystem::path& package_path, const GeneratedTime& last_gen_time,
                             GroupMetaConfig& parent)
      : YamlGeneratedFile(package_path, last_gen_time), parent_(parent)
    {
    }

    std::filesystem::path getRelativePath() const override
    {
      return KINEMATICS_FILE;
    }

    std::string getDescription() const override
    {
      return "Specifies which kinematic solver plugin to use for each planning group in the SRDF, as well as "
             "the kinematic solver search resolution.";
    }

    bool hasChanges() const override
    {
      return parent_.changed_;
    }

    bool writeYaml(YAML::Emitter& emitter) override;

  protected:
    GroupMetaConfig& parent_;
  };

  void collectFiles(const std::filesystem::path& package_path, const GeneratedTime& last_gen_time,
                    std::vector<GeneratedFilePtr>& files) override
  {
    files.push_back(std::make_shared<GeneratedGroupMetaConfig>(package_path, last_gen_time, *this));
  }

  void collectVariables(std::vector<TemplateVariable>& variables) override;

protected:
  // Helper method with old name that conveniently returns a bool
  bool inputKinematicsYAML(const std::filesystem::path& file_path);

  /// Planning groups extra data not found in srdf but used in config files
  std::map<std::string, GroupMetaData> group_meta_data_;

  GroupMetaData default_values_;

  bool changed_{ false };
};
}  // namespace srdf_setup
}  // namespace moveit_setup
