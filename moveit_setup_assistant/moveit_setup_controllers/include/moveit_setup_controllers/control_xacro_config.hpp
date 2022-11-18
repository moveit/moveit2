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

#include <moveit_setup_controllers/modified_urdf_config.hpp>
#include <moveit_setup_framework/data/srdf_config.hpp>

namespace moveit_setup
{
namespace controllers
{
struct ControlInterfaces
{
  std::vector<std::string> command_interfaces;
  std::vector<std::string> state_interfaces;
};

inline std::vector<std::string> getAvailableInterfaceNames()
{
  return { "position", "velocity", "effort" };
}

class ControlXacroConfig : public IncludedXacroConfig
{
public:
  /**@name SetupConfig overrides */
  /**@{*/
  void onInit() override;
  void loadPrevious(const std::filesystem::path& /*config_package_path*/, const YAML::Node& node) override;
  YAML::Node saveToYaml() const override;
  bool isConfigured() const override;
  /**@}*/

  /**@name IncludedXacroConfig overrides */
  /**@{*/
  std::string getFilepath() const override;
  bool hasChanges() const override;
  std::vector<std::pair<std::string, std::string>> getArguments() const override;
  std::vector<std::string> getCommands() const override;
  /**@}*/

  /**
   * @brief Load the original command interfaces from the original (unmodified) URDF
   *
   * Needs to be run whenever the URDF/SRDF may have changed
   */
  void loadFromDescription();

  bool hasAllControlTagsInOriginal() const;

  /**@name Data access and modification */
  /**@{*/

  const ControlInterfaces& getAvailableControlInterfaces() const
  {
    return available_ci_;
  }

  const ControlInterfaces& getDefaultControlInterfaces() const
  {
    return default_ci_;
  }

  /**
   * @brief Use the specified controller interfaces for all the lacking joints
   */
  void setControlInterfaces(const ControlInterfaces& ci);

  /**
   * @brief Get all the control interfaces for all of the specified joint names
   */
  const ControlInterfaces getControlInterfaces(const std::vector<std::string>& joint_names) const;

  /**@}*/

  /**
   * @brief Return the additional joint xml needed for ros2_control tags
   */
  std::string getJointsXML() const;

  class GeneratedControlHeader : public TemplatedGeneratedFile
  {
  public:
    GeneratedControlHeader(const std::filesystem::path& package_path, const GeneratedTime& last_gen_time,
                           ControlXacroConfig& parent)
      : TemplatedGeneratedFile(package_path, last_gen_time), parent_(parent)
    {
    }

    std::filesystem::path getRelativePath() const override
    {
      return std::filesystem::path("config") / (parent_.urdf_config_->getRobotName() + ".ros2_control.xacro");
    }

    std::filesystem::path getTemplatePath() const override
    {
      return getSharePath("moveit_setup_controllers") / "templates" / "config" / "ros2_control.xacro";
    }

    std::string getDescription() const override
    {
      return "Macro definition for required ros2_control xacro additions.";
    }

    bool hasChanges() const override
    {
      return parent_.hasChanges();
    }

  protected:
    ControlXacroConfig& parent_;
  };

  class GeneratedInitialPositions : public YamlGeneratedFile
  {
  public:
    GeneratedInitialPositions(const std::filesystem::path& package_path, const GeneratedTime& last_gen_time,
                              ControlXacroConfig& parent)
      : YamlGeneratedFile(package_path, last_gen_time), parent_(parent)
    {
    }

    bool hasChanges() const override
    {
      return parent_.hasChanges();
    }

    std::filesystem::path getRelativePath() const override
    {
      return "config/initial_positions.yaml";
    }

    std::string getDescription() const override
    {
      return "Initial positions for ros2_control";
    }

    bool writeYaml(YAML::Emitter& emitter) override;

  protected:
    ControlXacroConfig& parent_;
  };

  void collectFiles(const std::filesystem::path& package_path, const GeneratedTime& last_gen_time,
                    std::vector<GeneratedFilePtr>& files) override
  {
    files.push_back(std::make_shared<GeneratedControlHeader>(package_path, last_gen_time, *this));
    files.push_back(std::make_shared<GeneratedInitialPositions>(package_path, last_gen_time, *this));
  }

  void collectVariables(std::vector<TemplateVariable>& variables) override;

protected:
  void getControlInterfaces(const std::string& joint_name, ControlInterfaces& ci) const;

  std::vector<std::string> joint_names_;  /// A list of all joints used by the current SRDF groups
  std::unordered_map<std::string, ControlInterfaces> original_joint_interfaces_, new_joint_interfaces_;

  ControlInterfaces default_ci_, available_ci_;
  srdf::Model::GroupState initial_group_state_;

  bool changed_;
};
}  // namespace controllers
}  // namespace moveit_setup
