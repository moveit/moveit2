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

#include <moveit_setup_controllers/controllers_config.hpp>

namespace moveit_setup
{
namespace controllers
{
static const std::string MOVEIT_CONTROLLERS_YAML = "config/moveit_controllers.yaml";

class MoveItControllersConfig : public ControllersConfig
{
public:
  void loadPrevious(const std::filesystem::path& package_path, const YAML::Node& node) override;

  class GeneratedControllersConfig : public YamlGeneratedFile
  {
  public:
    GeneratedControllersConfig(const std::filesystem::path& package_path, const GeneratedTime& last_gen_time,
                               MoveItControllersConfig& parent)
      : YamlGeneratedFile(package_path, last_gen_time), parent_(parent)
    {
    }

    bool hasChanges() const override
    {
      return parent_.changed_ || parent_.hasChangedGroups();
    }

    std::filesystem::path getRelativePath() const override
    {
      return MOVEIT_CONTROLLERS_YAML;
    }

    std::string getDescription() const override
    {
      return "Creates configurations for moveit_controllers.";
    }

    bool writeYaml(YAML::Emitter& emitter) override;

  protected:
    MoveItControllersConfig& parent_;
  };

  void collectFiles(const std::filesystem::path& package_path, const GeneratedTime& last_gen_time,
                    std::vector<GeneratedFilePtr>& files) override
  {
    files.push_back(std::make_shared<GeneratedControllersConfig>(package_path, last_gen_time, *this));
  }

protected:
  /**
   * Helper function for parsing ros_controllers.yaml file
   * @param YAML::Node - individual controller to be parsed
   * @return true if the file was read correctly
   */
  bool parseController(const std::string& name, const YAML::Node& controller);

  std::map<std::string, YAML::Node> trajectory_parameters_;
};
}  // namespace controllers
}  // namespace moveit_setup
