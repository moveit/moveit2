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
#pragma once

#include <moveit_setup_framework/config.hpp>
#include <moveit_setup_framework/data_warehouse.hpp>

namespace moveit_setup_app_plugins
{
using SensorParameters = std::map<std::string, std::string>;

class PerceptionConfig : public moveit_setup_framework::SetupConfig
{
public:
  void loadPrevious(const std::string& package_path, const YAML::Node& node) override;

  bool isConfigured() const override
  {
    return !sensors_plugin_config_parameter_list_.empty();
  }

  /**
   * \brief Used for adding a sensor plugin configuation parameter to the sensor plugin configuration parameter list
   */
  std::vector<SensorParameters>& getSensorPluginConfig()
  {
    return sensors_plugin_config_parameter_list_;
  }

  /**
   * \brief Clear the sensor plugin configuration parameter list
   */
  void clearSensorPluginConfig();

  void setConfig(const SensorParameters& parameters);

  class GeneratedSensorConfig : public moveit_setup_framework::YamlGeneratedFile
  {
  public:
    GeneratedSensorConfig(const std::string& package_path, const std::time_t& last_gen_time, PerceptionConfig& parent)
      : YamlGeneratedFile(package_path, last_gen_time), parent_(parent)
    {
    }

    bool hasChanges() const override
    {
      return parent_.changed_;
    }

    std::string getRelativePath() const override
    {
      return "config/sensors_3d.yaml";
    }

    std::string getDescription() const override
    {
      return "Configuration for perception with 3d sensors.";
    }

    bool writeYaml(YAML::Emitter& emitter) override;

  protected:
    PerceptionConfig& parent_;
  };

  void collectFiles(const std::string& package_path, const std::time_t& last_gen_time,
                    std::vector<moveit_setup_framework::GeneratedFilePtr>& files) override
  {
    files.push_back(std::make_shared<GeneratedSensorConfig>(package_path, last_gen_time, *this));
  }

protected:
  /**
   * Input sensors_3d file - contains 3d sensors config data
   *
   * @param file_path path to sensors_3d yaml file in the config package
   * @return true if the file was read correctly
   */
  bool input3DSensorsYAML(const std::string& file_path);

  /// Sensor plugin configuration parameter list, each sensor plugin type is a map of string pairs
  std::vector<SensorParameters> sensors_plugin_config_parameter_list_;
  bool changed_{ false };
};
}  // namespace moveit_setup_app_plugins
