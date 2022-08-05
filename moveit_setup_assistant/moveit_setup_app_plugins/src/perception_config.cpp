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

#include <moveit_setup_app_plugins/perception_config.hpp>

namespace moveit_setup
{
namespace app
{
// ******************************************************************************************
// Loads sensors_3d yaml file
// ******************************************************************************************
void PerceptionConfig::loadPrevious(const std::filesystem::path& package_path, const YAML::Node& /*node*/)
{
  // Loads parameters values from sensors_3d yaml file if available
  std::filesystem::path sensors_3d_yaml_path = package_path / "config/sensors_3d.yaml";
  if (!std::filesystem::is_regular_file(sensors_3d_yaml_path))
  {
    sensors_3d_yaml_path = getSharePath("moveit_setup_app_plugins") / "templates" / "config/sensors_3d.yaml";
  }
  try
  {
    sensors_plugin_config_parameter_list_ = load3DSensorsYAML(sensors_3d_yaml_path);
  }
  catch (const std::runtime_error& e)
  {
    RCLCPP_ERROR_STREAM(*logger_, e.what());
  }
}

// ******************************************************************************************
// Input sensors_3d yaml file
// ******************************************************************************************
std::vector<SensorParameters> PerceptionConfig::load3DSensorsYAML(const std::filesystem::path& file_path)
{
  std::vector<SensorParameters> config;
  // Is there a sensors config in the package?
  if (file_path.empty())
    return config;

  // Load file
  std::ifstream input_stream(file_path);
  if (!input_stream.good())
  {
    throw std::runtime_error("Unable to open file for reading " + file_path.string());
  }

  // Begin parsing
  try
  {
    const YAML::Node& doc = YAML::Load(input_stream);
    // Get sensors node
    const YAML::Node& sensors_node = doc["sensors"];

    // Make sure that the sensors are written as a sequence
    if (sensors_node && sensors_node.IsSequence())
    {
      // Loop over the sensors available in the file
      for (const YAML::Node& sensor_name_node : sensors_node)
      {
        std::string sensor_name = sensor_name_node.as<std::string>();
        const YAML::Node& sensor = doc[sensor_name];

        SensorParameters sensor_map;
        sensor_map["name"] = sensor_name;
        for (YAML::const_iterator sensor_it = sensor.begin(); sensor_it != sensor.end(); ++sensor_it)
        {
          sensor_map[sensor_it->first.as<std::string>()] = sensor_it->second.as<std::string>();
        }

        config.push_back(sensor_map);
      }
    }
    return config;
  }
  catch (YAML::ParserException& e)  // Catch errors, rethrow as runtime_error
  {
    throw std::runtime_error(std::string("Error parsing sensors yaml: ") + e.what());
  }
}

// ******************************************************************************************
// Used to clear sensor plugin configuration parameter list
// ******************************************************************************************
void PerceptionConfig::clearSensorPluginConfig()
{
  if (sensors_plugin_config_parameter_list_.empty())
  {
    return;
  }
  changed_ = true;
  sensors_plugin_config_parameter_list_.clear();
}

// ******************************************************************************************
// Used to add a sensor plugin configuration parameter to the sensor plugin configuration parameter list
// ******************************************************************************************
void PerceptionConfig::setConfig(const SensorParameters& parameters)
{
  changed_ = true;
  if (sensors_plugin_config_parameter_list_.empty())
  {
    sensors_plugin_config_parameter_list_.push_back(parameters);
  }
  else
  {
    // Right now, the widget only supports editing one plugin
    sensors_plugin_config_parameter_list_[0] = parameters;
  }
  sensors_plugin_config_parameter_list_[0]["name"] = "default_sensor";
}

// ******************************************************************************************
// Output 3D Sensor configuration file
// ******************************************************************************************
bool PerceptionConfig::GeneratedSensorConfig::writeYaml(YAML::Emitter& emitter)
{
  emitter << YAML::BeginMap;
  emitter << YAML::Key << "sensors";
  emitter << YAML::BeginSeq;
  for (auto& sensor_config : parent_.sensors_plugin_config_parameter_list_)
  {
    emitter << YAML::Value << sensor_config["name"];
  }
  emitter << YAML::EndSeq;
  for (auto& sensor_config : parent_.sensors_plugin_config_parameter_list_)
  {
    emitter << YAML::Key << sensor_config["name"];
    emitter << YAML::BeginMap;
    for (auto& parameter : sensor_config)
    {
      if (parameter.first == "name")
      {
        continue;
      }
      emitter << YAML::Key << parameter.first;
      emitter << YAML::Value << parameter.second;
    }
    emitter << YAML::EndMap;
  }
  emitter << YAML::EndMap;
  return true;
}
}  // namespace app
}  // namespace moveit_setup

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(moveit_setup::app::PerceptionConfig, moveit_setup::SetupConfig)
