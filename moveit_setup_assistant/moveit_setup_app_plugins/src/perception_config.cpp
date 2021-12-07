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

namespace moveit_setup_app_plugins
{
// ******************************************************************************************
// Loads sensors_3d yaml file
// ******************************************************************************************
void PerceptionConfig::loadPrevious(const std::string& package_path, const YAML::Node& node)
{
  // Loads parameters values from sensors_3d yaml file if available
  std::string sensors_3d_yaml_path = moveit_setup_framework::appendPaths(package_path, "config/sensors_3d.yaml");
  if (boost::filesystem::is_regular_file(sensors_3d_yaml_path))
  {
    input3DSensorsYAML(sensors_3d_yaml_path);
  }
  else
  {
    // TODO: Handle defaults
    // fs::path default_sensors_3d_yaml_path = "templates/moveit_config_pkg_template/config/sensors_3d.yaml";
    // Load from default
    // input3DSensorsYAML(default_sensors_3d_yaml_path.make_preferred().string());
  }
}

// ******************************************************************************************
// Input sensors_3d yaml file
// ******************************************************************************************
bool PerceptionConfig::input3DSensorsYAML(const std::string& file_path)
{
  // Load file
  std::ifstream input_stream(file_path.c_str());
  if (!input_stream.good())
  {
    RCLCPP_ERROR_STREAM((*logger_), "Unable to open file for reading " << file_path);
    return false;
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
      SensorParameters sensor_map;
      bool empty_node = true;

      // Loop over the sensors available in the file
      for (const YAML::Node& sensor : sensors_node)
      {
        if (const YAML::Node& sensor_node = sensor)
        {
          for (YAML::const_iterator sensor_it = sensor_node.begin(); sensor_it != sensor_node.end(); ++sensor_it)
          {
            empty_node = false;
            sensor_map[sensor_it->first.as<std::string>()] = sensor_it->second.as<std::string>();
          }
          // Don't push empty nodes
          if (!empty_node)
            sensors_plugin_config_parameter_list_.push_back(sensor_map);
        }
      }
    }
    return true;
  }
  catch (YAML::ParserException& e)  // Catch errors
  {
    RCLCPP_ERROR_STREAM((*logger_), "Error parsing sensors yaml: " << e.what());
  }

  return false;  // if it gets to this point an error has occured
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
// Used to add a sensor plugin configuation parameter to the sensor plugin configuration parameter list
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
}

// ******************************************************************************************
// Output 3D Sensor configuration file
// ******************************************************************************************
bool PerceptionConfig::GeneratedSensorConfig::writeYaml(YAML::Emitter& emitter)
{
  emitter << YAML::BeginMap;
  emitter << YAML::Comment("The name of this file shouldn't be changed, or else the Setup Assistant won't detect it");
  emitter << YAML::Key << "sensors";
  emitter << YAML::Value << YAML::BeginSeq;
  for (auto& sensor_config : parent_.sensors_plugin_config_parameter_list_)
  {
    emitter << YAML::BeginMap;
    for (auto& parameter : sensor_config)
    {
      emitter << YAML::Key << parameter.first;
      emitter << YAML::Value << parameter.second;
    }
    emitter << YAML::EndMap;
  }
  emitter << YAML::EndSeq;
  emitter << YAML::EndMap;
  return true;
}
}  // namespace moveit_setup_app_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(moveit_setup_app_plugins::PerceptionConfig, moveit_setup_framework::SetupConfig)
