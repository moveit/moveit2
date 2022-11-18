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

#include <moveit_setup_controllers/moveit_controllers_config.hpp>
#include <moveit_setup_framework/data_warehouse.hpp>

namespace moveit_setup
{
namespace controllers
{
void MoveItControllersConfig::loadPrevious(const std::filesystem::path& package_path, const YAML::Node& /*node*/)
{
  changed_ = false;
  controllers_.clear();
  trajectory_parameters_.clear();

  // Load moveit controllers yaml file if available-----------------------------------------------
  std::filesystem::path ros_controllers_yaml_path = package_path / MOVEIT_CONTROLLERS_YAML;
  std::ifstream input_stream(ros_controllers_yaml_path);
  if (!input_stream.good())
  {
    RCLCPP_WARN_STREAM(*logger_, "Does not exist " << ros_controllers_yaml_path);
    return;
  }

  // Begin parsing
  try
  {
    // Used in parsing controllers
    ControllerInfo control_setting;
    YAML::Node doc = YAML::Load(input_stream);
    YAML::Node controllers = doc["moveit_simple_controller_manager"];

    std::vector<std::string> controller_names;
    getYamlProperty(controllers, "controller_names", controller_names);

    // Loop through all controllers
    for (const std::string& controller_name : controller_names)
    {
      const YAML::Node& cnode = controllers[controller_name];
      if (!cnode.IsDefined())
      {
        RCLCPP_WARN_STREAM(*logger_, "Configuration for controller " << controller_name << " does not exist! Ignoring.");
        continue;
      }

      if (!parseController(controller_name, cnode))
      {
        return;
      }
    }

    YAML::Node trajectory_execution = doc["trajectory_execution"];
    if (trajectory_execution.IsDefined() && trajectory_execution.IsMap())
    {
      for (const auto& kv : trajectory_execution)
      {
        trajectory_parameters_[kv.first.as<std::string>()] = kv.second;
      }
    }
  }
  catch (YAML::ParserException& e)  // Catch errors
  {
    RCLCPP_ERROR_STREAM(*logger_, e.what());
  }
}

// ******************************************************************************************
// Helper function for parsing an individual Controller from moveit_controllers yaml file
// ******************************************************************************************
bool MoveItControllersConfig::parseController(const std::string& name, const YAML::Node& controller_node)
{
  // Used in parsing controllers
  ControllerInfo control_setting;
  control_setting.name_ = name;
  getYamlProperty(controller_node, "type", control_setting.type_);
  if (control_setting.type_.empty())
  {
    RCLCPP_ERROR_STREAM(*logger_, "Couldn't parse type for controller " << name << " in moveit_controllers.yaml");
    return false;
  }

  for (const std::string parameter : { "action_ns", "default" })
  {
    if (controller_node[parameter].IsDefined())
    {
      control_setting.parameters_[parameter] = controller_node[parameter].as<std::string>();
    }
  }

  const YAML::Node& joints_node = controller_node["joints"];

  if (joints_node.IsSequence())
  {
    control_setting.joints_ = joints_node.as<std::vector<std::string>>();
  }
  else if (joints_node.IsDefined())
  {
    control_setting.joints_.push_back(joints_node.as<std::string>());
  }
  if (control_setting.joints_.empty())
  {
    RCLCPP_ERROR_STREAM(*logger_, "Couldn't parse joints for controller " << name << " in moveit_controllers.yaml");
    return false;
  }
  // All required fields were parsed correctly
  controllers_.push_back(control_setting);
  return true;
}

// ******************************************************************************************
// Output controllers config files
// ******************************************************************************************
bool MoveItControllersConfig::GeneratedControllersConfig::writeYaml(YAML::Emitter& emitter)
{
  emitter << YAML::Comment("MoveIt uses this configuration for controller management");
  emitter << YAML::Newline;
  emitter << YAML::BeginMap;
  {
    if (!parent_.trajectory_parameters_.empty())
    {
      emitter << YAML::Key << "trajectory_execution" << YAML::Value;
      emitter << YAML::BeginMap;
      for (const auto& kv : parent_.trajectory_parameters_)
      {
        emitter << YAML::Key << kv.first << YAML::Value << kv.second;
      }
      emitter << YAML::EndMap;
    }

    emitter << YAML::Key << "moveit_controller_manager" << YAML::Value
            << "moveit_simple_controller_manager/MoveItSimpleControllerManager";
    emitter << YAML::Newline;
    emitter << YAML::Newline;

    emitter << YAML::Key << "moveit_simple_controller_manager" << YAML::Value;
    emitter << YAML::BeginMap;
    {
      emitter << YAML::Key << "controller_names";
      emitter << YAML::Value;
      emitter << YAML::BeginSeq;
      for (const ControllerInfo& ci : parent_.controllers_)
      {
        emitter << ci.name_;
      }
      emitter << YAML::EndSeq;
      emitter << YAML::Newline;
      emitter << YAML::Newline;

      for (const auto& controller : parent_.controllers_)
      {
        emitter << YAML::Key << controller.name_;
        emitter << YAML::Value;
        emitter << YAML::BeginMap;
        {
          emitter << YAML::Key << "type" << YAML::Value << controller.type_;
          if (controller.type_ == "FollowJointTrajectory")
          {
            emitter << YAML::Key << "action_ns" << YAML::Value << "follow_joint_trajectory";
            emitter << YAML::Key << "default" << YAML::Value << "true";
          }

          // Write joints
          emitter << YAML::Key << "joints";
          emitter << YAML::Value;
          emitter << YAML::BeginSeq;

          // Iterate through the joints
          for (const std::string& joint : controller.joints_)
          {
            emitter << joint;
          }
          emitter << YAML::EndSeq;

          for (const auto& pair : controller.parameters_)
          {
            emitter << YAML::Key << pair.first;
            emitter << YAML::Value << pair.second;
          }
        }
        emitter << YAML::EndMap;
      }
    }
    emitter << YAML::EndMap;
  }
  emitter << YAML::EndMap;

  return true;
}

}  // namespace controllers
}  // namespace moveit_setup

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(moveit_setup::controllers::MoveItControllersConfig, moveit_setup::SetupConfig)
