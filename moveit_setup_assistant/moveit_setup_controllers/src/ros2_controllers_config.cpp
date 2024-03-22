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

#include <moveit_setup_controllers/ros2_controllers_config.hpp>

namespace moveit_setup
{
namespace controllers
{
void ROS2ControllersConfig::onInit()
{
  control_xacro_config_ = config_data_->get<ControlXacroConfig>("control_xacro");
}

void ROS2ControllersConfig::loadPrevious(const std::filesystem::path& package_path, const YAML::Node& /*node*/)
{
  changed_ = false;
  controllers_.clear();

  // Load ros2 controllers yaml file if available-----------------------------------------------
  std::filesystem::path ros_controllers_yaml_path = package_path / CONTROLLERS_YAML;
  std::ifstream input_stream(ros_controllers_yaml_path);
  if (!input_stream.good())
  {
    RCLCPP_WARN_STREAM(*logger_, "Does not exist " << ros_controllers_yaml_path);
    return;
  }

  // Begin parsing
  try
  {
    YAML::Node config = YAML::Load(input_stream);

    // Parse Controller Manager Config
    const YAML::Node& cm_node = config["controller_manager"]["ros__parameters"];
    if (!cm_node.IsDefined())
    {
      return;
    }
    for (const auto& it : cm_node)
    {
      if (!it.second.IsMap())
        continue;

      const auto& type_node = it.second["type"];
      if (!type_node.IsDefined())
      {
        continue;
      }

      ControllerInfo controller;
      controller.name_ = it.first.as<std::string>();
      controller.type_ = type_node.as<std::string>();

      if (controller.type_ != "joint_state_broadcaster/JointStateBroadcaster")
      {
        controllers_.push_back(controller);
      }
    }

    // ParseIndividual Controller Configs
    for (auto& controller : controllers_)
    {
      const YAML::Node& controller_node = config[controller.name_]["ros__parameters"];
      if (!controller_node.IsDefined() || !controller_node.IsMap())
      {
        continue;
      }
      auto jnode = controller_node["joints"];
      if (jnode.IsDefined() && jnode.IsSequence())
      {
        // Looping through sequences with yaml is sometimes buggy when using iterators
        // so here we use a "classic" loop and disable the clang-tidy rule
        for (std::size_t i = 0; i < jnode.size(); i++)  // NOLINT(modernize-loop-convert)
        {
          controller.joints_.push_back(jnode[i].as<std::string>());
        }
      }

      if (controller.joints_.empty())
      {
        std::string one_joint;
        getYamlProperty(controller_node, "joint", one_joint);
        if (!one_joint.empty())
        {
          controller.joints_.push_back(one_joint);
        }
      }
    }
  }
  catch (YAML::ParserException& e)  // Catch errors
  {
    RCLCPP_ERROR_STREAM(*logger_, e.what());
  }
}

const ControlInterfaces ROS2ControllersConfig::getControlInterfaces(const std::vector<std::string>& joint_names) const
{
  return control_xacro_config_->getControlInterfaces(joint_names);
}

// ******************************************************************************************
// Output controllers config files
// ******************************************************************************************
bool ROS2ControllersConfig::GeneratedControllersConfig::writeYaml(YAML::Emitter& emitter)
{
  emitter << YAML::Comment("This config file is used by ros2_control");
  emitter << YAML::BeginMap;
  {
    // Output controller Manager Config
    emitter << YAML::Key << "controller_manager";
    emitter << YAML::Value;
    emitter << YAML::BeginMap;
    {
      emitter << YAML::Key << "ros__parameters";
      emitter << YAML::Value;
      emitter << YAML::BeginMap;
      {
        emitter << YAML::Key << "update_rate" << YAML::Value << 100;  // Hz
        emitter << YAML::Comment("Hz");
        emitter << YAML::Newline;
        emitter << YAML::Newline;

        // Output Types
        for (const ControllerInfo& ci : parent_.controllers_)
        {
          emitter << YAML::Key << ci.name_;
          emitter << YAML::Value;
          emitter << YAML::BeginMap;
          {
            emitter << YAML::Key << "type" << YAML::Value << ci.type_;
          }
          emitter << YAML::Newline << YAML::Newline;
          emitter << YAML::EndMap;
        }

        emitter << YAML::Key << "joint_state_broadcaster";
        emitter << YAML::Value;
        emitter << YAML::BeginMap;
        {
          emitter << YAML::Key << "type" << YAML::Value << "joint_state_broadcaster/JointStateBroadcaster";
        }
        emitter << YAML::EndMap;
      }
      emitter << YAML::EndMap;
    }
    emitter << YAML::EndMap;
    emitter << YAML::Newline;
    emitter << YAML::Newline;
  }
  // Output Controller Configs
  for (const ControllerInfo& ci : parent_.controllers_)
  {
    emitter << YAML::Key << ci.name_;
    emitter << YAML::Value;
    emitter << YAML::BeginMap;
    {
      emitter << YAML::Key << "ros__parameters";
      emitter << YAML::Value;
      emitter << YAML::BeginMap;
      {
        if (ci.type_ != "position_controllers/GripperActionController")
        {
          emitter << YAML::Key << "joints" << YAML::Value << ci.joints_;
        }
        else
        {
          emitter << YAML::Key << "joint" << YAML::Value << ci.joints_[0];
        }

        if (ci.type_ == "joint_trajectory_controller/JointTrajectoryController")
        {
          const ControlInterfaces interfaces = parent_.getControlInterfaces(ci.joints_);
          emitter << YAML::Key << "command_interfaces" << YAML::Value << interfaces.command_interfaces;
          emitter << YAML::Key << "state_interfaces" << YAML::Value << interfaces.state_interfaces;
          emitter << YAML::Key << "allow_nonzero_velocity_at_trajectory_end" << YAML::Value << true;
        }
      }
      emitter << YAML::EndMap;
    }
    emitter << YAML::EndMap;
  }
  emitter << YAML::EndMap;

  return true;
}

}  // namespace controllers
}  // namespace moveit_setup

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(moveit_setup::controllers::ROS2ControllersConfig, moveit_setup::SetupConfig)
