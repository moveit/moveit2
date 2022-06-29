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

#include <moveit_setup_controllers/control_xacro_config.hpp>
#include <moveit_setup_framework/data_warehouse.hpp>
#include <tinyxml2.h>
#include <algorithm>

namespace moveit_setup
{
namespace controllers
{
void ControlXacroConfig::onInit()
{
  IncludedXacroConfig::onInit();
  available_ci_.command_interfaces = getAvailableInterfaceNames();
  available_ci_.state_interfaces = getAvailableInterfaceNames();
  default_ci_.command_interfaces = { "position" };
  default_ci_.state_interfaces = { "position", "velocity" };
}

void ControlXacroConfig::loadPrevious(const std::filesystem::path& /*config_package_path*/, const YAML::Node& node)
{
  if (!node.IsDefined())
  {
    return;
  }

  getYamlProperty(node, "command", default_ci_.command_interfaces);
  getYamlProperty(node, "state", default_ci_.state_interfaces);
  setControlInterfaces(default_ci_);
  changed_ = false;
}

YAML::Node ControlXacroConfig::saveToYaml() const
{
  YAML::Node node;
  node["command"] = default_ci_.command_interfaces;
  node["state"] = default_ci_.state_interfaces;
  return node;
}

bool ControlXacroConfig::isConfigured() const
{
  return !new_joint_interfaces_.empty();
}

std::string ControlXacroConfig::getFilepath() const
{
  return urdf_config_->getRobotName() + ".ros2_control.xacro";
}

bool ControlXacroConfig::hasChanges() const
{
  return changed_;
}

std::vector<std::pair<std::string, std::string>> ControlXacroConfig::getArguments() const
{
  std::vector<std::pair<std::string, std::string>> arguments;
  arguments.push_back(std::make_pair("initial_positions_file", "initial_positions.yaml"));
  return arguments;
}

std::vector<std::string> ControlXacroConfig::getCommands() const
{
  std::string command = "<xacro:";
  command += urdf_config_->getRobotName();
  command += "_ros2_control name=\"FakeSystem\" initial_positions_file=\"$(arg initial_positions_file)\"/>";
  return { command };
}

void getInterfaceNames(const tinyxml2::XMLElement* joint_el, const std::string& element_name,
                       std::vector<std::string>& interface_names)
{
  for (const tinyxml2::XMLElement* el = joint_el->FirstChildElement(element_name.c_str()); el != nullptr;
       el = el->NextSiblingElement())
  {
    interface_names.push_back(el->Attribute("name"));
  }
}

void ControlXacroConfig::loadFromDescription()
{
  // Reset Data
  original_joint_interfaces_.clear();
  joint_names_.clear();

  // Load the joint names for all joints used by the groups
  auto srdf_config = config_data_->get<SRDFConfig>("srdf");
  for (const std::string& group_name : srdf_config->getGroupNames())
  {
    for (const std::string& joint_name : srdf_config->getJointNames(group_name, true, false))  // exclude passive
    {
      if (std::find(joint_names_.begin(), joint_names_.end(), joint_name) == joint_names_.end())
      {
        // This is a new joint, add to list of joint names
        joint_names_.push_back(joint_name);
      }
    }
  }

  // Read the URDF
  tinyxml2::XMLDocument urdf_xml;
  using tinyxml2::XMLElement;

  auto urdf_config = config_data_->get<URDFConfig>("urdf");
  urdf_xml.Parse(urdf_config->getURDFContents().c_str());
  for (XMLElement* control_el = urdf_xml.FirstChildElement("ros2_control"); control_el != nullptr;
       control_el = control_el->NextSiblingElement())
  {
    for (XMLElement* joint_el = control_el->FirstChildElement("joint"); joint_el != nullptr;
         joint_el = joint_el->NextSiblingElement())
    {
      std::string joint_name = joint_el->Attribute("name");

      // Parse the interfaces
      ControlInterfaces ci;
      getInterfaceNames(joint_el, "command_interface", ci.command_interfaces);
      getInterfaceNames(joint_el, "state_interface", ci.state_interfaces);
      original_joint_interfaces_[joint_name] = ci;
    }
  }
  setControlInterfaces(default_ci_);
}

bool ControlXacroConfig::hasAllControlTagsInOriginal() const
{
  return joint_names_.size() == original_joint_interfaces_.size();
}

void ControlXacroConfig::setControlInterfaces(const ControlInterfaces& ci)
{
  default_ci_ = ci;
  for (const std::string& joint_name : joint_names_)
  {
    if (original_joint_interfaces_.count(joint_name))
    {
      continue;
    }

    new_joint_interfaces_[joint_name] = default_ci_;

    // Setup the initial state
    std::vector<double> joint_value;
    // TODO: There could be an option to load a group state from the RobotPoses
    //       For now, just assume one DOF, and it is zero
    joint_value.push_back(0.0);
    initial_group_state_.joint_values_[joint_name] = joint_value;
  }

  changed_ = true;
}

void uniqueMerge(std::vector<std::string>& main, const std::vector<std::string>& addition)
{
  for (const std::string& s : addition)
  {
    if (std::find(main.begin(), main.end(), s) == main.end())
    {
      main.push_back(s);
    }
  }
}

bool getControlInterfaceHelper(const std::unordered_map<std::string, ControlInterfaces>& interfaces,
                               const std::string& joint_name, ControlInterfaces& ci)
{
  const auto& it = interfaces.find(joint_name);
  if (it == interfaces.end())
  {
    return false;
  }
  const ControlInterfaces& found_interfaces = it->second;
  uniqueMerge(ci.command_interfaces, found_interfaces.command_interfaces);
  uniqueMerge(ci.state_interfaces, found_interfaces.state_interfaces);
  return true;
}

void ControlXacroConfig::getControlInterfaces(const std::string& joint_name, ControlInterfaces& ci) const
{
  if (getControlInterfaceHelper(original_joint_interfaces_, joint_name, ci))
  {
    return;
  }
  getControlInterfaceHelper(new_joint_interfaces_, joint_name, ci);
}

const ControlInterfaces ControlXacroConfig::getControlInterfaces(const std::vector<std::string>& joint_names) const
{
  ControlInterfaces ci;
  for (const std::string& joint_name : joint_names)
  {
    getControlInterfaces(joint_name, ci);
  }
  return ci;
}

std::string ControlXacroConfig::getJointsXML() const
{
  std::string joints = "";
  const std::string tab = "            ";
  // Loop through all joints to preserve joint ordering
  for (const std::string& joint_name : joint_names_)
  {
    auto pair = new_joint_interfaces_.find(joint_name);
    if (pair == new_joint_interfaces_.end())
    {
      continue;
    }

    const ControlInterfaces& ci = pair->second;

    joints += tab;
    joints += "<joint name=\"" + joint_name + "\">\n";
    for (const std::string& command_interface : ci.command_interfaces)
    {
      joints += tab;
      joints += "    <command_interface name=\"";
      joints += command_interface;
      joints += "\"/>\n";
    }
    for (const std::string& state_interface : ci.state_interfaces)
    {
      joints += tab;
      joints += "    <state_interface name=\"";
      joints += state_interface;
      if (state_interface == "position")
      {
        joints += "\">\n";
        joints += tab;
        joints += "      <param name=\"initial_value\">${initial_positions['";
        joints += joint_name;
        joints += "']}</param>\n";
        joints += tab;
        joints += "    </state_interface>\n";
      }
      else
      {
        joints += "\"/>\n";
      }
    }
    joints += tab;
    joints += "</joint>\n";
  }
  return joints;
}
bool ControlXacroConfig::GeneratedInitialPositions::writeYaml(YAML::Emitter& emitter)
{
  emitter << YAML::Comment("Default initial positions for " + parent_.urdf_config_->getRobotName() +
                           "'s ros2_control fake system");
  emitter << YAML::Newline;
  emitter << YAML::BeginMap;
  {
    emitter << YAML::Key << "initial_positions";
    emitter << YAML::Value;
    emitter << YAML::BeginMap;
    for (const auto& pair : parent_.initial_group_state_.joint_values_)
    {
      emitter << YAML::Key << pair.first;

      const std::vector<double>& jv = pair.second;

      emitter << YAML::Value;
      if (jv.size() == 1)
      {
        emitter << jv[0];
      }
      else
      {
        emitter << jv;
      }
    }
    emitter << YAML::EndMap;
  }
  emitter << YAML::EndMap;

  return true;
}

void ControlXacroConfig::collectVariables(std::vector<TemplateVariable>& variables)
{
  variables.push_back(TemplateVariable("ROS2_CONTROL_JOINTS", getJointsXML()));
}

}  // namespace controllers
}  // namespace moveit_setup

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(moveit_setup::controllers::ControlXacroConfig, moveit_setup::SetupConfig)
