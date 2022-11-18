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

#include <moveit_setup_controllers/modified_urdf_config.hpp>

namespace moveit_setup
{
void ModifiedUrdfConfig::onInit()
{
  urdf_config_ = config_data_->get<URDFConfig>("urdf");
}

bool ModifiedUrdfConfig::isConfigured() const
{
  return !getIncludedXacros().empty();
}

void ModifiedUrdfConfig::loadPrevious(const std::filesystem::path& /*config_package_path*/, const YAML::Node& node)
{
  std::vector<std::string> xacro_names_vector;
  if (node.IsDefined())
    getYamlProperty(node, "xacros", xacro_names_vector);
  cached_xacro_names_ = std::set(xacro_names_vector.begin(), xacro_names_vector.end());
}

YAML::Node ModifiedUrdfConfig::saveToYaml() const
{
  YAML::Node node;
  node["xacros"] = getIncludedXacroNames();
  return node;
}

bool ModifiedUrdfConfig::hasChanges() const
{
  // Returns true if any of the included xacros are configured and have individually changed
  // or the list of included xacros have changed.
  unsigned int count = 0;
  for (auto& pair : getIncludedXacroMap())
  {
    if (!pair.second->isConfigured())
    {
      continue;
    }
    if (pair.second->hasChanges() || cached_xacro_names_.count(pair.first) == 0)
    {
      return true;
    }
    count++;
  }

  return count != cached_xacro_names_.size();
}

void ModifiedUrdfConfig::collectFiles(const std::filesystem::path& package_path, const GeneratedTime& last_gen_time,
                                      std::vector<GeneratedFilePtr>& files)
{
  files.push_back(std::make_shared<GeneratedModifiedURDF>(package_path, last_gen_time, *this));

  auto xacro_names = getIncludedXacroNames();
  cached_xacro_names_ = std::set<std::string>(xacro_names.begin(), xacro_names.end());
}

void ModifiedUrdfConfig::collectDependencies(std::set<std::string>& packages) const
{
  packages.insert("xacro");
}

void ModifiedUrdfConfig::collectVariables(std::vector<TemplateVariable>& variables)
{
  std::string args = "", imports = "", commands = "";

  for (const auto& pair : getIncludedXacroMap())
  {
    const IncludedXacroConfig::Ptr& xacro = pair.second;
    if (!xacro->isConfigured())
    {
      continue;
    }
    for (const std::pair<std::string, std::string>& argument : xacro->getArguments())
    {
      args += "    <xacro:arg name=\"";
      args += argument.first;
      args += "\" default=\"";
      args += argument.second;
      args += "\" />\n";
    }

    imports += "    <!-- Import ";
    imports += pair.first;
    imports += " -->\n";

    imports += "    <xacro:include filename=\"";
    imports += xacro->getFilepath();
    imports += "\" />\n\n";

    for (const std::string& command : xacro->getCommands())
    {
      commands += "    ";
      commands += command;
      commands += "\n";
    }
  }

  variables.push_back(TemplateVariable("MODIFIED_XACRO_ARGS", args));
  variables.push_back(TemplateVariable("MODIFIED_XACRO_IMPORTS", imports));
  variables.push_back(TemplateVariable("MODIFIED_XACRO_COMMANDS", commands));
}
}  // namespace moveit_setup

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(moveit_setup::ModifiedUrdfConfig, moveit_setup::SetupConfig)
