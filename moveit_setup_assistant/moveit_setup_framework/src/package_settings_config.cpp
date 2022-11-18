/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik Robotics, Inc.
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
#include <moveit_setup_framework/data/package_settings_config.hpp>
#include <moveit_setup_framework/utilities.hpp>
#include <regex>

namespace moveit_setup
{
const std::unordered_map<std::string, std::string>
    BACKWARDS_KEY_LOOKUP({ { "urdf", "URDF" }, { "srdf", "SRDF" }, { "package_settings", "CONFIG" } });

const std::regex MAIL_REGEX("\\b[A-Z0-9._%+-]+@[A-Z0-9.-]+\\.[A-Z]{2,63}\\b", std::regex::icase);

void PackageSettingsConfig::loadPrevious(const std::filesystem::path& /*config_package_path*/, const YAML::Node& node)
{
  getYamlProperty(node, "author_name", author_name_);
  getYamlProperty(node, "author_email", author_email_);
  unsigned long int timestamp_i;
  getYamlProperty(node, "generated_timestamp", timestamp_i);
  config_pkg_generated_timestamp_ = fromEpoch(timestamp_i);
}

YAML::Node PackageSettingsConfig::saveToYaml() const
{
  YAML::Node node;
  node["author_name"] = author_name_;
  node["author_email"] = author_email_;
  node["generated_timestamp"] = toEpoch(config_pkg_generated_timestamp_);
  return node;
}

void PackageSettingsConfig::setPackagePath(const std::filesystem::path& package_path)
{
  config_pkg_path_ = package_path;
}

void PackageSettingsConfig::setPackageName(const std::string& package_name)
{
  new_package_name_ = package_name;
}

void PackageSettingsConfig::loadExisting(const std::string& package_path_or_name)
{
  if (package_path_or_name.empty())
  {
    throw std::runtime_error("Please specify a configuration package path to load.");
  }
  // Check if it is a path that exists
  if (std::filesystem::is_directory(package_path_or_name))
  {
    // they inputted a full path
    setPackagePath(package_path_or_name);
  }
  else
  {
    // Determine the path from a name
    /* TODO(dlu): Ideally, the package path is in source so that when we write back to it,
     *            the changes will be reflected and then we can check them into git.
     *            However, there's no easy way to determine the source folder from C++.
     *            You could run colcon list -p --packages-select $PACKAGE_NAME but the
     *            results are dependent on what folder you are in and opening an external
     *            process is messy. For now, we just use the share path and rely on the user
     *            to write back to the proper directory in the ConfigurationFiles step
     */
    auto share_dir = getSharePath(package_path_or_name);

    // check that the folder exists
    if (!std::filesystem::is_directory(share_dir))
    {
      throw std::runtime_error("The specified path is not a directory or is not accessible.");
    }

    setPackagePath(share_dir);
  }

  // Load the package name from the package.xml
  std::filesystem::path relative_path;  // we don't use this output value
  extractPackageNameFromPath(config_pkg_path_, new_package_name_, relative_path);

  // Load Config Yaml
  std::filesystem::path config_path = config_pkg_path_ / SETUP_ASSISTANT_FILE;
  if (!std::filesystem::is_regular_file(config_path))
  {
    throw std::runtime_error("The chosen package location exists but was not created using MoveIt Setup Assistant. "
                             "If this is a mistake, provide the missing file: " +
                             config_path.string());
  }

  std::ifstream input_stream(config_path);
  if (!input_stream.good())
  {
    throw std::runtime_error("Unable to open file for reading " + config_path.string());
  }

  // Begin parsing
  try
  {
    const YAML::Node& doc = YAML::Load(input_stream);
    // Get title node
    const YAML::Node& title_node = doc["moveit_setup_assistant_config"];

    for (const std::string& name : config_data_->getRegisteredNames())
    {
      std::string yaml_key = name;
      /* Generally speaking, we use each config's name as the key to read the yaml from
       *
       * However, for backwards compatibility, we also allow for the three legacy keys, which we translate here.
       * If the name is found in BACKWARDS_KEY_LOOKUP and the legacy key is present in the yaml dictionary,
       * we load using the legacy key instead.
       */
      auto backwards_match = BACKWARDS_KEY_LOOKUP.find(name);
      if (backwards_match != BACKWARDS_KEY_LOOKUP.end() && title_node[backwards_match->second].IsDefined())
      {
        yaml_key = backwards_match->second;
      }

      // We load the previous regardless of whether the title_node[yaml_key] is actually defined
      config_data_->get(name)->loadPrevious(config_pkg_path_, title_node[yaml_key]);
    }
  }
  catch (YAML::ParserException& e)  // Catch errors, translate to runtime_error
  {
    throw std::runtime_error("Error parsing " + config_path.string() + ": " + e.what());
  }
}

bool PackageSettingsConfig::GeneratedSettings::writeYaml(YAML::Emitter& emitter)
{
  emitter << YAML::BeginMap;
  // Output every available planner ---------------------------------------------------
  emitter << YAML::Key << "moveit_setup_assistant_config";
  emitter << YAML::Value << YAML::BeginMap;

  for (const auto& config : parent_.config_data_->getConfigured())
  {
    YAML::Node node = config->saveToYaml();
    if (!node.size())
    {
      continue;
    }
    emitter << YAML::Key << config->getName();
    emitter << YAML::Value << node;
  }
  emitter << YAML::EndMap;
  return true;
}

void PackageSettingsConfig::loadDependencies()
{
  package_dependencies_.clear();
  for (const auto& config : config_data_->getConfigured())
  {
    config->collectDependencies(package_dependencies_);
  }
}

void PackageSettingsConfig::collectVariables(std::vector<TemplateVariable>& variables)
{
  variables.push_back(TemplateVariable("GENERATED_PACKAGE_NAME", new_package_name_));

  // TODO: Add new variables for other fields existing in the package.xml
  //       i.e. read the version so that the version is not overwritten
  variables.push_back(TemplateVariable("AUTHOR_NAME", author_name_));
  variables.push_back(TemplateVariable("AUTHOR_EMAIL", author_email_));

  std::stringstream deps;
  for (const auto& dependency : package_dependencies_)
  {
    deps << "  <exec_depend>" << dependency << "</exec_depend>\n";
  }
  variables.push_back(TemplateVariable("OTHER_DEPENDENCIES", deps.str()));
}

bool PackageSettingsConfig::hasValidName() const
{
  // Make sure there is something that isn't just whitespace
  return author_name_.find_first_not_of(' ') != std::string::npos;
}

bool PackageSettingsConfig::hasValidEmail() const
{
  return std::regex_match(author_email_, MAIL_REGEX);
}

void PackageSettingsConfig::setGenerationTime()
{
  config_pkg_generated_timestamp_ = GeneratedTime::clock::now();
}

}  // namespace moveit_setup

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(moveit_setup::PackageSettingsConfig, moveit_setup::SetupConfig)
