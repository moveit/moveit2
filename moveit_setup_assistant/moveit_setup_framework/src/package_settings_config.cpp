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
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <regex>

namespace moveit_setup_framework
{
const std::unordered_map<std::string, std::string>
    BACKWARDS_KEY_LOOKUP({ { "urdf", "URDF" }, { "srdf", "SRDF" }, { "package_settings", "CONFIG" } });

const std::regex MAIL_REGEX("\\b[A-Z0-9._%+-]+@[A-Z0-9.-]+\\.[A-Z]{2,4}\\b", std::regex::icase);

void PackageSettingsConfig::loadPrevious(const std::string& /*config_package_path*/, const YAML::Node& node)
{
  getYamlProperty(node, "author_name", author_name_);
  getYamlProperty(node, "author_email", author_email_);
  getYamlProperty(node, "generated_timestamp", config_pkg_generated_timestamp_);
}

YAML::Node PackageSettingsConfig::saveToYaml() const
{
  YAML::Node node;
  node["author_name"] = author_name_;
  node["author_email"] = author_email_;
  node["generated_timestamp"] = std::time(nullptr);  // TODO: is this cross-platform?
  return node;
}

void PackageSettingsConfig::setPackagePath(const std::string& package_path)
{
  config_pkg_path_ = package_path;

  // Determine new package name
  boost::filesystem::path fs_package_path;
  // Remove end slash if there is one
  if (!package_path.compare(package_path.size() - 1, 1, "/"))
  {
    fs_package_path = boost::filesystem::path(package_path.substr(0, package_path.size() - 1));
  }
  else
  {
    fs_package_path = boost::filesystem::path(package_path);
  }

  // Get the last directory name
  new_package_name_ = fs_package_path.filename().string();

  // check for empty
  if (new_package_name_.empty())
    new_package_name_ = "unknown";
}

void PackageSettingsConfig::loadExisting(const std::string& package_path)
{
  if (package_path.empty())
  {
    throw std::runtime_error("Please specify a configuration package path to load.");
  }
  // check that the folder exists
  if (boost::filesystem::is_directory(package_path))
  {
    // they inputted a full path
    setPackagePath(package_path);
  }
  else
  {
    // does not exist, check if its a package
    std::string share_dir = ament_index_cpp::get_package_share_directory(package_path);

    // check that the folder exists
    if (!boost::filesystem::is_directory(share_dir))
    {
      throw std::runtime_error("The specified path is not a directory or is not accessible.");
    }

    setPackagePath(share_dir);
  }

  // Load Config Yaml
  std::string config_path = appendPaths(config_pkg_path_, ".setup_assistant");
  if (!boost::filesystem::is_regular_file(config_path))
  {
    throw std::runtime_error("The chosen package location exists but was not created using MoveIt Setup Assistant. "
                             "If this is a mistake, provide the missing file: " +
                             config_path);
  }

  std::ifstream input_stream(config_path.c_str());
  if (!input_stream.good())
  {
    throw std::runtime_error("Unable to open file for reading " + config_path);
  }

  // Begin parsing
  try
  {
    const YAML::Node& doc = YAML::Load(input_stream);
    // Get title node
    const YAML::Node& title_node = doc["moveit_setup_assistant_config"];

    for (const std::string& name : config_data_->getRegisteredNames())
    {
      // Replace old keys with new ones
      std::string yaml_key = name;
      auto backwards_match = BACKWARDS_KEY_LOOKUP.find(name);
      if (backwards_match != BACKWARDS_KEY_LOOKUP.end())
      {
        yaml_key = backwards_match->second;
      }

      config_data_->get(name)->loadPrevious(config_pkg_path_, title_node[yaml_key]);
    }
  }
  catch (YAML::ParserException& e)  // Catch errors
  {
    throw std::runtime_error("Error parsing " + config_path + ": " + e.what());
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

void PackageSettingsConfig::collectVariables(std::vector<TemplateVariable>& variables)
{
  variables.push_back(TemplateVariable("GENERATED_PACKAGE_NAME", new_package_name_));
  variables.push_back(TemplateVariable("AUTHOR_NAME", author_name_));
  variables.push_back(TemplateVariable("AUTHOR_EMAIL", author_email_));

  std::stringstream deps;
  for (const auto& depenency : package_dependencies_)
  {
    deps << "  <run_depend>" << depenency << "</run_depend>\n";
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
}  // namespace moveit_setup_framework

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(moveit_setup_framework::PackageSettingsConfig, moveit_setup_framework::SetupConfig)
