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

#include <moveit_setup_framework/data/urdf_config.hpp>
#include <moveit_setup_framework/utilities.hpp>
#include <moveit/rdf_loader/rdf_loader.h>
#include <boost/algorithm/string/join.hpp>

namespace moveit_setup
{
void URDFConfig::onInit()
{
  parent_node_->declare_parameter("robot_description", rclcpp::ParameterType::PARAMETER_STRING);
}

void URDFConfig::loadPrevious(const std::filesystem::path& /*config_package_path*/, const YAML::Node& node)
{
  if (!getYamlProperty(node, "package", urdf_pkg_name_))
  {
    throw std::runtime_error("cannot find package property in URDF");
  }

  if (!getYamlProperty(node, "relative_path", urdf_pkg_relative_path_))
  {
    throw std::runtime_error("cannot find relative_path property in URDF");
  }

  getYamlProperty(node, "xacro_args", xacro_args_);
  loadFromPackage(urdf_pkg_name_, urdf_pkg_relative_path_, xacro_args_);
}

YAML::Node URDFConfig::saveToYaml() const
{
  YAML::Node node;
  node["package"] = urdf_pkg_name_;
  node["relative_path"] = urdf_pkg_relative_path_.string();
  if (!xacro_args_.empty())
  {
    node["xacro_args"] = xacro_args_;
  }
  return node;
}

void URDFConfig::loadFromPath(const std::filesystem::path& urdf_file_path, const std::string& xacro_args)
{
  urdf_path_ = urdf_file_path;
  xacro_args_ = xacro_args;
  xacro_args_vec_ = { xacro_args_ };
  setPackageName();
  load();
}

void URDFConfig::loadFromPath(const std::filesystem::path& urdf_file_path, const std::vector<std::string>& xacro_args)
{
  urdf_path_ = urdf_file_path;
  xacro_args_vec_ = xacro_args;
  xacro_args_ = boost::algorithm::join(xacro_args_vec_, " ");
  setPackageName();
  load();
}

void URDFConfig::setPackageName()
{
  bool package_found = extractPackageNameFromPath(urdf_path_, urdf_pkg_name_, urdf_pkg_relative_path_);
  if (!package_found)
  {
    urdf_pkg_name_ = "";
    urdf_pkg_relative_path_ = urdf_path_;  // just the absolute path
  }
  else
  {
    // Check that ROS can find the package
    const std::filesystem::path robot_desc_pkg_path = getSharePath(urdf_pkg_name_);

    if (robot_desc_pkg_path.empty())
    {
      RCLCPP_WARN(*logger_,
                  "Package Not Found In ROS Workspace. ROS was unable to find the package name '%s'"
                  " within the ROS workspace. This may cause issues later.",
                  urdf_pkg_name_.c_str());
    }
  }
}

void URDFConfig::loadFromPackage(const std::filesystem::path& package_name, const std::filesystem::path& relative_path,
                                 const std::string& xacro_args)
{
  urdf_pkg_name_ = package_name;
  urdf_pkg_relative_path_ = relative_path;
  xacro_args_ = xacro_args;

  urdf_path_ = getSharePath(urdf_pkg_name_) / relative_path;
  load();
}

void URDFConfig::load()
{
  RCLCPP_DEBUG_STREAM(*logger_, "URDF Package Name: " << urdf_pkg_name_);
  RCLCPP_DEBUG_STREAM(*logger_, "URDF Package Path: " << urdf_pkg_relative_path_);

  if (!rdf_loader::RDFLoader::loadXmlFileToString(urdf_string_, urdf_path_, xacro_args_vec_))
  {
    throw std::runtime_error("URDF/COLLADA file not found: " + urdf_path_.string());
  }

  if (urdf_string_.empty() && rdf_loader::RDFLoader::isXacroFile(urdf_path_))
  {
    throw std::runtime_error("Running xacro failed.\nPlease check console for errors.");
  }

  // Verify that file is in correct format / not an XACRO by loading into robot model
  if (!urdf_model_->initString(urdf_string_))
  {
    throw std::runtime_error("URDF/COLLADA file is not a valid robot model.");
  }
  urdf_from_xacro_ = rdf_loader::RDFLoader::isXacroFile(urdf_path_);

  // Set parameter
  parent_node_->set_parameter(rclcpp::Parameter("robot_description", urdf_string_));

  RCLCPP_INFO_STREAM(*logger_, "Loaded " << urdf_model_->getName() << " robot model.");
}

bool URDFConfig::isXacroFile() const
{
  return rdf_loader::RDFLoader::isXacroFile(urdf_path_);
}

bool URDFConfig::isConfigured() const
{
  return urdf_model_->getRoot() != nullptr;
}

void URDFConfig::collectDependencies(std::set<std::string>& packages) const
{
  packages.insert(urdf_pkg_name_);
}

void URDFConfig::collectVariables(std::vector<TemplateVariable>& variables)
{
  std::string urdf_location;
  if (urdf_pkg_name_.empty())
  {
    urdf_location = urdf_path_;
  }
  else
  {
    urdf_location = "$(find " + urdf_pkg_name_ + ")/" + urdf_pkg_relative_path_.string();
  }

  variables.push_back(TemplateVariable("URDF_LOCATION", urdf_location));

  if (urdf_from_xacro_)
  {
    variables.push_back(
        TemplateVariable("URDF_LOAD_ATTRIBUTE", "command=\"xacro " + xacro_args_ + " '" + urdf_location + "'\""));
  }
  else
  {
    variables.push_back(TemplateVariable("URDF_LOAD_ATTRIBUTE", "textfile=\"" + urdf_location + "\""));
  }
}
}  // namespace moveit_setup

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(moveit_setup::URDFConfig, moveit_setup::SetupConfig)
