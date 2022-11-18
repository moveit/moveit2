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

#include <moveit_setup_framework/data_warehouse.hpp>
#include <moveit_setup_framework/data/urdf_config.hpp>
#include <moveit_setup_framework/data/srdf_config.hpp>
#include <moveit_setup_framework/data/package_settings_config.hpp>

namespace moveit_setup
{
DataWarehouse::DataWarehouse(const rclcpp::Node::SharedPtr& parent_node)
  : parent_node_(parent_node), config_loader_("moveit_setup_framework", "moveit_setup::SetupConfig")
{
  registerType("urdf", "moveit_setup::URDFConfig");
  registerType("srdf", "moveit_setup::SRDFConfig");
  registerType("package_settings", "moveit_setup::PackageSettingsConfig");
}

void DataWarehouse::preloadWithURDFPath(const std::filesystem::path& urdf_path)
{
  get<URDFConfig>("urdf")->loadFromPath(urdf_path);
}

void DataWarehouse::preloadWithFullConfig(const std::string& package_path_or_name)
{
  get<PackageSettingsConfig>("package_settings")->loadExisting(package_path_or_name);
}

SetupConfigPtr DataWarehouse::get(const std::string& config_name, std::string config_class)
{
  if (config_class.empty())
  {
    auto it = registered_types_.find(config_name);
    if (it == registered_types_.end())
    {
      throw std::runtime_error(config_name + " does not have a registered type in the data warehouse");
    }
    config_class = it->second;
  }
  auto it = configs_.find(config_name);
  if (it != configs_.end())
  {
    return it->second;
  }
  auto config = config_loader_.createSharedInstance(config_class);
  config->initialize(shared_from_this(), parent_node_, config_name);
  configs_[config_name] = config;
  return config;
}

void DataWarehouse::registerType(const std::string& config_name, const std::string& config_class)
{
  registered_types_[config_name] = config_class;
  registered_names_.push_back(config_name);
}

std::vector<SetupConfigPtr> DataWarehouse::getConfigured()
{
  std::vector<SetupConfigPtr> configs;
  for (const std::string& config_name : registered_names_)
  {
    auto it = configs_.find(config_name);
    if (it == configs_.end())
    {
      continue;
    }
    SetupConfigPtr ptr = it->second;
    if (ptr->isConfigured())
    {
      configs.push_back(ptr);
    }
  }
  return configs;
}

const std::vector<std::string>& DataWarehouse::getRegisteredNames() const
{
  return registered_names_;
}
}  // namespace moveit_setup
