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

#include <moveit_setup_framework/config.hpp>
#include <moveit/macros/class_forward.h>
#include <pluginlib/class_loader.hpp>
#include <unordered_map>
#include <memory>

#pragma once

namespace moveit_setup
{
MOVEIT_CLASS_FORWARD(DataWarehouse);  // Defines DataWarehousePtr, ConstPtr, WeakPtr... etc

/**
 * @brief Container for all of the `SetupConfig` object singletons
 */
class DataWarehouse : public std::enable_shared_from_this<DataWarehouse>
{
public:
  DataWarehouse(const rclcpp::Node::SharedPtr& parent_node);

  void preloadWithURDFPath(const std::filesystem::path& urdf_path);
  void preloadWithFullConfig(const std::string& package_path_or_name);

  /**
   * @brief Get the singleton for a given config name and class
   *
   * @param config_name The name of the SetupConfig
   * @param config_class The string used to load the class via pluginlib. If empty, the registered type is used.
   * @throws std::runtime_error If the name is not registered to a type
   * @returns Shared pointer to the generic SetupConfig object
   */
  SetupConfigPtr get(const std::string& config_name, std::string config_class = "");

  /**
   * @brief Get the specific singleton for a given config name and class
   *
   * Unlike the non-templated version of this method, this returns
   * the SetupConfig pointer cast to the specific config type.
   *
   * @param config_name The name of the SetupConfig
   * @param config_class The string used to load the class via pluginlib. If empty, the registered type is used.
   * @throws std::runtime_error If the name is not registered to a type
   * @returns Shared pointer to the specific SetupConfig object
   */
  template <typename T>
  std::shared_ptr<T> get(const std::string& config_name, const std::string& config_class = "")
  {
    return std::static_pointer_cast<T>(get(config_name, config_class));
  }

  /**
   * @brief Get all of the registered configs that match the given config_class
   *
   * @param config_class The string representing the class name
   * @returns Map of shared pointers from config names to singleton config
   */
  template <typename T>
  std::unordered_map<std::string, std::shared_ptr<T>> getAll()
  {
    std::unordered_map<std::string, std::shared_ptr<T>> matches;
    for (const auto& pair : registered_types_)
    {
      SetupConfigPtr uncast_ptr = get(pair.first);
      std::shared_ptr<T> ptr = std::dynamic_pointer_cast<T>(uncast_ptr);
      if (!ptr)
      {
        continue;
      }

      matches[pair.first] = ptr;
    }
    return matches;
  }

  /**
   * @brief Associates a class_name with the given name. Makes calls to get more succinct.
   */
  void registerType(const std::string& config_name, const std::string& config_class);

  /**
   * @brief Returns a list of the SetupConfig for which isConfigured is true
   */
  std::vector<SetupConfigPtr> getConfigured();

  /**
   * @brief Returns a list of config_names that have registered types associated with them
   */
  const std::vector<std::string>& getRegisteredNames() const;

  /// Is this application in debug mode?
  bool debug{ false };

protected:
  rclcpp::Node::SharedPtr parent_node_;
  pluginlib::ClassLoader<SetupConfig> config_loader_;
  std::unordered_map<std::string, SetupConfigPtr> configs_;        // mapping from name to config
  std::unordered_map<std::string, std::string> registered_types_;  // mapping from name to config class type
  std::vector<std::string> registered_names_;                      // string version of keys for the maps, with ordering
};
}  // namespace moveit_setup
