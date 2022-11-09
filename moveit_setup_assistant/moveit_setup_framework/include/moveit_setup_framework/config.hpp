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

#pragma once

#include <moveit_setup_framework/generated_file.hpp>
#include <moveit_setup_framework/templates.hpp>
#include <moveit/macros/class_forward.h>
#include <rclcpp/node.hpp>
#include <yaml-cpp/yaml.h>

#include <set>
#include <utility>
#include <utility>

namespace moveit_setup
{
class DataWarehouse;
// NB: We don't use DataWarehousePtr here because it gets complicated with this abstract usage

MOVEIT_CLASS_FORWARD(SetupConfig);  // Defines SetupConfigPtr, ConstPtr, WeakPtr... etc

/**
 * @brief where all the data for each part of the configuration is stored.
 */
class SetupConfig
{
public:
  /**
   * @brief Called after construction to initialize the step
   * @param config_data Pointer to all the other configs
   * @param parent_node Shared pointer to the parent node
   * @param name
   */
  void initialize(const std::shared_ptr<DataWarehouse>& config_data, const rclcpp::Node::SharedPtr& parent_node,
                  const std::string& name)
  {
    std::move(config_);
    data_ = std::move(config_data);
    parent_node_ = parent_node;
    name_ = name;
    logger_ = std::make_shared<rclcpp::Logger>(parent_node->get_logger().get_child(name));
    onInit();
  }

  /**
   * @brief Overridable initialization method
   */
  virtual void onInit()
  {
  }

  /**
   * @brief The name for this part of the configuration
   */
  const std::string& getName()
  {
    return name_;
  }

  /**
   * @brief Return true if this part of the configuration is completely set up.
   */
  virtual bool isConfigured() const
  {
    return false;
  }

  /**
   * @brief Loads the configuration from an existing MoveIt configuration.
   *
   * The data can be loaded directly from files in the configuration
   * via the package path.
   *
   * Certain other pieces of "meta" information may be stored in the .setup_assistant
   * yaml file in the root of the configuration. If there is a node in that file that
   * matches this config's name, it is passed in as an argument.
   *
   * @param package_path The path to the root folder of the configuration.
   */
  virtual void loadPrevious(const std::filesystem::path& /*config_package_path*/, const YAML::Node& /*node*/)
  {
  }

  /**
   * @brief Optionally save "meta" information for saving in the .setup_assistant yaml file
   */
  virtual YAML::Node saveToYaml() const
  {
    return YAML::Node();
  }

  /**
   * @brief Collect the files generated by this configuration and add them to the vector
   *
   * @param[in] package_path the path to the root of the config package
   * @param[in] last_gen_time The time (if any) when the config package was last generated
   * @parma[out] files Where to put the new generated files
   */
  virtual void collectFiles(const std::filesystem::path& /*package_path*/, const GeneratedTime& /*last_gen_time*/,
                            std::vector<GeneratedFilePtr>& /*files*/)
  {
  }

  /**
   * @brief Collect the package dependencies generated by this configuration
   *
   * @param[out] packages Names of ROS packages
   */
  virtual void collectDependencies(std::set<std::string>& /*packages*/) const
  {
  }

  /**
   * @brief Collect key/value pairs for use in templates
   *
   * @param[out] variables Where to put the new Variables
   */
  virtual void collectVariables(std::vector<TemplateVariable>& /*variables*/)
  {
  }

protected:
  std::shared_ptr<DataWarehouse> config_data_;
  rclcpp::Node::SharedPtr parent_node_;
  std::string name_;
  std::shared_ptr<rclcpp::Logger> logger_;
};
}  // namespace moveit_setup
