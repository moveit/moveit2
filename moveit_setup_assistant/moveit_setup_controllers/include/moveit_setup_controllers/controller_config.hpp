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
#pragma once

#include <moveit_setup_framework/config.hpp>
#include <moveit_setup_framework/templates.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit_setup_framework/utilities.hpp>

namespace moveit_setup_controllers
{
/**
 * Single controller "instance" configuration
 */
struct ControllerInfo
{
  std::string name_;                 // controller name
  std::string type_;                 // controller type
  std::vector<std::string> joints_;  // joints controlled by this controller
};

static const std::string CONTROLLERS_YAML = "config/ros_controllers.yaml";

/**
 * @brief All the controller configurations
 */
class ControllersConfig : public moveit_setup_framework::SetupConfig
{
public:
  void loadPrevious(const std::string& package_path, const YAML::Node& node) override;

  bool isConfigured() const override
  {
    return !ros_controllers_config_.empty();
  }

  /**
   * \brief Gets ros_controllers_config_ vector
   * \return pointer to ros_controllers_config_
   */
  std::vector<ControllerInfo>& getControllers()
  {
    return ros_controllers_config_;
  }

  class GeneratedControllersConfig : public moveit_setup_framework::YamlGeneratedFile
  {
  public:
    GeneratedControllersConfig(const std::string& package_path, const std::time_t& last_gen_time,
                               ControllersConfig& parent)
      : YamlGeneratedFile(package_path, last_gen_time), parent_(parent)
    {
    }

    bool hasChanges() const override
    {
      // TODO: Also return true when the SRDF groups have changed
      return parent_.changed_;
    }

    std::string getRelativePath() const override
    {
      return CONTROLLERS_YAML;
    }

    std::string getDescription() const override
    {
      return "Creates configurations for ros_controllers.";
    }

    bool writeYaml(YAML::Emitter& emitter) override;

  protected:
    ControllersConfig& parent_;
  };

  class GeneratedContollerLaunch : public moveit_setup_framework::TemplatedGeneratedFile
  {
  public:
    GeneratedContollerLaunch(const std::string& package_path, const std::time_t& last_gen_time,
                             ControllersConfig& parent)
      : TemplatedGeneratedFile(package_path, last_gen_time), parent_(parent)
    {
    }

    bool hasChanges() const override
    {
      // TODO: Also return true when the SRDF groups have changed
      return parent_.changed_;
    }

    std::string getRelativePath() const override
    {
      return "launch/ros_controllers.launch";
    }

    std::string getTemplatePath() const override
    {
      std::string pkg_path = ament_index_cpp::get_package_share_directory("moveit_setup_controllers");
      std::string templates_folder = moveit_setup_framework::appendPaths(pkg_path, "templates");
      return moveit_setup_framework::appendPaths(templates_folder, getRelativePath());
    }

    std::string getDescription() const override
    {
      return "ros_controllers launch file";
    }

  protected:
    ControllersConfig& parent_;
  };

  void collectFiles(const std::string& package_path, const std::time_t& last_gen_time,
                    std::vector<moveit_setup_framework::GeneratedFilePtr>& files) override
  {
    files.push_back(std::make_shared<GeneratedControllersConfig>(package_path, last_gen_time, *this));
    files.push_back(std::make_shared<GeneratedContollerLaunch>(package_path, last_gen_time, *this));
  }

  void collectVariables(std::vector<moveit_setup_framework::TemplateVariable>& variables) override;

  /**
   * \brief Adds a controller to ros_controllers_config_ vector
   * \param name Name of the controller
   * \param type type of the controller
   * \param joint_names vector of the joint names
   * \return true if inserted correctly
   */
  bool addController(const std::string& name, const std::string& type, const std::vector<std::string>& joint_names);

  /**
   * \brief Adds a controller to ros_controllers_config_ vector
   * \param new_controller a new Controller to add
   * \return true if inserted correctly
   */
  bool addController(const ControllerInfo& new_controller);

  /**
   * Find the associated controller by name
   *
   * @param controller_name - name of controller to find in datastructure
   * @return pointer to data in datastructure
   */
  ControllerInfo* findControllerByName(const std::string& controller_name);

  /**
   * Delete controller by name
   *
   * @param controller_name - name of controller to delete
   * @return true if deleted, false if not found
   */
  bool deleteController(const std::string& controller_name);

protected:
  /**
   * Helper function for parsing ros_controllers.yaml file
   * @param YAML::Node - individual controller to be parsed
   * @return true if the file was read correctly
   */
  bool parseController(const YAML::Node& controller);

  /// Controllers config data
  std::vector<ControllerInfo> ros_controllers_config_;
  bool changed_;
};
}  // namespace moveit_setup_controllers
