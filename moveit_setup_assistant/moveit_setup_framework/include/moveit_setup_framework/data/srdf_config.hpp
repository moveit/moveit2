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

/* Author: David V. Lu!! */
#pragma once

#include <moveit_setup_framework/config.hpp>
#include <moveit_setup_framework/data/urdf_config.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>  // for getting kinematic model
#include <srdfdom/srdf_writer.h>                   // for writing srdf data

namespace moveit_setup_framework
{
class SRDFConfig : public SetupConfig
{
public:
  void onInit() override;

  bool isConfigured() const override
  {
    return robot_model_ != nullptr;
  }

  void loadPrevious(const std::string& package_path, const YAML::Node& node) override;
  YAML::Node saveToYaml() const override;

  /// Load SRDF File
  void loadSRDFFile(const std::string& package_path, const std::string& relative_path);
  void loadSRDFFile(const std::string& srdf_file_path);

  moveit::core::RobotModelPtr getRobotModel() const
  {
    return robot_model_;
  }

  /// Provide a shared planning scene
  planning_scene::PlanningScenePtr getPlanningScene();

  void updateRobotModel(bool mark_as_changed = false);

  std::vector<std::string> getLinkNames() const;

  std::vector<srdf::Model::DisabledCollision>& getDisabledCollisions()
  {
    return srdf_.disabled_collisions_;
  }

  std::vector<srdf::Model::EndEffector>& getEndEffectors()
  {
    return srdf_.end_effectors_;
  }

  std::vector<srdf::Model::Group>& getGroups()
  {
    return srdf_.groups_;
  }

  std::vector<srdf::Model::VirtualJoint>& getVirtualJoints()
  {
    return srdf_.virtual_joints_;
  }

  class GeneratedSRDF : public GeneratedFile
  {
  public:
    GeneratedSRDF(const std::string& package_path, const std::time_t& last_gen_time, SRDFConfig& parent)
      : GeneratedFile(package_path, last_gen_time), parent_(parent)
    {
    }

    std::string getRelativePath() const override
    {
      return parent_.srdf_pkg_relative_path_;
    }

    std::string getDescription() const override
    {
      return "SRDF (<a href='http://www.ros.org/wiki/srdf'>Semantic Robot Description Format</a>) is a "
             "representation of semantic information about robots. This format is intended to represent "
             "information about the robot that is not in the URDF file, but it is useful for a variety of "
             "applications. The intention is to include information that has a semantic aspect to it.";
    }

    bool hasChanges() const override
    {
      return parent_.has_changes_;
    }

    bool write() override
    {
      std::string path = getPath();
      createParentFolders(path);
      return parent_.write(path);
    }

  protected:
    SRDFConfig& parent_;
  };

  void collectFiles(const std::string& package_path, const std::time_t& last_gen_time,
                    std::vector<GeneratedFilePtr>& files) override
  {
    files.push_back(std::make_shared<GeneratedSRDF>(package_path, last_gen_time, *this));
  }

  void collectVariables(std::vector<TemplateVariable>& variables) override;

  bool write(const std::string& path)
  {
    return srdf_.writeSRDF(path);
  }

  std::string getPath() const
  {
    return srdf_path_;
  }

protected:
  void getRelativePath();
  void loadURDFModel();

  // ******************************************************************************************
  // SRDF Data
  // ******************************************************************************************
  /// Full file-system path to srdf
  std::string srdf_path_;

  /// Path relative to loaded configuration package
  std::string srdf_pkg_relative_path_;

  /// SRDF Data and Writer
  srdf::SRDFWriter srdf_;

  std::shared_ptr<urdf::Model> urdf_model_;

  moveit::core::RobotModelPtr robot_model_;

  /// Shared planning scene
  planning_scene::PlanningScenePtr planning_scene_;

  bool has_changes_;
};
}  // namespace moveit_setup_framework
