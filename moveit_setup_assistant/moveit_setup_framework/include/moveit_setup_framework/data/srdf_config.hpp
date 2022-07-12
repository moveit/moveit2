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
#include <moveit_setup_framework/templates.hpp>
#include <moveit_setup_framework/data/urdf_config.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>  // for getting kinematic model
#include <srdfdom/srdf_writer.h>                   // for writing srdf data

namespace moveit_setup
{
// bits of information that can be changed in the SRDF
enum InformationFields
{
  NONE = 0,
  COLLISIONS = 1 << 1,
  VIRTUAL_JOINTS = 1 << 2,
  GROUPS = 1 << 3,
  GROUP_CONTENTS = 1 << 4,
  POSES = 1 << 5,
  END_EFFECTORS = 1 << 6,
  PASSIVE_JOINTS = 1 << 7,
  OTHER = 1 << 8,
};

static const std::string JOINT_LIMITS_FILE = "config/joint_limits.yaml";
static const std::string CARTESIAN_LIMITS_FILE = "config/pilz_cartesian_limits.yaml";

class SRDFConfig : public SetupConfig
{
public:
  void onInit() override;

  bool isConfigured() const override
  {
    return robot_model_ != nullptr;
  }

  void loadPrevious(const std::filesystem::path& package_path, const YAML::Node& node) override;
  YAML::Node saveToYaml() const override;

  /// Load SRDF File
  void loadSRDFFile(const std::filesystem::path& package_path, const std::filesystem::path& relative_path);
  void loadSRDFFile(const std::filesystem::path& srdf_file_path,
                    const std::vector<std::string>& xacro_args = std::vector<std::string>());

  moveit::core::RobotModelPtr getRobotModel() const
  {
    return robot_model_;
  }

  /// Provide a shared planning scene
  planning_scene::PlanningScenePtr getPlanningScene();

  /// Update the robot model with the new SRDF, AND mark the changes that have been made to the model
  /// changed_information should be composed of InformationFields
  void updateRobotModel(long changed_information = 0L);

  std::vector<std::string> getLinkNames() const;

  void clearCollisionData()
  {
    srdf_.no_default_collision_links_.clear();
    srdf_.enabled_collision_pairs_.clear();
    srdf_.disabled_collision_pairs_.clear();
  }

  std::vector<srdf::Model::CollisionPair>& getDisabledCollisions()
  {
    return srdf_.disabled_collision_pairs_;
  }

  std::vector<srdf::Model::EndEffector>& getEndEffectors()
  {
    return srdf_.end_effectors_;
  }

  std::vector<srdf::Model::Group>& getGroups()
  {
    return srdf_.groups_;
  }

  std::vector<std::string> getGroupNames() const
  {
    std::vector<std::string> group_names;
    for (const srdf::Model::Group& group : srdf_.groups_)
    {
      group_names.push_back(group.name_);
    }
    return group_names;
  }

  std::vector<srdf::Model::GroupState>& getGroupStates()
  {
    return srdf_.group_states_;
  }

  std::vector<srdf::Model::VirtualJoint>& getVirtualJoints()
  {
    return srdf_.virtual_joints_;
  }

  std::vector<srdf::Model::PassiveJoint>& getPassiveJoints()
  {
    return srdf_.passive_joints_;
  }

  /**
   * @brief Return the name of the child link of a joint
   * @return empty string if joint is not found
   */
  std::string getChildOfJoint(const std::string& joint_name) const;

  void removePoseByName(const std::string& pose_name, const std::string& group_name);

  std::vector<std::string> getJointNames(const std::string& group_name, bool include_multi_dof = true,
                                         bool include_passive = true);

  class GeneratedSRDF : public GeneratedFile
  {
  public:
    GeneratedSRDF(const std::filesystem::path& package_path, const GeneratedTime& last_gen_time, SRDFConfig& parent)
      : GeneratedFile(package_path, last_gen_time), parent_(parent)
    {
    }

    std::filesystem::path getRelativePath() const override
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
      return parent_.changes_ > 0;
    }

    bool write() override
    {
      std::filesystem::path path = getPath();
      createParentFolders(path);
      return parent_.write(path);
    }

  protected:
    SRDFConfig& parent_;
  };

  class GeneratedJointLimits : public YamlGeneratedFile
  {
  public:
    GeneratedJointLimits(const std::filesystem::path& package_path, const GeneratedTime& last_gen_time,
                         SRDFConfig& parent)
      : YamlGeneratedFile(package_path, last_gen_time), parent_(parent)
    {
    }

    std::filesystem::path getRelativePath() const override
    {
      return JOINT_LIMITS_FILE;
    }

    std::string getDescription() const override
    {
      return "Contains additional information about joints that appear in your planning groups that is not "
             "contained in the URDF, as well as allowing you to set maximum and minimum limits for velocity "
             "and acceleration than those contained in your URDF. This information is used by our trajectory "
             "filtering system to assign reasonable velocities and timing for the trajectory before it is "
             "passed to the robot's controllers.";
    }

    bool hasChanges() const override
    {
      return false;  // Can't be changed just yet
    }

    bool writeYaml(YAML::Emitter& emitter) override;

  protected:
    SRDFConfig& parent_;
  };

  class GeneratedCartesianLimits : public TemplatedGeneratedFile
  {
  public:
    using TemplatedGeneratedFile::TemplatedGeneratedFile;

    std::filesystem::path getRelativePath() const override
    {
      return CARTESIAN_LIMITS_FILE;
    }

    std::filesystem::path getTemplatePath() const override
    {
      return getSharePath("moveit_setup_framework") / "templates" / CARTESIAN_LIMITS_FILE;
    }

    std::string getDescription() const override
    {
      return "Cartesian velocity for planning in the workspace."
             "The velocity is used by pilz industrial motion planner as maximum velocity for cartesian "
             "planning requests scaled by the velocity scaling factor of an individual planning request.";
    }

    bool hasChanges() const override
    {
      return false;
    }
  };

  void collectFiles(const std::filesystem::path& package_path, const GeneratedTime& last_gen_time,
                    std::vector<GeneratedFilePtr>& files) override
  {
    files.push_back(std::make_shared<GeneratedSRDF>(package_path, last_gen_time, *this));
    files.push_back(std::make_shared<GeneratedJointLimits>(package_path, last_gen_time, *this));
    files.push_back(std::make_shared<GeneratedCartesianLimits>(package_path, last_gen_time));
  }

  void collectVariables(std::vector<TemplateVariable>& variables) override;

  bool write(const std::filesystem::path& path)
  {
    return srdf_.writeSRDF(path);
  }

  std::filesystem::path getPath() const
  {
    return srdf_path_;
  }

  unsigned long getChangeMask() const
  {
    return changes_;
  }

protected:
  void getRelativePath();
  void loadURDFModel();

  // ******************************************************************************************
  // SRDF Data
  // ******************************************************************************************
  /// Full file-system path to srdf
  std::filesystem::path srdf_path_;

  /// Path relative to loaded configuration package
  std::filesystem::path srdf_pkg_relative_path_;

  /// SRDF Data and Writer
  srdf::SRDFWriter srdf_;

  std::shared_ptr<urdf::Model> urdf_model_;

  moveit::core::RobotModelPtr robot_model_;

  /// Shared planning scene
  planning_scene::PlanningScenePtr planning_scene_;

  // bitfield of changes (composed of InformationFields)
  unsigned long changes_;
};
}  // namespace moveit_setup
