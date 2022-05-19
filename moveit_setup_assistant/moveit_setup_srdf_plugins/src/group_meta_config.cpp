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

#include <moveit_setup_srdf_plugins/group_meta_config.hpp>

namespace moveit_setup
{
namespace srdf_setup
{
bool GroupMetaConfig::isConfigured() const
{
  return !group_meta_data_.empty();
}

// ******************************************************************************************
// Input kinematics.yaml file
// ******************************************************************************************
void GroupMetaConfig::loadPrevious(const std::filesystem::path& package_path, const YAML::Node& /* node */)
{
  std::filesystem::path file_path = package_path / KINEMATICS_FILE;
  if (!inputKinematicsYAML(file_path))
  {
    throw std::runtime_error("Failed to parse kinematics yaml file. This file is not critical but any previous "
                             "kinematic solver settings have been lost. To re-populate this file edit each "
                             "existing planning group and choose a solver, then save each change.");
  }
}

void GroupMetaConfig::deleteGroup(const std::string& group_name)
{
  group_meta_data_.erase(group_name);
  changed_ = true;
}

void GroupMetaConfig::renameGroup(const std::string& old_group_name, const std::string& new_group_name)
{
  group_meta_data_[new_group_name] = group_meta_data_[old_group_name];
  group_meta_data_.erase(old_group_name);
  changed_ = true;
}

const GroupMetaData& GroupMetaConfig::getMetaData(const std::string& group_name) const
{
  const auto& match = group_meta_data_.find(group_name);
  if (match != group_meta_data_.end())
  {
    return match->second;
  }
  else
  {
    return default_values_;
  }
}

void GroupMetaConfig::setMetaData(const std::string& group_name, const GroupMetaData& meta_data)
{
  group_meta_data_[group_name] = meta_data;
  changed_ = true;
}

bool GroupMetaConfig::inputKinematicsYAML(const std::filesystem::path& file_path)
{
  // Load file
  std::ifstream input_stream(file_path);
  if (!input_stream.good())
  {
    return false;
  }

  // Begin parsing
  try
  {
    YAML::Node doc = YAML::Load(input_stream);

    // Loop through all groups
    for (YAML::const_iterator group_it = doc.begin(); group_it != doc.end(); ++group_it)
    {
      const std::string& group_name = group_it->first.as<std::string>();
      const YAML::Node& group = group_it->second;

      // Create new meta data
      GroupMetaData meta_data;

      getYamlProperty(group, "kinematics_solver", meta_data.kinematics_solver_);
      getYamlProperty(group, "kinematics_solver_search_resolution", meta_data.kinematics_solver_search_resolution_,
                      DEFAULT_KIN_SOLVER_SEARCH_RESOLUTION);
      getYamlProperty(group, "kinematics_solver_timeout", meta_data.kinematics_solver_timeout_,
                      DEFAULT_KIN_SOLVER_TIMEOUT);

      // Assign meta data to vector
      group_meta_data_[group_name] = meta_data;
    }

    return true;
  }
  catch (YAML::ParserException& e)  // Catch errors
  {
    return false;
  }
}

// ******************************************************************************************
// Output kinematic config files
// ******************************************************************************************
bool GroupMetaConfig::GeneratedGroupMetaConfig::writeYaml(YAML::Emitter& emitter)
{
  emitter << YAML::BeginMap;

  // Output every group and the kinematic solver it can use ----------------------------------
  for (const auto& meta_pair : parent_.group_meta_data_)
  {
    const std::string& group_name = meta_pair.first;
    const GroupMetaData& meta_data = meta_pair.second;

    // Only save kinematic data if the solver is not "None"
    if (meta_data.kinematics_solver_.empty() || meta_data.kinematics_solver_ == "None")
      continue;

    emitter << YAML::Key << group_name;
    emitter << YAML::Value << YAML::BeginMap;

    // Kinematic Solver
    emitter << YAML::Key << "kinematics_solver";
    emitter << YAML::Value << meta_data.kinematics_solver_;

    // Search Resolution
    emitter << YAML::Key << "kinematics_solver_search_resolution";
    emitter << YAML::Value << meta_data.kinematics_solver_search_resolution_;

    // Solver Timeout
    emitter << YAML::Key << "kinematics_solver_timeout";
    emitter << YAML::Value << meta_data.kinematics_solver_timeout_;

    emitter << YAML::EndMap;
  }

  emitter << YAML::EndMap;
  return true;
}

void GroupMetaConfig::collectVariables(std::vector<TemplateVariable>& variables)
{
  // TODO: Put any additional parameters files into the ROS 2 launch files where they can be read

  // Add parameter files for the kinematics solvers that should be loaded
  // in addition to kinematics.yaml by planning_context.launch
  std::string kinematics_parameters_files_block;
  for (const auto& groups : group_meta_data_)
  {
    if (groups.second.kinematics_parameters_file_.empty())
      continue;

    // add a linebreak if we have more than one entry
    if (!kinematics_parameters_files_block.empty())
      kinematics_parameters_files_block += "\n";

    std::string line = "    <rosparam command=\"load\" ns=\"" + groups.first + "\" file=\"" +
                       groups.second.kinematics_parameters_file_ + "\"/>";
    kinematics_parameters_files_block += line;
  }
  variables.push_back(TemplateVariable("KINEMATICS_PARAMETERS_FILE_NAMES_BLOCK", kinematics_parameters_files_block));
}

}  // namespace srdf_setup
}  // namespace moveit_setup

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(moveit_setup::srdf_setup::GroupMetaConfig, moveit_setup::SetupConfig)
