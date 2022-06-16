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
#include <moveit_setup_framework/data/srdf_config.hpp>
#include <moveit_setup_framework/data_warehouse.hpp>
#include <moveit_setup_framework/utilities.hpp>
#include <moveit/rdf_loader/rdf_loader.h>

namespace moveit_setup
{
void SRDFConfig::onInit()
{
  parent_node_->declare_parameter("robot_description_semantic", rclcpp::ParameterType::PARAMETER_STRING);
  changes_ = 0L;
}

void SRDFConfig::loadPrevious(const std::filesystem::path& package_path, const YAML::Node& node)
{
  if (!getYamlProperty(node, "relative_path", srdf_pkg_relative_path_))
  {
    throw std::runtime_error("cannot find relative_path property in SRDF");
  }
  loadSRDFFile(package_path, srdf_pkg_relative_path_);
}

YAML::Node SRDFConfig::saveToYaml() const
{
  YAML::Node node;
  node["relative_path"] = srdf_pkg_relative_path_.string();
  return node;
}

void SRDFConfig::loadURDFModel()
{
  if (urdf_model_ != nullptr)
  {
    return;
  }

  auto urdf_config = config_data_->get<URDFConfig>("urdf");
  urdf_model_ = urdf_config->getModelPtr();
  srdf_.robot_name_ = urdf_model_->getName();
  parent_node_->set_parameter(rclcpp::Parameter("robot_description_semantic", srdf_.getSRDFString()));
}

void SRDFConfig::loadSRDFFile(const std::filesystem::path& package_path, const std::filesystem::path& relative_path)
{
  srdf_pkg_relative_path_ = relative_path;
  loadSRDFFile(std::filesystem::path(package_path) / relative_path);
}
// ******************************************************************************************
// Load SRDF File to Parameter Server
// ******************************************************************************************
void SRDFConfig::loadSRDFFile(const std::filesystem::path& srdf_file_path, const std::vector<std::string>& xacro_args)
{
  srdf_path_ = srdf_file_path;

  loadURDFModel();

  std::string srdf_string;
  if (!rdf_loader::RDFLoader::loadXmlFileToString(srdf_string, srdf_path_, xacro_args))
  {
    throw std::runtime_error("SRDF file not found: " + srdf_path_.string());
  }

  // Verify that file is in correct format / not an XACRO by loading into robot model
  if (!srdf_.initString(*urdf_model_, srdf_string))
  {
    throw std::runtime_error("SRDF file not a valid semantic robot description model.");
  }

  // Set parameter
  parent_node_->set_parameter(rclcpp::Parameter("robot_description_semantic", srdf_string));

  updateRobotModel();

  RCLCPP_INFO_STREAM(*logger_, "Robot semantic model successfully loaded.");
}

void SRDFConfig::getRelativePath()
{
  if (!srdf_pkg_relative_path_.empty())
  {
    return;
  }
  srdf_pkg_relative_path_ = std::filesystem::path("config") / (urdf_model_->getName() + ".srdf");
}

void SRDFConfig::updateRobotModel(long changed_information)
{
  // Initialize with a URDF Model Interface and a SRDF Model
  loadURDFModel();
  if (changed_information > 0)
  {
    srdf_.updateSRDFModel(*urdf_model_);
  }

  robot_model_ = std::make_shared<moveit::core::RobotModel>(urdf_model_, srdf_.srdf_model_);

  if (srdf_pkg_relative_path_.empty())
  {
    srdf_pkg_relative_path_ = std::filesystem::path("config") / (urdf_model_->getName() + ".srdf");
    srdf_.robot_name_ = urdf_model_->getName();
    changes_ |= OTHER;
  }
  changes_ |= changed_information;

  // Reset the planning scene
  planning_scene_.reset();
}

// ******************************************************************************************
// Provide a shared planning scene
// ******************************************************************************************
planning_scene::PlanningScenePtr SRDFConfig::getPlanningScene()
{
  if (!planning_scene_)
  {
    // make sure kinematic model exists
    getRobotModel();

    // Allocate an empty planning scene
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
  }
  return planning_scene_;
}

std::vector<std::string> SRDFConfig::getLinkNames() const
{
  std::vector<std::string> names;
  for (const moveit::core::LinkModel* link_model : robot_model_->getLinkModels())
  {
    names.push_back(link_model->getName());
  }
  return names;
}

std::string SRDFConfig::getChildOfJoint(const std::string& joint_name) const
{
  const moveit::core::JointModel* joint_model = getRobotModel()->getJointModel(joint_name);
  // Check that a joint model was found
  if (!joint_model)
  {
    return "";
  }
  return joint_model->getChildLinkModel()->getName();
}

void SRDFConfig::removePoseByName(const std::string& pose_name, const std::string& group_name)
{
  for (std::vector<srdf::Model::GroupState>::iterator pose_it = srdf_.group_states_.begin();
       pose_it != srdf_.group_states_.end(); ++pose_it)
  {
    if (pose_it->name_ == pose_name && pose_it->group_ == group_name)
    {
      srdf_.group_states_.erase(pose_it);
      updateRobotModel(moveit_setup::POSES);
      return;
    }
  }
}

std::vector<std::string> SRDFConfig::getJointNames(const std::string& group_name, bool include_multi_dof,
                                                   bool include_passive)
{
  std::vector<std::string> names;
  const moveit::core::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group_name);

  // Iterate through the joints
  for (const moveit::core::JointModel* joint : joint_model_group->getActiveJointModels())
  {
    if (!include_multi_dof && joint->getVariableCount() > 1)
    {
      continue;
    }
    else if (!include_passive && joint->isPassive())
    {
      continue;
    }
    names.push_back(joint->getName());
  }
  return names;
}

void SRDFConfig::collectVariables(std::vector<TemplateVariable>& variables)
{
  variables.push_back(TemplateVariable("ROBOT_NAME", srdf_.robot_name_));
  variables.push_back(TemplateVariable("ROBOT_ROOT_LINK", robot_model_->getRootLinkName()));
  variables.push_back(TemplateVariable("PLANNING_FRAME", robot_model_->getModelFrame()));
}

/**
 * \brief Custom std::set comparator, used for sorting the joint_limits.yaml file into alphabetical order
 * \param jm1 - a pointer to the first joint model to compare
 * \param jm2 - a pointer to the second joint model to compare
 * \return bool of alphabetical sorting comparison
 */
struct JointModelCompare
{
  bool operator()(const moveit::core::JointModel* jm1, const moveit::core::JointModel* jm2) const
  {
    return jm1->getName() < jm2->getName();
  }
};

bool SRDFConfig::GeneratedJointLimits::writeYaml(YAML::Emitter& emitter)
{
  emitter << YAML::Comment("joint_limits.yaml allows the dynamics properties specified in the URDF "
                           "to be overwritten or augmented as needed");
  emitter << YAML::Newline;

  emitter << YAML::BeginMap;

  emitter << YAML::Comment("For beginners, we downscale velocity and acceleration limits.") << YAML::Newline;
  emitter << YAML::Comment("You can always specify higher scaling factors (<= 1.0) in your motion requests.");
  emitter << YAML::Comment("Increase the values below to 1.0 to always move at maximum speed.");
  emitter << YAML::Key << "default_velocity_scaling_factor";
  emitter << YAML::Value << "0.1";

  emitter << YAML::Key << "default_acceleration_scaling_factor";
  emitter << YAML::Value << "0.1";

  emitter << YAML::Newline << YAML::Newline;
  emitter << YAML::Comment("Specific joint properties can be changed with the keys "
                           "[max_position, min_position, max_velocity, max_acceleration]")
          << YAML::Newline;
  emitter << YAML::Comment("Joint limits can be turned off with [has_velocity_limits, has_acceleration_limits]");

  emitter << YAML::Key << "joint_limits";
  emitter << YAML::Value << YAML::BeginMap;

  // Union all the joints in groups. Uses a custom comparator to allow the joints to be sorted by name
  std::set<const moveit::core::JointModel*, JointModelCompare> joints;

  // Loop through groups
  for (srdf::Model::Group& group : parent_.srdf_.groups_)
  {
    // Get list of associated joints
    const moveit::core::JointModelGroup* joint_model_group = parent_.getRobotModel()->getJointModelGroup(group.name_);

    const std::vector<const moveit::core::JointModel*>& joint_models = joint_model_group->getJointModels();

    // Iterate through the joints
    for (const moveit::core::JointModel* joint_model : joint_models)
    {
      // Check that this joint only represents 1 variable.
      if (joint_model->getVariableCount() == 1)
        joints.insert(joint_model);
    }
  }

  // Add joints to yaml file, if no more than 1 dof
  for (const moveit::core::JointModel* joint : joints)
  {
    emitter << YAML::Key << joint->getName();
    emitter << YAML::Value << YAML::BeginMap;

    const moveit::core::VariableBounds& b = joint->getVariableBounds()[0];

    // Output property
    emitter << YAML::Key << "has_velocity_limits";
    if (b.velocity_bounded_)
      emitter << YAML::Value << "true";
    else
      emitter << YAML::Value << "false";

    // Output property
    emitter << YAML::Key << "max_velocity";
    emitter << YAML::Value << std::min(fabs(b.max_velocity_), fabs(b.min_velocity_));

    // Output property
    emitter << YAML::Key << "has_acceleration_limits";
    if (b.acceleration_bounded_)
      emitter << YAML::Value << "true";
    else
      emitter << YAML::Value << "false";

    // Output property
    emitter << YAML::Key << "max_acceleration";
    emitter << YAML::Value << std::min(fabs(b.max_acceleration_), fabs(b.min_acceleration_));

    emitter << YAML::EndMap;
  }

  emitter << YAML::EndMap;
  return true;  // file created successfully
}

}  // namespace moveit_setup

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(moveit_setup::SRDFConfig, moveit_setup::SetupConfig)
