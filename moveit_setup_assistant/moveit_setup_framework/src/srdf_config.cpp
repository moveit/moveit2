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

namespace moveit_setup_framework
{
void SRDFConfig::onInit()
{
  parent_node_->declare_parameter("robot_description_semantic", rclcpp::ParameterType::PARAMETER_STRING);
  has_changes_ = false;
}

void SRDFConfig::loadPrevious(const std::string& package_path, const YAML::Node& node)
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
  node["relative_path"] = srdf_pkg_relative_path_;
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
}

void SRDFConfig::loadSRDFString(const std::string& srdf_string)
{
  loadURDFModel();

  // Verify that file is in correct format / not an XACRO by loading into robot model
  if (!srdf_.initString(*urdf_model_, srdf_string))
  {
    throw std::runtime_error("SRDF file not a valid semantic robot description model.");
  }

  // Set parameter
  parent_node_->set_parameter(rclcpp::Parameter("robot_description_semantic", srdf_string));

  updateRobotModel();

  RCLCPP_INFO_STREAM((*logger_), "Robot semantic model successfully loaded.");
}

void SRDFConfig::loadSRDFFile(const std::string& package_path, const std::string& relative_path)
{
  srdf_pkg_relative_path_ = relative_path;
  loadSRDFFile(appendPaths(package_path, relative_path));
}
// ******************************************************************************************
// Load SRDF File to Parameter Server
// ******************************************************************************************
void SRDFConfig::loadSRDFFile(const std::string& srdf_file_path)
{
  srdf_path_ = srdf_file_path;

  loadURDFModel();

  // empty
  const std::vector<std::string> xacro_args;

  std::string srdf_string;
  if (!rdf_loader::RDFLoader::loadXmlFileToString(srdf_string, srdf_path_, xacro_args))
  {
    throw std::runtime_error("SRDF file not found: " + srdf_path_);
  }

  // Verify that file is in correct format / not an XACRO by loading into robot model
  if (!srdf_.initString(*urdf_model_, srdf_string))
  {
    throw std::runtime_error("SRDF file not a valid semantic robot description model.");
  }

  // Set parameter
  parent_node_->set_parameter(rclcpp::Parameter("robot_description_semantic", srdf_string));

  updateRobotModel();

  RCLCPP_INFO_STREAM((*logger_), "Robot semantic model successfully loaded.");
}

void SRDFConfig::getRelativePath()
{
  if (!srdf_pkg_relative_path_.empty())
  {
    return;
  }
  srdf_pkg_relative_path_ = appendPaths("config", urdf_model_->getName() + ".srdf");
}

void SRDFConfig::updateRobotModel(bool mark_as_changed)
{
  // Initialize with a URDF Model Interface and a SRDF Model
  loadURDFModel();
  robot_model_ = std::make_shared<moveit::core::RobotModel>(urdf_model_, srdf_.srdf_model_);

  if (srdf_pkg_relative_path_.empty())
  {
    srdf_pkg_relative_path_ = appendPaths("config", urdf_model_->getName() + ".srdf");
    srdf_.robot_name_ = urdf_model_->getName();
    has_changes_ = true;
  }
  has_changes_ |= mark_as_changed;

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

void SRDFConfig::collectVariables(std::vector<TemplateVariable>& variables)
{
  variables.push_back(TemplateVariable("ROBOT_NAME", srdf_.robot_name_));
  variables.push_back(TemplateVariable("ROBOT_ROOT_LINK", robot_model_->getRootLinkName()));
  variables.push_back(TemplateVariable("PLANNING_FRAME", robot_model_->getModelFrame()));

  std::stringstream vjb;
  unsigned int counter = 0;
  for (const auto& vj : srdf_.virtual_joints_)
  {
    if (vj.type_ == "fixed")
    {
      continue;
    }
    vjb << "  <node pkg=\"tf2_ros\" type=\"static_transform_publisher\" name=\"virtual_joint_broadcaster_" << counter
        << "\" args=\"0 0 0 0 0 0 " << vj.parent_frame_ << " " << vj.child_link_ << "\" />" << std::endl;
    counter++;
  }
  variables.push_back(TemplateVariable("VIRTUAL_JOINT_BROADCASTER", vjb.str()));
}

}  // namespace moveit_setup_framework

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(moveit_setup_framework::SRDFConfig, moveit_setup_framework::SetupConfig)
