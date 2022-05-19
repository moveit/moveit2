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

#include <moveit_setup_controllers/controller_config.hpp>
#include <moveit_setup_framework/data/srdf_config.hpp>
#include <moveit_setup_framework/data_warehouse.hpp>

namespace moveit_setup
{
namespace controllers
{
void ControllersConfig::loadPrevious(const std::filesystem::path& package_path, const YAML::Node& /*node*/)
{
  changed_ = false;
  // Load ros controllers yaml file if available-----------------------------------------------
  std::filesystem::path ros_controllers_yaml_path = package_path / CONTROLLERS_YAML;
  std::ifstream input_stream(ros_controllers_yaml_path);
  if (!input_stream.good())
  {
    RCLCPP_WARN_STREAM((*logger_), "Does not exist " << ros_controllers_yaml_path);
    return;
  }

  // Begin parsing
  try
  {
    // Used in parsing controllers
    ControllerInfo control_setting;
    YAML::Node controllers = YAML::Load(input_stream);

    // Loop through all controllers
    for (YAML::const_iterator controller_it = controllers.begin(); controller_it != controllers.end(); ++controller_it)
    {
      // Follow Joint Trajectory action controllers
      if (controller_it->first.as<std::string>() == "controller_list")
      {
        if (!parseController(controller_it->second))
          return;
      }
      // Other settings found in the ros_controllers file
      else
      {
        const std::string& controller_name = controller_it->first.as<std::string>();
        control_setting.joints_.clear();

        // Push joints if found in the controller
        if (const YAML::Node& joints = controller_it->second["joints"])
        {
          if (joints.IsSequence())
          {
            for (YAML::const_iterator joint_it = joints.begin(); joint_it != joints.end(); ++joint_it)
            {
              control_setting.joints_.push_back(joint_it->as<std::string>());
            }
          }
          else
          {
            control_setting.joints_.push_back(joints.as<std::string>());
          }
        }

        // If the setting has joints then it is a controller that needs to be parsed
        if (!control_setting.joints_.empty())
        {
          if (const YAML::Node& urdf_node = controller_it->second["type"])
          {
            control_setting.type_ = controller_it->second["type"].as<std::string>();
            control_setting.name_ = controller_name;
            ros_controllers_config_.push_back(control_setting);
            control_setting.joints_.clear();
          }
        }
      }
    }
  }
  catch (YAML::ParserException& e)  // Catch errors
  {
    RCLCPP_ERROR_STREAM((*logger_), e.what());
  }
}

// ******************************************************************************************
// Helper function for parsing an individual Controller from ros_controllers yaml file
// ******************************************************************************************
bool ControllersConfig::parseController(const YAML::Node& controller)
{
  // Used in parsing controllers
  ControllerInfo control_setting;

  if (const YAML::Node& trajectory_controllers = controller)
  {
    for (const YAML::Node& trajectory_controller : trajectory_controllers)
    {
      // Controller node
      if (const YAML::Node& controller_node = trajectory_controller)
      {
        if (const YAML::Node& joints = controller_node["joints"])
        {
          control_setting.joints_.clear();
          for (YAML::const_iterator joint_it = joints.begin(); joint_it != joints.end(); ++joint_it)
          {
            control_setting.joints_.push_back(joint_it->as<std::string>());
          }
          if (!getYamlProperty(controller_node, "name", control_setting.name_))
          {
            RCLCPP_ERROR_STREAM((*logger_), "Couldn't parse ros_controllers.yaml");
            return false;
          }
          if (!getYamlProperty(controller_node, "type", control_setting.type_))
          {
            RCLCPP_ERROR_STREAM((*logger_), "Couldn't parse ros_controllers.yaml");
            return false;
          }
          // All required fields were parsed correctly
          ros_controllers_config_.push_back(control_setting);
        }
        else
        {
          RCLCPP_ERROR_STREAM((*logger_), "Couldn't parse ros_controllers.yaml");
          return false;
        }
      }
    }
  }
  return true;
}

// ******************************************************************************************
// Adds a controller to ros_controllers_config_ vector
// ******************************************************************************************
bool ControllersConfig::addController(const std::string& name, const std::string& type,
                                      const std::vector<std::string>& joint_names)
{
  ControllerInfo controller;
  controller.name_ = name;
  controller.type_ = type;
  controller.joints_ = joint_names;
  return addController(controller);
}

// ******************************************************************************************
// Adds a controller to ros_controllers_config_ vector
// ******************************************************************************************
bool ControllersConfig::addController(const ControllerInfo& new_controller)
{
  // Used for holding our search results
  ControllerInfo* searched_ros_controller = nullptr;

  // Find if there is an existing controller with the same name
  searched_ros_controller = findControllerByName(new_controller.name_);

  if (searched_ros_controller && searched_ros_controller->type_ == new_controller.type_)
    return false;

  ros_controllers_config_.push_back(new_controller);
  return true;
}

// ******************************************************************************************
// Find controller by name
// ******************************************************************************************
ControllerInfo* ControllersConfig::findControllerByName(const std::string& controller_name)
{
  // Find the Controller we are editing based on the Controller name string
  ControllerInfo* searched_ros_controller = nullptr;  // used for holding our search results

  for (ControllerInfo& ros_control_config : ros_controllers_config_)
  {
    if (ros_control_config.name_ == controller_name)  // string match
    {
      searched_ros_controller = &ros_control_config;  // convert to pointer from iterator
      break;                                          // we are done searching
    }
  }

  return searched_ros_controller;
}

// ******************************************************************************************
// Deletes a controller by name
// ******************************************************************************************
bool ControllersConfig::deleteController(const std::string& controller_name)
{
  for (std::vector<ControllerInfo>::iterator controller_it = ros_controllers_config_.begin();
       controller_it != ros_controllers_config_.end(); ++controller_it)
  {
    if (controller_it->name_ == controller_name)  // string match
    {
      ros_controllers_config_.erase(controller_it);
      // we are done searching
      return true;
    }
  }
  return false;
}

// ******************************************************************************************
// Helper function to write the FollowJointTrajectory for each planning group to ros_controller.yaml,
// and erases the controller that have been written, to avoid mixing between FollowJointTrajectory
// which are published under the namespace of 'controller_list' and other types of controllers.
// ******************************************************************************************
void outputFollowJointTrajectoryYAML(YAML::Emitter& emitter, std::vector<ControllerInfo>& ros_controllers_config_output)
{
  // Write default controllers
  emitter << YAML::Key << "controller_list";
  emitter << YAML::Value << YAML::BeginSeq;
  {
    for (std::vector<ControllerInfo>::iterator controller_it = ros_controllers_config_output.begin();
         controller_it != ros_controllers_config_output.end();)
    {
      // Depending on the controller type, fill the required data
      if (controller_it->type_ == "FollowJointTrajectory")
      {
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "name";
        emitter << YAML::Value << controller_it->name_;
        emitter << YAML::Key << "action_ns";
        emitter << YAML::Value << "follow_joint_trajectory";
        emitter << YAML::Key << "default";
        emitter << YAML::Value << "True";
        emitter << YAML::Key << "type";
        emitter << YAML::Value << controller_it->type_;
        // Write joints
        emitter << YAML::Key << "joints";
        {
          if (controller_it->joints_.size() != 1)
          {
            emitter << YAML::Value << YAML::BeginSeq;

            // Iterate through the joints
            for (std::string& joint : controller_it->joints_)
            {
              emitter << joint;
            }
            emitter << YAML::EndSeq;
          }
          else
          {
            emitter << YAML::Value << YAML::BeginMap;
            emitter << controller_it->joints_[0];
            emitter << YAML::EndMap;
          }
        }
        controller_it = ros_controllers_config_output.erase(controller_it);
        emitter << YAML::EndMap;
      }
      else
      {
        controller_it++;
      }
    }
    emitter << YAML::EndSeq;
  }
}

// ******************************************************************************************
// Output controllers config files
// ******************************************************************************************
bool ControllersConfig::GeneratedControllersConfig::writeYaml(YAML::Emitter& /*emitter*/)
{
  // Copy ros_control_config_ to a new vector to avoid modifying it
  std::vector<ControllerInfo> ros_controllers_config_output(parent_.ros_controllers_config_);

  // Cache the joints' names.
  std::vector<std::vector<std::string>> planning_groups;
  std::vector<std::string> group_joints;

  auto srdf_config = parent_.config_data_->get<SRDFConfig>("srdf");

  // We are going to write the joints names many times.
  // Loop through groups to store the joints names in group_joints vector and reuse is.
  // TODO: Finish refactoring this method
  /*for (srdf::Model::Group& group : srdf_->groups_)
  {
    // Get list of associated joints
    const moveit::core::JointModelGroup* joint_model_group = getRobotModel()->getJointModelGroup(group.name_);
    const std::vector<const moveit::core::JointModel*>& joint_models = joint_model_group->getActiveJointModels();
    // Iterate through the joints and push into group_joints vector.
    for (const moveit::core::JointModel* joint : joint_models)
    {
      if (joint->isPassive() || joint->getMimic() != nullptr || joint->getType() == moveit::core::JointModel::FIXED)
        continue;
      else
        group_joints.push_back(joint->getName());
    }
    // Push all the group joints into planning_groups vector.
    planning_groups.push_back(group_joints);
    group_joints.clear();
}*/

  /*

  emitter << YAML::BeginMap;

  {
    emitter << YAML::Comment("Simulation settings for using moveit_sim_controllers");
    emitter << YAML::Key << "moveit_sim_hw_interface" << YAML::Value << YAML::BeginMap;
    // MoveIt Simulation Controller settings for setting initial pose
    {
      // Use the first planning group if initial joint_model_group was not set, else write a default value
      emitter << YAML::Key << "joint_model_group";
      emitter << YAML::Value << getDefaultStartPose().group_;

      // Use the first robot pose if initial joint_model_group_pose was not set, else write a default value
      emitter << YAML::Key << "joint_model_group_pose";
      emitter << YAML::Value << getDefaultStartPose().name_;

      emitter << YAML::EndMap;
    }
    // Settings for ros_control control loop
    emitter << YAML::Newline;
    emitter << YAML::Comment("Settings for ros_control_boilerplate control loop");
    emitter << YAML::Key << "generic_hw_control_loop" << YAML::Value << YAML::BeginMap;
    {
      emitter << YAML::Key << "loop_hz";
      emitter << YAML::Value << "300";
      emitter << YAML::Key << "cycle_time_error_threshold";
      emitter << YAML::Value << "0.01";
      emitter << YAML::EndMap;
    }
    // Settings for ros_control hardware interface
    emitter << YAML::Newline;
    emitter << YAML::Comment("Settings for ros_control hardware interface");
    emitter << YAML::Key << "hardware_interface" << YAML::Value << YAML::BeginMap;
    {
      // Get list of all joints for the robot
      const std::vector<const moveit::core::JointModel*>& joint_models = getRobotModel()->getJointModels();

      emitter << YAML::Key << "joints";
      {
        if (joint_models.size() != 1)
        {
          emitter << YAML::Value << YAML::BeginSeq;
          // Iterate through the joints
          for (std::vector<const moveit::core::JointModel*>::const_iterator joint_it = joint_models.begin();
               joint_it < joint_models.end(); ++joint_it)
          {
            if ((*joint_it)->isPassive() || (*joint_it)->getMimic() != nullptr ||
                (*joint_it)->getType() == moveit::core::JointModel::FIXED)
              continue;
            else
              emitter << (*joint_it)->getName();
          }
          emitter << YAML::EndSeq;
        }
        else
        {
          emitter << YAML::Value << YAML::BeginMap;
          emitter << joint_models[0]->getName();
          emitter << YAML::EndMap;
        }
      }
      emitter << YAML::Key << "sim_control_mode";
      emitter << YAML::Value << "1";
      emitter << YAML::Comment("0: position, 1: velocity");
      emitter << YAML::Newline;
      emitter << YAML::EndMap;
    }
    // Joint State Controller
    emitter << YAML::Comment("Publish all joint states");
    emitter << YAML::Newline << YAML::Comment("Creates the /joint_states topic necessary in ROS");
    emitter << YAML::Key << "joint_state_broadcaster" << YAML::Value << YAML::BeginMap;
    {
      emitter << YAML::Key << "type";
      emitter << YAML::Value << "joint_state_broadcaster/JointStateBroadcaster";
      emitter << YAML::Key << "publish_rate";
      emitter << YAML::Value << "50";
      emitter << YAML::EndMap;
    }

    // Writes Follow Joint Trajectory controllers to ros_controller.yaml
    outputFollowJointTrajectoryYAML(emitter, ros_controllers_config_output);

    for (const auto& controller : ros_controllers_config_output)
    {
      emitter << YAML::Key << controller.name_;
      emitter << YAML::Value << YAML::BeginMap;
      emitter << YAML::Key << "type";
      emitter << YAML::Value << controller.type_;

      // Write joints
      emitter << YAML::Key << "joints";
      {
        if (controller.joints_.size() != 1)
        {
          emitter << YAML::Value << YAML::BeginSeq;

          // Iterate through the joints
          for (const std::string& joint : controller.joints_)
          {
            emitter << joint;
          }
          emitter << YAML::EndSeq;
        }
        else
        {
          emitter << YAML::Value << YAML::BeginMap;
          emitter << controller.joints_[0];
          emitter << YAML::EndMap;
        }
      }
      // Write gains as they are required for vel and effort controllers
      emitter << YAML::Key << "gains";
      emitter << YAML::Value << YAML::BeginMap;
      {
        // Iterate through the joints
        for (const std::string& joint : controller.joints_)
        {
          emitter << YAML::Key << joint << YAML::Value << YAML::BeginMap;
          emitter << YAML::Key << "p";
          emitter << YAML::Value << "100";
          emitter << YAML::Key << "d";
          emitter << YAML::Value << "1";
          emitter << YAML::Key << "i";
          emitter << YAML::Value << "1";
          emitter << YAML::Key << "i_clamp";
          emitter << YAML::Value << "1" << YAML::EndMap;
        }
        emitter << YAML::EndMap;
      }
      emitter << YAML::EndMap;
    }
  }
  */
  return true;
}

void ControllersConfig::collectVariables(std::vector<TemplateVariable>& variables)
{
  // Add Controllers variable for ros_controllers.launch file
  std::stringstream controllers;
  for (ControllerInfo& controller : ros_controllers_config_)
  {
    // Check if the controller belongs to controller_list namespace
    if (controller.type_ != "FollowJointTrajectory")
      controllers << controller.name_ << " ";
  }

  variables.push_back(TemplateVariable("ROS_CONTROLLERS", controllers.str()));
}

}  // namespace controllers
}  // namespace moveit_setup

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(moveit_setup::controllers::ControllersConfig, moveit_setup::SetupConfig)
