/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan, E. Gil Jones */

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/time.hpp>
#include <typeinfo>

namespace robot_model_loader
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_ros.robot_model_loader");

RobotModelLoader::RobotModelLoader(const rclcpp::Node::SharedPtr& node, const std::string& robot_description,
                                   bool load_kinematics_solvers)
  : node_(node)
{
  Options opt(robot_description);
  opt.load_kinematics_solvers = load_kinematics_solvers;
  configure(opt);
}

RobotModelLoader::RobotModelLoader(const rclcpp::Node::SharedPtr& node, const Options& opt) : node_(node)
{
  configure(opt);
}

RobotModelLoader::~RobotModelLoader()
{
  // Make sure we destroy the robot model first. It contains the loaded
  // kinematics plugins, and those must be destroyed before the pluginlib class
  // that implements them is destroyed (that happens when kinematics_loader_ is
  // destroyed below). This is a workaround - ideally pluginlib would handle
  // this better.
  model_.reset();
  rdf_loader_.reset();
  kinematics_loader_.reset();
}

namespace
{
bool canSpecifyPosition(const moveit::core::JointModel* jmodel, const unsigned int index)
{
  bool ok = false;
  if (jmodel->getType() == moveit::core::JointModel::PLANAR && index == 2)
  {
    RCLCPP_ERROR(LOGGER, "Cannot specify position limits for orientation of planar joint '%s'",
                 jmodel->getName().c_str());
  }
  else if (jmodel->getType() == moveit::core::JointModel::FLOATING && index > 2)
  {
    RCLCPP_ERROR(LOGGER, "Cannot specify position limits for orientation of floating joint '%s'",
                 jmodel->getName().c_str());
  }
  else if (jmodel->getType() == moveit::core::JointModel::REVOLUTE &&
           static_cast<const moveit::core::RevoluteJointModel*>(jmodel)->isContinuous())
  {
    RCLCPP_ERROR(LOGGER, "Cannot specify position limits for continuous joint '%s'", jmodel->getName().c_str());
  }
  else
  {
    ok = true;
  }
  return ok;
}
}  // namespace

void RobotModelLoader::configure(const Options& opt)
{
  rclcpp::Clock clock;
  rclcpp::Time start = clock.now();
  if (!opt.urdf_string_.empty() && !opt.srdf_string.empty())
  {
    rdf_loader_ = std::make_shared<rdf_loader::RDFLoader>(opt.urdf_string_, opt.srdf_string);
  }
  else
  {
    rdf_loader_ = std::make_shared<rdf_loader::RDFLoader>(node_, opt.robot_description);
  }
  if (rdf_loader_->getURDF())
  {
    const srdf::ModelSharedPtr& srdf =
        rdf_loader_->getSRDF() ? rdf_loader_->getSRDF() : std::make_shared<srdf::Model>();
    model_ = std::make_shared<moveit::core::RobotModel>(rdf_loader_->getURDF(), srdf);
  }

  if (model_ && !rdf_loader_->getRobotDescription().empty())
  {
    // if there are additional joint limits specified in some .yaml file, read those in
    for (moveit::core::JointModel* joint_model : model_->getJointModels())
    {
      std::vector<moveit_msgs::msg::JointLimits> joint_limit = joint_model->getVariableBoundsMsg();
      for (std::size_t joint_id = 0; joint_id < joint_limit.size(); ++joint_id)
      {
        std::string prefix =
            rdf_loader_->getRobotDescription() + "_planning.joint_limits." + joint_limit[joint_id].joint_name + ".";

        std::string param_name;
        try
        {
          param_name = prefix + "max_position";
          if (!node_->has_parameter(param_name))
          {
            node_->declare_parameter(param_name, rclcpp::ParameterType::PARAMETER_DOUBLE);
          }
          double max_position;
          if (node_->get_parameter(param_name, max_position))
          {
            if (canSpecifyPosition(joint_model, joint_id))
            {
              joint_limit[joint_id].has_position_limits = true;
              joint_limit[joint_id].max_position = max_position;
            }
          }

          param_name = prefix + "min_position";
          if (!node_->has_parameter(param_name))
          {
            node_->declare_parameter(param_name, rclcpp::ParameterType::PARAMETER_DOUBLE);
          }
          double min_position;
          if (node_->get_parameter(param_name, min_position))
          {
            if (canSpecifyPosition(joint_model, joint_id))
            {
              joint_limit[joint_id].has_position_limits = true;
              joint_limit[joint_id].min_position = min_position;
            }
          }

          // Check if parameter has been declared to avoid exception
          param_name = prefix + "has_velocity_limits";
          if (!node_->has_parameter(param_name))
          {
            node_->declare_parameter(param_name, rclcpp::ParameterType::PARAMETER_BOOL);
          }
          bool has_vel_limits = false;
          if (node_->get_parameter(param_name, has_vel_limits))
            joint_limit[joint_id].has_velocity_limits = has_vel_limits;

          param_name = prefix + "has_acceleration_limits";
          if (!node_->has_parameter(param_name))
          {
            node_->declare_parameter(param_name, rclcpp::ParameterType::PARAMETER_BOOL);
          }
          bool has_acc_limits = false;
          if (node_->get_parameter(param_name, has_acc_limits))
            joint_limit[joint_id].has_acceleration_limits = has_acc_limits;

          param_name = prefix + "has_jerk_limits";
          if (!node_->has_parameter(param_name))
          {
            node_->declare_parameter(param_name, rclcpp::ParameterType::PARAMETER_BOOL);
          }
          bool has_jerk_limits = false;
          if (node_->get_parameter(param_name, has_jerk_limits))
            joint_limit[joint_id].has_jerk_limits = has_jerk_limits;

          if (has_vel_limits)
          {
            param_name = prefix + "max_velocity";
            if (!node_->has_parameter(param_name))
            {
              node_->declare_parameter(param_name, rclcpp::ParameterType::PARAMETER_DOUBLE);
            }

            if (!node_->get_parameter(param_name, joint_limit[joint_id].max_velocity))
            {
              RCLCPP_ERROR(LOGGER, "Specified a velocity limit for joint: %s but did not set a max velocity",
                           joint_limit[joint_id].joint_name.c_str());
            }
          }

          if (has_acc_limits)
          {
            param_name = prefix + "max_acceleration";
            if (!node_->has_parameter(param_name))
            {
              node_->declare_parameter(param_name, rclcpp::ParameterType::PARAMETER_DOUBLE);
            }

            if (!node_->get_parameter(param_name, joint_limit[joint_id].max_acceleration))
            {
              RCLCPP_ERROR(LOGGER, "Specified an acceleration limit for joint: %s but did not set a max acceleration",
                           joint_limit[joint_id].joint_name.c_str());
            }
          }

          if (has_jerk_limits)
          {
            param_name = prefix + "max_jerk";
            if (!node_->has_parameter(param_name))
            {
              node_->declare_parameter(param_name, rclcpp::ParameterType::PARAMETER_DOUBLE);
            }

            if (!node_->get_parameter(param_name, joint_limit[joint_id].max_jerk))
            {
              RCLCPP_ERROR(LOGGER, "Specified a jerk limit for joint: %s but did not set a max jerk",
                           joint_limit[joint_id].joint_name.c_str());
            }
          }
        }
        catch (const rclcpp::ParameterTypeException& e)
        {
          RCLCPP_ERROR_STREAM(LOGGER, "When getting the parameter " << param_name.c_str() << ": " << e.what());
        }
      }
      joint_model->setVariableBounds(joint_limit);
    }
  }

  if (model_ && opt.load_kinematics_solvers)
    loadKinematicsSolvers();

  RCLCPP_DEBUG(node_->get_logger(), "Loaded kinematic model in %f seconds", (clock.now() - start).seconds());
}

void RobotModelLoader::loadKinematicsSolvers(const kinematics_plugin_loader::KinematicsPluginLoaderPtr& kloader)
{
  if (rdf_loader_ && model_)
  {
    // load the kinematics solvers
    if (kloader)
    {
      kinematics_loader_ = kloader;
    }
    else
    {
      kinematics_loader_ =
          std::make_shared<kinematics_plugin_loader::KinematicsPluginLoader>(node_, rdf_loader_->getRobotDescription());
    }
    moveit::core::SolverAllocatorFn kinematics_allocator =
        kinematics_loader_->getLoaderFunction(rdf_loader_->getSRDF());
    const std::vector<std::string>& groups = kinematics_loader_->getKnownGroups();
    std::stringstream ss;
    std::copy(groups.begin(), groups.end(), std::ostream_iterator<std::string>(ss, " "));
    RCLCPP_DEBUG(LOGGER, "Loaded information about the following groups: '%s' ", ss.str().c_str());
    if (groups.empty() && !model_->getJointModelGroups().empty())
      RCLCPP_WARN(LOGGER, "No kinematics plugins defined. Fill and load kinematics.yaml!");

    std::map<std::string, moveit::core::SolverAllocatorFn> imap;
    for (const std::string& group : groups)
    {
      // Check if a group in kinematics.yaml exists in the srdf
      if (!model_->hasJointModelGroup(group))
        continue;

      const moveit::core::JointModelGroup* jmg = model_->getJointModelGroup(group);

      kinematics::KinematicsBasePtr solver = kinematics_allocator(jmg);
      if (solver)
      {
        std::string error_msg;
        if (solver->supportsGroup(jmg, &error_msg))
        {
          imap[group] = kinematics_allocator;
        }
        else
        {
          const auto& s = *solver;  // avoid clang-tidy's -Wpotentially-evaluated-expression
          RCLCPP_ERROR(LOGGER, "Kinematics solver %s does not support joint group %s.  Error: %s", typeid(s).name(),
                       group.c_str(), error_msg.c_str());
        }
      }
      else
      {
        RCLCPP_ERROR(LOGGER, "Kinematics solver could not be instantiated for joint group %s.", group.c_str());
      }
    }
    model_->setKinematicsAllocators(imap);

    // set the default IK timeouts
    const std::map<std::string, double>& timeout = kinematics_loader_->getIKTimeout();
    for (const std::pair<const std::string, double>& it : timeout)
    {
      if (!model_->hasJointModelGroup(it.first))
        continue;
      moveit::core::JointModelGroup* jmg = model_->getJointModelGroup(it.first);
      jmg->setDefaultIKTimeout(it.second);
    }
  }
}
}  // namespace robot_model_loader
