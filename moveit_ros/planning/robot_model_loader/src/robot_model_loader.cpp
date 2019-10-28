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
#include <moveit/profiler/profiler.h>
#include "rclcpp/rclcpp.hpp"
#include <typeinfo>

rclcpp::Logger LOGGER_ROBOT_MODEL_LOADER = rclcpp::get_logger("planning").get_child("robot_model_loader");

namespace robot_model_loader
{
RobotModelLoader::RobotModelLoader(const std::string& robot_description, bool load_kinematics_solvers)
{
  Options opt(robot_description);
  opt.load_kinematics_solvers_ = load_kinematics_solvers;
  printf("RobotModelLoader Node is not declare!!\n");
  // configure(opt);
}
RobotModelLoader::RobotModelLoader(const std::string& robot_description, std::shared_ptr<rclcpp::Node>& node, bool load_kinematics_solvers)
{
  Options opt(robot_description);
  opt.load_kinematics_solvers_ = load_kinematics_solvers;
  this->node_ = node;
  configure(opt);
}

RobotModelLoader::RobotModelLoader(const Options& opt)
{
  printf("RobotModelLoader Node is not declare!!\n");
  // configure(opt);
}
RobotModelLoader::RobotModelLoader(std::shared_ptr<rclcpp::Node>& node, const Options& opt)
{
  this->node_ = node;
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
bool canSpecifyPosition(const robot_model::JointModel* jmodel, const unsigned int index)
{
  bool ok = false;
  if (jmodel->getType() == robot_model::JointModel::PLANAR && index == 2){
    RCLCPP_ERROR(LOGGER_ROBOT_MODEL_LOADER,"Cannot specify position limits for orientation of planar joint '%s'", jmodel->getName().c_str());
  }
  else if (jmodel->getType() == robot_model::JointModel::FLOATING && index > 2){
    RCLCPP_ERROR(LOGGER_ROBOT_MODEL_LOADER,"Cannot specify position limits for orientation of floating joint '%s'", jmodel->getName().c_str());
  }
  else if (jmodel->getType() == robot_model::JointModel::REVOLUTE &&
           static_cast<const robot_model::RevoluteJointModel*>(jmodel)->isContinuous()){
    RCLCPP_ERROR(LOGGER_ROBOT_MODEL_LOADER,"Cannot specify position limits for continuous joint '%s'", jmodel->getName().c_str());
  }
  else{
    ok = true;
  }
  return ok;
}
}  // namespace

void RobotModelLoader::configure(const Options& opt)
{
  moveit::tools::Profiler::ScopedStart prof_start;
  moveit::tools::Profiler::ScopedBlock prof_block("RobotModelLoader::configure");
  //
  rclcpp::Clock clock;
  rclcpp::Time start = clock.now();
  if (!opt.urdf_string_.empty() && !opt.srdf_string_.empty())
    rdf_loader_.reset(new rdf_loader::RDFLoader(opt.urdf_string_, opt.srdf_string_));
  else
    rdf_loader_.reset(new rdf_loader::RDFLoader(node_, opt.robot_description_));
  if (rdf_loader_->getURDF())
  {
    const srdf::ModelSharedPtr& srdf =
        rdf_loader_->getSRDF() ? rdf_loader_->getSRDF() : srdf::ModelSharedPtr(new srdf::Model());
    model_.reset(new robot_model::RobotModel(rdf_loader_->getURDF(), srdf));
  }

  if (model_ && !rdf_loader_->getRobotDescription().empty())
  {
    moveit::tools::Profiler::ScopedBlock prof_block2("RobotModelLoader::configure joint limits");

    // if there are additional joint limits specified in some .yaml file, read those in
    // TODO (anasarrak): Add the correct node name to the .yaml
    auto additional_joints_parameters = std::make_shared<rclcpp::SyncParametersClient>(node_);

    for (std::size_t i = 0; i < model_->getJointModels().size(); ++i)
    {
      robot_model::JointModel* jmodel = model_->getJointModels()[i];
      std::vector<moveit_msgs::msg::JointLimits> jlim = jmodel->getVariableBoundsMsg();
      for (std::size_t j = 0; j < jlim.size(); ++j)
      {
        std::string prefix = rdf_loader_->getRobotDescription() + "_planning/joint_limits/" + jlim[j].joint_name + "/";

        double max_position;
        if (additional_joints_parameters->has_parameter(prefix + "max_position"))
        {
          max_position = node_->get_parameter(prefix + "max_position").get_value<double>();
          if (canSpecifyPosition(jmodel, j))
          {
            jlim[j].has_position_limits = true;
            jlim[j].max_position = max_position;
          }
        }
        double min_position;
        if (additional_joints_parameters->has_parameter(prefix + "min_position"))
        {
          min_position = node_->get_parameter(prefix + "min_position").get_value<double>();
          if (canSpecifyPosition(jmodel, j))
          {
            jlim[j].has_position_limits = true;
            jlim[j].min_position = min_position;
          }
        }
        double max_velocity;
        if (additional_joints_parameters->has_parameter(prefix + "max_velocity"))
        {
          max_velocity = node_->get_parameter(prefix + "max_velocity").get_value<double>();
          jlim[j].has_velocity_limits = true;
          jlim[j].max_velocity = max_velocity;
        }
        bool has_vel_limits;
        if (additional_joints_parameters->has_parameter(prefix + "has_velocity_limits"))
          has_vel_limits = node_->get_parameter(prefix + "has_velocity_limits").get_value<bool>();
          jlim[j].has_velocity_limits = has_vel_limits;

        double max_acc;
        if (additional_joints_parameters->has_parameter(prefix + "max_acceleration"))
        {
          max_acc = node_->get_parameter(prefix + "max_acceleration").get_value<double>();
          jlim[j].has_acceleration_limits = true;
          jlim[j].max_acceleration = max_acc;
        }
        bool has_acc_limits;
        if (additional_joints_parameters->has_parameter(prefix + "has_acceleration_limits"))
        has_acc_limits = node_->get_parameter(prefix + "has_acceleration_limits").get_value<bool>();
          jlim[j].has_acceleration_limits = has_acc_limits;
      }
      jmodel->setVariableBounds(jlim);
    }
  }

  if (model_ && opt.load_kinematics_solvers_)
    loadKinematicsSolvers();

    RCLCPP_DEBUG(node_->get_logger(), "Loaded kinematic model in %d seconds", (clock.now() - start).seconds());
}

void RobotModelLoader::loadKinematicsSolvers(const kinematics_plugin_loader::KinematicsPluginLoaderPtr& kloader)
{
  moveit::tools::Profiler::ScopedStart prof_start;
  moveit::tools::Profiler::ScopedBlock prof_block("RobotModelLoader::loadKinematicsSolvers");

  if (rdf_loader_ && model_)
  {
    // load the kinematics solvers
    if (kloader)
      kinematics_loader_ = kloader;
    else
      kinematics_loader_.reset(
          new kinematics_plugin_loader::KinematicsPluginLoader(rdf_loader_->getRobotDescription()));
    robot_model::SolverAllocatorFn kinematics_allocator = kinematics_loader_->getLoaderFunction(rdf_loader_->getSRDF(), node_);
    const std::vector<std::string>& groups = kinematics_loader_->getKnownGroups();
    std::stringstream ss;
    std::copy(groups.begin(), groups.end(), std::ostream_iterator<std::string>(ss, " "));
    RCLCPP_DEBUG(LOGGER_ROBOT_MODEL_LOADER,"Loaded information about the following groups: '%S' ", ss.str().c_str());
    if (groups.empty() && !model_->getJointModelGroups().empty())
      RCLCPP_WARN(LOGGER_ROBOT_MODEL_LOADER,"No kinematics plugins defined. Fill and load kinematics.yaml!");

    std::map<std::string, robot_model::SolverAllocatorFn> imap;
    for (std::size_t i = 0; i < groups.size(); ++i)
    {
      // Check if a group in kinematics.yaml exists in the srdf
      if (!model_->hasJointModelGroup(groups[i]))
        continue;

      const robot_model::JointModelGroup* jmg = model_->getJointModelGroup(groups[i]);

      kinematics::KinematicsBasePtr solver = kinematics_allocator(jmg);
      if (solver)
      {
        std::string error_msg;
        if (solver->supportsGroup(jmg, &error_msg))
        {
          imap[groups[i]] = kinematics_allocator;
        }
        else
        {
          RCLCPP_ERROR(LOGGER_ROBOT_MODEL_LOADER,"Kinematics solver %s does not support joint group %s.  Error: %s", typeid(*solver).name(),
                    groups[i].c_str(), error_msg.c_str());
        }
      }
      else
      {
        RCLCPP_ERROR(LOGGER_ROBOT_MODEL_LOADER,"Kinematics solver could not be instantiated for joint group %s.", groups[i].c_str());
      }
    }
    model_->setKinematicsAllocators(imap);

    // set the default IK timeouts
    const std::map<std::string, double>& timeout = kinematics_loader_->getIKTimeout();
    for (std::map<std::string, double>::const_iterator it = timeout.begin(); it != timeout.end(); ++it)
    {
      if (!model_->hasJointModelGroup(it->first))
        continue;
      robot_model::JointModelGroup* jmg = model_->getJointModelGroup(it->first);
      jmg->setDefaultIKTimeout(it->second);
    }
  }
}
}  // namespace robot_model_loader
