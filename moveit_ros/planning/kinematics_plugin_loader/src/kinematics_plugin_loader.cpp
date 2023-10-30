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

/* Author: Ioan Sucan, Dave Coleman */

#include <moveit/kinematics_plugin_loader/kinematics_plugin_loader.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>
#include <sstream>
#include <vector>
#include <map>
#include <memory>
#include <mutex>
#include <moveit/utils/logger.hpp>

namespace kinematics_plugin_loader
{
class KinematicsPluginLoader::KinematicsLoaderImpl
{
public:
  /**
   * \brief Pimpl Implementation of KinematicsLoader
   * \param robot_description
   * \param possible_kinematics_solvers
   * \param search_res
   */
  KinematicsLoaderImpl(const rclcpp::Node::SharedPtr& node, const std::string& robot_description,
                       const std::map<std::string, std::string>& possible_kinematics_solvers,
                       const std::map<std::string, double>& search_res, const rclcpp::Logger& logger)
    : node_(node)
    , robot_description_(robot_description)
    , possible_kinematics_solvers_(possible_kinematics_solvers)
    , search_res_(search_res)
    , logger_(logger)
  {
    try
    {
      kinematics_loader_ =
          std::make_shared<pluginlib::ClassLoader<kinematics::KinematicsBase>>("moveit_core", "kinematics::"
                                                                                              "KinematicsBase");
    }
    catch (pluginlib::PluginlibException& e)
    {
      RCLCPP_ERROR(logger_, "Unable to construct kinematics loader. Error: %s", e.what());
    }
  }

  /**
   * \brief Helper function to decide which, and how many, tip frames a planning group has
   * \param jmg - joint model group pointer
   * \return tips - list of valid links in a planning group to plan for
   */
  std::vector<std::string> chooseTipFrames(const moveit::core::JointModelGroup* jmg)
  {
    std::vector<std::string> tips;
    // get the last link in the chain
    RCLCPP_DEBUG(logger_,
                 "Choosing tip frame of kinematic solver for group %s"
                 "based on last link in chain",
                 jmg->getName().c_str());

    tips.push_back(jmg->getLinkModels().back()->getName());

    // Error check
    if (tips.empty())
    {
      RCLCPP_ERROR(logger_, "Error choosing kinematic solver tip frame(s).");
    }

    // Debug tip choices
    std::stringstream tip_debug;
    tip_debug << "Planning group '" << jmg->getName() << "' has tip(s): ";
    for (const auto& tip : tips)
      tip_debug << tip << ", ";
    RCLCPP_DEBUG_STREAM(logger_, tip_debug.str());

    return tips;
  }

  kinematics::KinematicsBasePtr allocKinematicsSolver(const moveit::core::JointModelGroup* jmg)
  {
    kinematics::KinematicsBasePtr result;
    if (!kinematics_loader_)
    {
      RCLCPP_ERROR(logger_, "Invalid kinematics loader.");
      return result;
    }
    if (!jmg)
    {
      RCLCPP_ERROR(logger_, "Specified group is nullptr. Cannot allocate kinematics solver.");
      return result;
    }
    const std::vector<const moveit::core::LinkModel*>& links = jmg->getLinkModels();
    if (links.empty())
    {
      RCLCPP_ERROR(logger_, "No links specified for group '%s'. Cannot allocate kinematics solver.",
                   jmg->getName().c_str());
      return result;
    }

    RCLCPP_DEBUG(logger_, "Trying to allocate kinematics solver for group '%s'", jmg->getName().c_str());

    const std::string& base = links.front()->getParentJointModel()->getParentLinkModel() ?
                                  links.front()->getParentJointModel()->getParentLinkModel()->getName() :
                                  jmg->getParentModel().getModelFrame();

    // just to be sure, do not call the same pluginlib instance allocation function in parallel
    std::scoped_lock slock(lock_);
    for (const auto& [group, solver] : possible_kinematics_solvers_)
    {
      // Don't bother trying to load a solver for the wrong group
      if (group != jmg->getName())
      {
        continue;
      }
      try
      {
        result = kinematics_loader_->createUniqueInstance(solver);
        if (result)
        {
          // choose the tip of the IK solver
          const std::vector<std::string> tips = chooseTipFrames(jmg);

          // choose search resolution
          double search_res = search_res_.find(jmg->getName())->second;  // we know this exists, by construction

          if (!result->initialize(node_, jmg->getParentModel(), jmg->getName(),
                                  (base.empty() || base[0] != '/') ? base : base.substr(1), tips, search_res))
          {
            RCLCPP_ERROR(logger_, "Kinematics solver of type '%s' could not be initialized for group '%s'",
                         solver.c_str(), jmg->getName().c_str());
            result.reset();
            continue;
          }

          result->setDefaultTimeout(jmg->getDefaultIKTimeout());
          RCLCPP_DEBUG(logger_,
                       "Successfully allocated and initialized a kinematics solver of type '%s' with search "
                       "resolution %lf for group '%s' at address %p",
                       solver.c_str(), search_res, jmg->getName().c_str(), result.get());
          break;
        }
      }
      catch (pluginlib::PluginlibException& e)
      {
        RCLCPP_ERROR(logger_, "The kinematics plugin (%s) failed to load. Error: %s", solver.c_str(), e.what());
      }
    }

    if (!result)
    {
      RCLCPP_DEBUG(logger_, "No usable kinematics solver was found for this group.\n"
                            "Did you load kinematics.yaml into your node's namespace?");
    }
    return result;
  }

  // cache solver between two consecutive calls
  // first call in RobotModelLoader::loadKinematicsSolvers() is just to check suitability for jmg
  // second call in JointModelGroup::setSolverAllocators() is to actually retrieve the instance for use
  kinematics::KinematicsBasePtr allocKinematicsSolverWithCache(const moveit::core::JointModelGroup* jmg)
  {
    std::scoped_lock slock(cache_lock_);
    kinematics::KinematicsBasePtr& cached = instances_[jmg];
    if (cached.unique())
      return std::move(cached);  // pass on unique instance

    // create a new instance and store in instances_
    cached = allocKinematicsSolver(jmg);
    return cached;
  }

  void status() const
  {
    for (const auto& [group, solver] : possible_kinematics_solvers_)
    {
      RCLCPP_INFO(logger_, "Solver for group '%s': '%s' (search resolution = %lf)", group.c_str(), solver.c_str(),
                  search_res_.at(group));
    }
  }

private:
  const rclcpp::Node::SharedPtr node_;
  std::string robot_description_;
  std::map<std::string, std::string> possible_kinematics_solvers_;
  std::map<std::string, double> search_res_;
  std::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase>> kinematics_loader_;
  std::map<const moveit::core::JointModelGroup*, kinematics::KinematicsBasePtr> instances_;
  std::mutex lock_;
  std::mutex cache_lock_;
  rclcpp::Logger logger_;
};

void KinematicsPluginLoader::status() const
{
  if (loader_)
  {
    loader_->status();
  }
  else
  {
    RCLCPP_INFO(logger_, "Loader function was never required");
  }
}

moveit::core::SolverAllocatorFn KinematicsPluginLoader::getLoaderFunction(const srdf::ModelSharedPtr& srdf_model)
{
  if (!loader_)
  {
    RCLCPP_DEBUG(logger_, "Configuring kinematics solvers");
    groups_.clear();

    std::map<std::string, std::string> possible_kinematics_solvers;
    std::map<std::string, double> search_res;
    std::map<std::string, std::vector<std::string>> iksolver_to_tip_links;

    if (srdf_model)
    {
      const std::vector<srdf::Model::Group>& known_groups = srdf_model->getGroups();

      RCLCPP_DEBUG(logger_, "Loading kinematics parameters...");
      // read the list of plugin names for possible kinematics solvers
      for (const srdf::Model::Group& known_group : known_groups)
      {
        std::string kinematics_param_prefix = robot_description_ + "_kinematics." + known_group.name_;
        group_param_listener_.try_emplace(known_group.name_,
                                          std::make_shared<kinematics::ParamListener>(node_, kinematics_param_prefix));
        group_params_.try_emplace(known_group.name_, group_param_listener_.at(known_group.name_)->get_params());

        std::string kinematics_solver_param_name = kinematics_param_prefix + ".kinematics_solver";
        const auto kinematics_solver = group_params_.at(known_group.name_).kinematics_solver;

        if (kinematics_solver.empty())
        {
          RCLCPP_DEBUG(logger_, "No kinematics solver specified for group '%s'.", known_group.name_.c_str());
          continue;
        }

        // Only push back a group if it has a kinematics solver.
        groups_.push_back(known_group.name_);

        possible_kinematics_solvers[known_group.name_] = kinematics_solver;
        RCLCPP_DEBUG(logger_, "Found kinematics solver '%s' for group '%s'.", kinematics_solver.c_str(),
                     known_group.name_.c_str());

        std::string kinematics_solver_res_param_name = kinematics_param_prefix + ".kinematics_solver_search_resolution";
        const auto kinematics_solver_search_resolution =
            group_params_.at(known_group.name_).kinematics_solver_search_resolution;
        search_res[known_group.name_] = kinematics_solver_search_resolution;
        RCLCPP_DEBUG(logger_, "Found param %s : %f", kinematics_solver_res_param_name.c_str(),
                     kinematics_solver_search_resolution);

        std::string kinematics_solver_timeout_param_name = kinematics_param_prefix + ".kinematics_solver_timeout";
        const auto kinematics_solver_timeout = group_params_.at(known_group.name_).kinematics_solver_timeout;
        ik_timeout_[known_group.name_] = kinematics_solver_timeout;
        RCLCPP_DEBUG(logger_, "Found param %s : %f", kinematics_solver_timeout_param_name.c_str(),
                     kinematics_solver_timeout);
      }
    }

    loader_ = std::make_shared<KinematicsLoaderImpl>(node_, robot_description_, possible_kinematics_solvers, search_res,
                                                     logger_);
  }

  return [&loader = *loader_](const moveit::core::JointModelGroup* jmg) {
    return loader.allocKinematicsSolverWithCache(jmg);
  };
}
}  // namespace kinematics_plugin_loader
