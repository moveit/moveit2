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

namespace kinematics_plugin_loader
{
rclcpp::Logger LOGGER = rclcpp::get_logger("kinematics_plugin_loader");
class KinematicsPluginLoader::KinematicsLoaderImpl
{
public:
  /**
   * \brief Pimpl Implementation of KinematicsLoader
   * \param robot_description
   * \param possible_kinematics_solvers
   * \param search_res
   * \param iksolver_to_tip_links - a map between each ik solver and a vector of custom-specified tip link(s)
   */
  KinematicsLoaderImpl(const rclcpp::Node::SharedPtr& node, const std::string& robot_description,
                       const std::map<std::string, std::vector<std::string>>& possible_kinematics_solvers,
                       const std::map<std::string, std::vector<double>>& search_res,
                       const std::map<std::string, std::vector<std::string>>& iksolver_to_tip_links)
    : node_(node)
    , robot_description_(robot_description)
    , possible_kinematics_solvers_(possible_kinematics_solvers)
    , search_res_(search_res)
    , iksolver_to_tip_links_(iksolver_to_tip_links)
  {
    try
    {
      kinematics_loader_ =
          std::make_shared<pluginlib::ClassLoader<kinematics::KinematicsBase>>("moveit_core", "kinematics::"
                                                                                              "KinematicsBase");
    }
    catch (pluginlib::PluginlibException& e)
    {
      RCLCPP_ERROR(LOGGER, "Unable to construct kinematics loader. Error: %s", e.what());
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
    std::map<std::string, std::vector<std::string>>::const_iterator ik_it = iksolver_to_tip_links_.find(jmg->getName());

    // Check if tips were loaded onto the rosparam server previously
    if (ik_it != iksolver_to_tip_links_.end())
    {
      // the tip is being chosen based on a corresponding rosparam ik link
      RCLCPP_DEBUG(LOGGER,
                   "Choosing tip frame of kinematic solver for group %s"
                   "based on values in rosparam server.",
                   jmg->getName().c_str());
      tips = ik_it->second;
    }
    else
    {
      // get the last link in the chain
      RCLCPP_DEBUG(LOGGER,
                   "Choosing tip frame of kinematic solver for group %s"
                   "based on last link in chain",
                   jmg->getName().c_str());

      tips.push_back(jmg->getLinkModels().back()->getName());
    }

    // Error check
    if (tips.empty())
    {
      RCLCPP_ERROR(LOGGER, "Error choosing kinematic solver tip frame(s).");
    }

    // Debug tip choices
    std::stringstream tip_debug;
    tip_debug << "Planning group '" << jmg->getName() << "' has tip(s): ";
    for (const auto& tip : tips)
      tip_debug << tip << ", ";
    RCLCPP_DEBUG_STREAM(LOGGER, tip_debug.str());

    return tips;
  }

  kinematics::KinematicsBasePtr allocKinematicsSolver(const moveit::core::JointModelGroup* jmg)
  {
    kinematics::KinematicsBasePtr result;
    if (!kinematics_loader_)
    {
      RCLCPP_ERROR(LOGGER, "Invalid kinematics loader.");
      return result;
    }
    if (!jmg)
    {
      RCLCPP_ERROR(LOGGER, "Specified group is nullptr. Cannot allocate kinematics solver.");
      return result;
    }
    const std::vector<const moveit::core::LinkModel*>& links = jmg->getLinkModels();
    if (links.empty())
    {
      RCLCPP_ERROR(LOGGER, "No links specified for group '%s'. Cannot allocate kinematics solver.",
                   jmg->getName().c_str());
      return result;
    }

    RCLCPP_DEBUG(LOGGER, "Trying to allocate kinematics solver for group '%s'", jmg->getName().c_str());

    std::map<std::string, std::vector<std::string>>::const_iterator it =
        possible_kinematics_solvers_.find(jmg->getName());
    if (it == possible_kinematics_solvers_.end())
    {
      RCLCPP_DEBUG(LOGGER, "No kinematics solver available for this group");
      return result;
    }

    const std::string& base = links.front()->getParentJointModel()->getParentLinkModel() ?
                                  links.front()->getParentJointModel()->getParentLinkModel()->getName() :
                                  jmg->getParentModel().getModelFrame();

    // just to be sure, do not call the same pluginlib instance allocation function in parallel
    std::scoped_lock slock(lock_);
    for (std::size_t i = 0; !result && i < it->second.size(); ++i)
    {
      try
      {
        result = kinematics_loader_->createUniqueInstance(it->second[i]);
        if (result)
        {
          // choose the tip of the IK solver
          const std::vector<std::string> tips = chooseTipFrames(jmg);

          // choose search resolution
          double search_res = search_res_.find(jmg->getName())->second[i];  // we know this exists, by construction

          if (!result->initialize(node_, jmg->getParentModel(), jmg->getName(),
                                  (base.empty() || base[0] != '/') ? base : base.substr(1), tips, search_res))
          {
            RCLCPP_ERROR(LOGGER, "Kinematics solver of type '%s' could not be initialized for group '%s'",
                         it->second[i].c_str(), jmg->getName().c_str());
            result.reset();
            continue;
          }

          result->setDefaultTimeout(jmg->getDefaultIKTimeout());
          RCLCPP_DEBUG(LOGGER,
                       "Successfully allocated and initialized a kinematics solver of type '%s' with search "
                       "resolution %lf for group '%s' at address %p",
                       it->second[i].c_str(), search_res, jmg->getName().c_str(), result.get());
          break;
        }
      }
      catch (pluginlib::PluginlibException& e)
      {
        RCLCPP_ERROR(LOGGER, "The kinematics plugin (%s) failed to load. Error: %s", it->first.c_str(), e.what());
      }
    }

    if (!result)
    {
      RCLCPP_DEBUG(LOGGER, "No usable kinematics solver was found for this group.\n"
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
    for (std::map<std::string, std::vector<std::string>>::const_iterator it = possible_kinematics_solvers_.begin();
         it != possible_kinematics_solvers_.end(); ++it)
    {
      for (std::size_t i = 0; i < it->second.size(); ++i)
      {
        RCLCPP_INFO(LOGGER, "Solver for group '%s': '%s' (search resolution = %lf)", it->first.c_str(),
                    it->second[i].c_str(), search_res_.at(it->first)[i]);
      }
    }
  }

private:
  const rclcpp::Node::SharedPtr node_;
  std::string robot_description_;
  std::map<std::string, std::vector<std::string>> possible_kinematics_solvers_;
  std::map<std::string, std::vector<double>> search_res_;
  std::map<std::string, std::vector<std::string>> iksolver_to_tip_links_;  // a map between each ik solver and a vector
                                                                           // of custom-specified tip link(s)
  std::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase>> kinematics_loader_;
  std::map<const moveit::core::JointModelGroup*, kinematics::KinematicsBasePtr> instances_;
  std::mutex lock_;
  std::mutex cache_lock_;
};

void KinematicsPluginLoader::status() const
{
  if (loader_)
    loader_->status();
  else
    RCLCPP_INFO(LOGGER, "Loader function was never required");
}

moveit::core::SolverAllocatorFn KinematicsPluginLoader::getLoaderFunction()
{
  if (loader_)
    return [&loader = *loader_](const moveit::core::JointModelGroup* jmg) {
      return loader.allocKinematicsSolverWithCache(jmg);
    };

  rdf_loader::RDFLoader rml(node_, robot_description_);
  robot_description_ = rml.getRobotDescription();
  return getLoaderFunction(rml.getSRDF());
}

moveit::core::SolverAllocatorFn KinematicsPluginLoader::getLoaderFunction(const srdf::ModelSharedPtr& srdf_model)
{
  if (!loader_)
  {
    RCLCPP_DEBUG(LOGGER, "Configuring kinematics solvers");
    groups_.clear();

    std::map<std::string, std::vector<std::string>> possible_kinematics_solvers;
    std::map<std::string, std::vector<double>> search_res;
    std::map<std::string, std::vector<std::string>> iksolver_to_tip_links;

    if (srdf_model)
    {
      const std::vector<srdf::Model::Group>& known_groups = srdf_model->getGroups();

      RCLCPP_DEBUG(LOGGER, "Loading kinematics parameters...");
      // read the list of plugin names for possible kinematics solvers
      for (const srdf::Model::Group& known_group : known_groups)
      {
        groups_.push_back(known_group.name_);

        std::string kinematics_param_prefix = robot_description_ + "_kinematics." + known_group.name_;
        kinematics_param_listener_.try_emplace(
            known_group.name_, std::make_shared<kinematics::ParamListener>(node_, kinematics_param_prefix));
        kinematics_params_.try_emplace(known_group.name_,
                                       kinematics_param_listener_.at(known_group.name_)->get_params());

        std::string kinematics_solver_param_name = kinematics_param_prefix + ".kinematics_solver";
        const auto kinematics_solvers = kinematics_params_.at(known_group.name_).kinematics_solver;

        for (auto solver : kinematics_solvers)
        {
          possible_kinematics_solvers[known_group.name_].push_back(solver);
          RCLCPP_DEBUG(LOGGER, "Found kinematics solver '%s' for group '%s'.", solver.c_str(),
                       known_group.name_.c_str());
        }

        std::string kinematics_solver_res_param_name = kinematics_param_prefix + ".kinematics_solver_search_resolution";
        const auto kinematics_solver_search_resolutions =
            kinematics_params_.at(known_group.name_).kinematics_solver_search_resolution;

        for (auto kinematics_solver_search_resolution : kinematics_solver_search_resolutions)
        {
          search_res[known_group.name_].push_back(kinematics_solver_search_resolution);
          RCLCPP_DEBUG(LOGGER, "Found param %s : %f", kinematics_solver_res_param_name.c_str(),
                       kinematics_solver_search_resolution);
        }

        std::string kinematics_solver_timeout_param_name = kinematics_param_prefix + ".kinematics_solver_timeout";
        const auto kinematics_solver_timeout = kinematics_params_.at(known_group.name_).kinematics_solver_timeout;
        RCLCPP_DEBUG(LOGGER, "Found param %s : %f", kinematics_solver_timeout_param_name.c_str(),
                     kinematics_solver_timeout);
        ik_timeout_[known_group.name_] = kinematics_solver_timeout;

        // Allow a kinematic solver's tip links to be specified on the rosparam server as an array
        std::string ksolver_ik_links_param_name = kinematics_param_prefix + ".kinematics_solver_ik_links";
        const auto kinematics_solver_ik_links = kinematics_params_.at(known_group.name_).kinematics_solver_ik_links;
        for (auto& kinematics_solver_ik_link : kinematics_solver_ik_links)
        {
          RCLCPP_DEBUG(LOGGER, "Found tip %s for group %s", kinematics_solver_ik_link.c_str(),
                       known_group.name_.c_str());
          iksolver_to_tip_links[known_group.name_].push_back(kinematics_solver_ik_link);
        }
      }
    }

    loader_ = std::make_shared<KinematicsLoaderImpl>(node_, robot_description_, possible_kinematics_solvers, search_res,
                                                     iksolver_to_tip_links);
  }

  return [&loader = *loader_](const moveit::core::JointModelGroup* jmg) {
    return loader.allocKinematicsSolverWithCache(jmg);
  };
}
}  // namespace kinematics_plugin_loader
