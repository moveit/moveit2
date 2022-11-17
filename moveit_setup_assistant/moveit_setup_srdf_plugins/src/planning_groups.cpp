/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik Robotics
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

#include <moveit_setup_srdf_plugins/planning_groups.hpp>
#include <moveit/kinematics_base/kinematics_base.h>
#include <pluginlib/class_loader.hpp>  // for loading all avail kinematic planners

//// Cycle checking
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>

namespace moveit_setup
{
namespace srdf_setup
{
// Used for checking for cycles in a subgroup hierarchy
struct CycleDetector : public boost::dfs_visitor<>
{
  CycleDetector(bool& has_cycle) : m_has_cycle(has_cycle)
  {
  }

  template <class Edge, class Graph>
  void backEdge(Edge /*unused*/, Graph& /*unused*/)
  {
    m_has_cycle = true;
  }

protected:
  bool& m_has_cycle;
};

void PlanningGroups::onInit()
{
  SuperSRDFStep::onInit();
  config_data_->registerType("group_meta", "moveit_setup::srdf_setup::GroupMetaConfig");
  group_meta_config_ = config_data_->get<GroupMetaConfig>("group_meta");
}

void PlanningGroups::renameGroup(const std::string& old_group_name, const std::string& new_group_name)
{
  // Rename the actual group
  rename(old_group_name, new_group_name);

  long changes = 0L;

  // Change all references to this group name in other subgroups
  // Loop through every group
  for (srdf::Model::Group& group : srdf_config_->getGroups())
  {
    // Loop through every subgroup
    for (std::string& subgroup : group.subgroups_)
    {
      // Check if that subgroup references old group name. if so, update it
      if (subgroup == old_group_name)  // same name
      {
        subgroup.assign(new_group_name);  // updated
        changes |= GROUP_CONTENTS;
      }
    }
  }

  // Change all references to this group name in the end effectors screen
  for (srdf::Model::EndEffector& eef : srdf_config_->getEndEffectors())
  {
    // Check if this eef's parent group references old group name. if so, update it
    if (eef.parent_group_ == old_group_name)  // same name
    {
      RCLCPP_DEBUG_STREAM(*logger_, "Changed eef '" << eef.name_ << "' to new parent group name " << new_group_name);
      eef.parent_group_ = new_group_name;  // updated
      changes |= END_EFFECTORS;
    }

    // Check if this eef's group references old group name. if so, update it
    if (eef.component_group_.compare(old_group_name) == 0)  // same name
    {
      RCLCPP_DEBUG_STREAM(*logger_, "Changed eef '" << eef.name_ << "' to new group name " << new_group_name);
      eef.component_group_ = new_group_name;  // updated
      changes |= END_EFFECTORS;
    }
  }

  // Change all references to this group name in the robot poses screen
  for (srdf::Model::GroupState& gs : srdf_config_->getGroupStates())
  {
    // Check if this eef's parent group references old group name. if so, update it
    if (gs.group_ == old_group_name)  // same name
    {
      RCLCPP_DEBUG_STREAM(*logger_, "Changed group state group '" << gs.group_ << "' to new parent group name "
                                                                  << new_group_name);
      gs.group_ = new_group_name;  // updated
      changes |= POSES;
    }
  }

  group_meta_config_->renameGroup(old_group_name, new_group_name);
  changes |= GROUPS;

  // Now update the robot model based on our changed to the SRDF
  srdf_config_->updateRobotModel(changes);
}

void PlanningGroups::deleteGroup(const std::string& group_name)
{
  long changes = 0L;

  // Remove poses in this group
  for (const std::string& pose_name : getPosesByGroup(group_name))
  {
    srdf_config_->removePoseByName(pose_name, group_name);
  }

  // Remove end effectors in this group
  auto& eefs = srdf_config_->getEndEffectors();
  auto it = eefs.begin();
  while (it != eefs.end())
  {
    if (it->component_group_ == group_name)
    {
      it = eefs.erase(it);
      changes |= END_EFFECTORS;
    }
    else
    {
      it++;
    }
  }

  // delete actual group
  remove(group_name);

  // Delete references in subgroups
  for (srdf::Model::Group& group : srdf_config_->getGroups())
  {
    auto& subgroups = group.subgroups_;
    std::vector<std::string>::iterator subgroup_it = std::find(subgroups.begin(), subgroups.end(), group_name);
    while (subgroup_it != subgroups.end())
    {
      subgroups.erase(subgroup_it);
      changes |= GROUP_CONTENTS;
      subgroup_it = std::find(subgroups.begin(), subgroups.end(), group_name);
    }
  }

  group_meta_config_->deleteGroup(group_name);

  srdf_config_->updateRobotModel(changes);
}

void PlanningGroups::setJoints(const std::string& group_name, const std::vector<std::string>& joint_names)
{
  // Find the group we are editing based on the group name string
  srdf::Model::Group* searched_group = find(group_name);

  // save the data
  searched_group->joints_ = joint_names;

  // Update the kinematic model with changes
  srdf_config_->updateRobotModel(GROUP_CONTENTS);
}

void PlanningGroups::setLinks(const std::string& group_name, const std::vector<std::string>& link_names)
{
  // Find the group we are editing based on the group name string
  srdf::Model::Group* searched_group = find(group_name);

  // save the data
  searched_group->links_ = link_names;

  // Update the kinematic model with changes
  srdf_config_->updateRobotModel(GROUP_CONTENTS);
}

void PlanningGroups::setChain(const std::string& group_name, const std::string& base, const std::string& tip)
{
  // Check that box the tip and base, or neither, have text
  if ((!tip.empty() && base.empty()) || (tip.empty() && !base.empty()))
  {
    throw std::runtime_error("You must specify a link for both the base and tip, or leave both "
                             "blank.");
  }

  // Check that both given links are valid links, unless they are both blank
  if (!tip.empty() && !base.empty())
  {
    // Check that they are not the same link
    if (tip.compare(base) == 0)  // they are same
    {
      throw std::runtime_error("Tip and base link cannot be the same link.");
    }

    bool found_tip = false;
    bool found_base = false;
    const std::vector<std::string>& links = getLinkNames();

    for (const std::string& link : links)
    {
      // Check if string matches either of user specified links
      if (link.compare(tip) == 0)  // they are same
        found_tip = true;
      else if (link.compare(base) == 0)  // they are same
        found_base = true;

      // Check if we are done searching
      if (found_tip && found_base)
        break;
    }

    // Check if we found both links
    if (!found_tip || !found_base)
    {
      throw std::runtime_error("Tip or base link(s) were not found in kinematic chain.");
    }
  }

  // Find the group we are editing based on the group name string
  srdf::Model::Group* searched_group = find(group_name);

  // clear the old data
  searched_group->chains_.clear();

  // Save the data if there is data to save
  if (!tip.empty() && !base.empty())
  {
    searched_group->chains_.push_back(std::pair<std::string, std::string>(base, tip));
  }

  // Update the kinematic model with changes
  srdf_config_->updateRobotModel(GROUP_CONTENTS);
}

void PlanningGroups::setSubgroups(const std::string& selected_group_name, const std::vector<std::string>& subgroups)
{
  // Check for cycles -------------------------------

  // Create vector index of all nodes
  std::map<std::string, int> group_nodes;

  // Create vector of all nodes for use as id's
  int node_id = 0;
  for (const std::string& group_name : getGroupNames())
  {
    // Add string to vector
    group_nodes.insert(std::pair<std::string, int>(group_name, node_id));
    ++node_id;
  }

  // Create the empty graph
  typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS> Graph;
  Graph g(group_nodes.size());

  // Traverse the group list again, this time inserting subgroups into graph
  int from_id = 0;  // track the outer node we are on to reduce searches performed
  for (srdf::Model::Group& group : srdf_config_->getGroups())
  {
    // Check if group_it is same as current group
    if (group.name_ == selected_group_name)  // yes, same group
    {
      // add new subgroup list from widget, not old one. this way we can check for new cycles
      for (const std::string& to_string : subgroups)
      {
        // convert subgroup string to associated id
        int to_id = group_nodes[to_string];

        // Add edge from from_id to to_id
        add_edge(from_id, to_id, g);
      }
    }
    else  // this group is not the group we are editing, so just add subgroups from memory
    {
      // add new subgroup list from widget, not old one. this way we can check for new cycles
      for (const std::string& to_string : group.subgroups_)
      {
        // Get std::string of subgroup
        // convert subgroup string to associated id
        int to_id = group_nodes[to_string];

        // Add edge from from_id to to_id
        add_edge(from_id, to_id, g);
      }
    }

    ++from_id;
  }

  // Check for cycles
  bool has_cycle = false;
  CycleDetector vis(has_cycle);
  boost::depth_first_search(g, visitor(vis));

  if (has_cycle)
  {
    throw std::runtime_error("Depth first search reveals a cycle in the subgroups");
  }

  // Find the group we are editing based on the group name string
  srdf::Model::Group* searched_group = find(selected_group_name);

  // save the data
  searched_group->subgroups_ = subgroups;

  // Update the kinematic model with changes
  srdf_config_->updateRobotModel(GROUP_CONTENTS);
}

std::string PlanningGroups::getChildOfJoint(const std::string& joint_name) const
{
  return srdf_config_->getChildOfJoint(joint_name);
}

std::string PlanningGroups::getJointType(const std::string& joint_name) const
{
  const moveit::core::JointModel* joint_model = srdf_config_->getRobotModel()->getJointModel(joint_name);
  if (!joint_model)
  {
    return "";
  }
  return joint_model->getTypeName();
}

LinkNameTree PlanningGroups::getLinkNameTree() const
{
  const moveit::core::JointModel* root_joint = srdf_config_->getRobotModel()->getRootJoint();
  return buildLinkNameTree(root_joint->getChildLinkModel());
}

std::vector<std::string> PlanningGroups::getPosesByGroup(const std::string& group_name) const
{
  std::vector<std::string> pose_names;
  for (const srdf::Model::GroupState& pose : srdf_config_->getGroupStates())
  {
    if (pose.group_ == group_name)
    {
      pose_names.push_back(pose.name_);
    }
  }
  return pose_names;
}

std::vector<std::string> PlanningGroups::getEndEffectorsByGroup(const std::string& group_name) const
{
  std::vector<std::string> eef_names;
  for (const srdf::Model::EndEffector& eef : srdf_config_->getEndEffectors())
  {
    if (eef.component_group_ == group_name)
    {
      eef_names.push_back(eef.name_);
    }
  }
  return eef_names;
}

std::vector<std::string> PlanningGroups::getKinematicPlanners() const
{
  // load all avail kin planners
  std::unique_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase>> loader;
  try
  {
    loader = std::make_unique<pluginlib::ClassLoader<kinematics::KinematicsBase>>("moveit_core",
                                                                                  "kinematics::KinematicsBase");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    throw std::runtime_error(std::string("Exception while creating class loader for kinematic "
                                         "solver plugins: ") +
                             ex.what());
  }

  std::vector<std::string> planners(loader->getDeclaredClasses());

  // Warn if no plugins are found
  if (planners.empty())
  {
    throw std::runtime_error("No MoveIt-compatible kinematics solvers found. Try "
                             "installing moveit_kinematics (sudo apt-get install "
                             "ros-${ROS_DISTRO}-moveit-kinematics)");
  }
  return planners;
}

std::vector<std::string> PlanningGroups::getOMPLPlanners() const
{
  std::vector<std::string> planner_names;
  // TODO: This should call ompl_interface::PlanningContextManager::getRegisteredPlannerAllocators to load the
  //       names dynamically
  planner_names.push_back("AnytimePathShortening");
  planner_names.push_back("SBL");
  planner_names.push_back("EST");
  planner_names.push_back("LBKPIECE");
  planner_names.push_back("BKPIECE");
  planner_names.push_back("KPIECE");
  planner_names.push_back("RRT");
  planner_names.push_back("RRTConnect");
  planner_names.push_back("RRTstar");
  planner_names.push_back("TRRT");
  planner_names.push_back("PRM");
  planner_names.push_back("PRMstar");
  planner_names.push_back("FMT");
  planner_names.push_back("BFMT");
  planner_names.push_back("PDST");
  planner_names.push_back("STRIDE");
  planner_names.push_back("BiTRRT");
  planner_names.push_back("LBTRRT");
  planner_names.push_back("BiEST");
  planner_names.push_back("ProjEST");
  planner_names.push_back("LazyPRM");
  planner_names.push_back("LazyPRMstar");
  planner_names.push_back("SPARS");
  planner_names.push_back("SPARStwo");

  return planner_names;
}

}  // namespace srdf_setup
}  // namespace moveit_setup
