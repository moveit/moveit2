/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ioan A. Sucan
 *  Copyright (c) 2013, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <moveit/exceptions/exceptions.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/revolute_joint_model.h>
#include <moveit/robot_model/robot_model.h>
#include <range/v3/all.hpp>
#include "order_robot_model_items.inc"
#include "rclcpp/rclcpp.hpp"

namespace views = ::ranges::views;
namespace actions = ::ranges::actions;

namespace moveit
{
namespace core
{
namespace
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_robot_model.joint_model_group");

// check if a parent or ancestor of joint is included in this group
bool includesParent(const JointModel* joint, const JointModelGroup* group)
{
  bool found = false;
  // if we find that an ancestor is also in the group, then the joint is not a root
  while (joint->getParentLinkModel() != nullptr)
  {
    joint = joint->getParentLinkModel()->getParentJointModel();
    if (group->hasJointModel(joint->getName()) && joint->getVariableCount() > 0 && joint->getMimic() == nullptr)
    {
      found = true;
      break;
    }
    else if (joint->getMimic() != nullptr)
    {
      const JointModel* mjoint = joint->getMimic();
      if (group->hasJointModel(mjoint->getName()) && mjoint->getVariableCount() > 0 && mjoint->getMimic() == nullptr)
        found = true;
      else if (includesParent(mjoint, group))
        found = true;
      if (found)
        break;
    }
  }
  return found;
}

// check if joint a is right below b, in the kinematic chain, with no active DOF missing
bool jointPrecedes(const JointModel* a, const JointModel* b)
{
  if (!a->getParentLinkModel())
    return false;
  const JointModel* p = a->getParentLinkModel()->getParentJointModel();
  while (p)
  {
    if (p == b)
      return true;
    if (p->getType() == JointModel::FIXED)
      p = p->getParentLinkModel() ? p->getParentLinkModel()->getParentJointModel() : nullptr;
    else
      break;
  }

  return false;
}

// These are used as filter functions
const auto is_fixed_joint = [](const auto& jm) { return jm->getVariableCount() == 0; };
const auto is_mimic_joint = [](const auto& jm) { return jm->getMimic() != nullptr; };
const auto has_geometry = [](const auto& lm) { return !lm->getShapes().empty(); };

// These are transform functions
const auto get_name = [](const auto& element) { return element->getName(); };
const auto get_variable_count = [](const auto& jm) { return jm->getVariableCount(); };
const auto get_variable_bounds_addr = [](const auto& jm) { return &jm->getVariableBounds(); };
const auto get_first_variable_index = [](const auto& jm) { return jm->getFirstVariableIndex(); };
const auto get_variable_names_view = [](const auto& jm) { return views::all(jm->getVariableNames()); };
const auto get_variable_names_size = [](const auto& jm) { return jm->getVariableNames().size(); };

// Helper to convert a tuple to a pair.  This is useful as a transform after
// zipping two views inorder to initialize a map.
const auto tuple_to_pair = [](const auto& element) {
  return std::make_pair(std::get<0>(element), std::get<1>(element));
};
}  // namespace

JointModelGroup::JointModelGroup(const std::string& group_name, const srdf::Model::Group& config,
                                 const std::vector<const JointModel*>& unsorted_group_joints,
                                 const RobotModel* parent_model)
  : parent_model_{ parent_model }
  , name_{ group_name }
  , common_root_{ nullptr }
  , variable_count_{ 0 }
  , active_variable_count_{ 0 }
  , is_contiguous_index_list_{ true }
  , is_chain_{ false }
  , is_single_dof_{ true }
  , config_{ config }
{
  // Disable clang-format to keep nicer formatting of ranges
  // clang-format off
  joint_model_vector_ = unsorted_group_joints
    | ranges::to<std::vector>()
    | actions::sort(OrderJointsByIndex());
  joint_model_name_vector_ = joint_model_vector_
    | views::transform(get_name)
    | ranges::to<std::vector>();

  // These views that are re-used for several calculations.  These are not const because
  // views that are const cause compilation issues.  Rational:
  // https://ericniebler.github.io/range-v3/index.html#autotoc_md6
  auto not_fixed_view = joint_model_vector_
    | views::remove_if(is_fixed_joint);
  auto start_index_view =
    views::concat(views::single(0),
      not_fixed_view
        | views::transform(get_variable_count)
        | views::partial_sum
      )
    | views::drop_last(1);

  // figure out active joints, mimic joints, fixed joints
  // construct index maps, list of variables
  active_joint_model_vector_ = not_fixed_view
    | views::remove_if(is_mimic_joint)
    | ranges::to<std::vector>();
  active_joint_model_name_vector_ = active_joint_model_vector_
    | views::transform(get_name)
    | ranges::to<std::vector>();
  fixed_joints_ = joint_model_vector_
    | views::filter(is_fixed_joint)
    | ranges::to<std::vector>();
  mimic_joints_ = not_fixed_view
    | views::filter(is_mimic_joint)
    | ranges::to<std::vector>();
  continuous_joint_model_vector_ = not_fixed_view
    | views::filter([](const auto& jm) {
        return jm->getType() == JointModel::REVOLUTE && static_cast<const RevoluteJointModel*>(jm)->isContinuous();
    })
    | ranges::to<std::vector>();
  variable_names_ = not_fixed_view
    | views::transform(get_variable_names_view)
    | views::join
    | ranges::to<std::vector>();
  variable_names_set_ = variable_names_
    | ranges::to<std::set>();
  joint_model_map_ = joint_model_vector_
    | views::transform([](const auto& jm) { return std::make_pair(jm->getName(), jm); })
    | ranges::to<std::map>();
  joint_variables_index_map_ =
    views::concat(
      views::zip(not_fixed_view | views::transform(get_variable_names_view),
                 start_index_view)
        | views::transform([](const auto& e) {
          const auto& [names, start_index] = e;
          return views::zip(names,
                            views::iota(start_index, start_index + names.size()));
        })
        | views::join
        | views::transform(tuple_to_pair)
      ,
      views::zip(not_fixed_view | views::transform(get_name),
                 start_index_view)
        | views::transform(tuple_to_pair)
    )
    | ranges::to<std::map>();
  active_joint_models_bounds_ = active_joint_model_vector_
    | views::transform(get_variable_bounds_addr)
    | ranges::to<std::vector>();
  variable_index_list_ =
    views::zip(not_fixed_view | views::transform(get_first_variable_index),
               not_fixed_view | views::transform(get_variable_names_size))
    | views::transform([](const auto& e) {
     const auto& [first_index, names_size] = e;
     return views::iota(first_index, first_index + names_size);
    })
    | views::join
    | ranges::to<std::vector>();
  active_joint_model_start_index_ =
    views::zip(start_index_view, not_fixed_view)
    | views::remove_if([](const auto& e) { return is_mimic_joint(std::get<1>(e)); })
    | views::transform([](const auto& e) { return std::get<0>(e); })
    | ranges::to<std::vector>();
  variable_count_ = ranges::accumulate(not_fixed_view | views::transform(get_variable_count), 0U);
  active_variable_count_ = ranges::accumulate(not_fixed_view | views::transform(get_variable_count), 0U);
  is_single_dof_ = ranges::none_of(not_fixed_view | views::transform(get_variable_count),
    [](const auto& e) { return e > 1; });

  // now we need to find all the set of joints within this group
  // that root distinct subtrees
  joint_roots_ = active_joint_model_vector_
    | views::filter([this](const auto& jm) {
      return !includesParent(jm, this);
    })
    | ranges::to<std::vector>();

  // when updating this group within a state, it is useful to know
  // if the full state of a group is contiguous within the full state of the robot
  const auto is_ascending = [](const auto& sub_view) {
    const auto& values = sub_view | ranges::to<std::vector>();
    return values[0] + 1 == values[1];
  };
  is_contiguous_index_list_ = !variable_index_list_.empty()
    && ranges::all_of(variable_index_list_ | views::sliding(2), is_ascending);

  // when updating/sampling a group state only,
  // only mimic joints that have their parent within the group get updated.
  group_mimic_update_ = mimic_joints_
    | views::filter([this](const auto& jm) { return hasJointModel(jm->getMimic()->getName()); })
    | views::transform([this](const auto& jm) {
      int src = joint_variables_index_map_[jm->getMimic()->getName()];
      int dest = joint_variables_index_map_[jm->getName()];
      return GroupMimicUpdate(src, dest, jm->getMimicFactor(), jm->getMimicOffset());
    })
    | ranges::to<std::vector>();

  // now we need to make another pass for group links (we include the fixed joints here)
  link_model_vector_ = joint_model_vector_
    | views::transform([](const auto& jm) { return jm->getChildLinkModel(); })
    | views::unique
    | ranges::to<std::vector>()
    | actions::sort(OrderLinksByIndex());
  link_model_map_ = link_model_vector_
    | views::transform([](const auto& lm) { return std::make_pair(lm->getName(), lm); })
    | ranges::to<std::map>();
  link_model_name_vector_ = link_model_vector_
    | views::transform(get_name)
    | ranges::to<std::vector>();
  link_model_with_geometry_vector_ = link_model_vector_
    | views::filter(has_geometry)
    | ranges::to<std::vector>();
  link_model_with_geometry_name_vector_ = link_model_with_geometry_vector_
    | views::transform(get_name)
    | ranges::to<std::vector>();

  // compute the common root of this group
  if (!joint_roots_.empty())
  {
    common_root_ = ranges::accumulate(joint_roots_, joint_roots_[0],
      [this](const auto& left, const auto& right) {
        return parent_model_->getCommonRoot(left, right);
      }
    );
  }

  // compute updated links
  updated_link_model_set_ = joint_roots_
    | views::transform([](const auto& jr) { return views::all(jr->getDescendantLinkModels()); })
    | views::join
    | ranges::to<std::set>();
  updated_link_model_name_set_ = updated_link_model_set_
    | views::transform(get_name)
    | ranges::to<std::set>();
  updated_link_model_vector_ = updated_link_model_set_
    | ranges::to<std::vector>()
    | actions::sort(OrderLinksByIndex());
  updated_link_model_with_geometry_vector_ = updated_link_model_set_
    | views::filter(has_geometry)
    | ranges::to<std::vector>()
    | actions::sort(OrderLinksByIndex());
  updated_link_model_with_geometry_set_ = updated_link_model_with_geometry_vector_
    | ranges::to<std::set>();
  updated_link_model_with_geometry_name_set_ = updated_link_model_with_geometry_vector_
    | views::transform(get_name)
    | ranges::to<std::set>();
  updated_link_model_name_vector_ = updated_link_model_vector_
    | views::transform(get_name)
    | ranges::to<std::vector>();
  updated_link_model_with_geometry_name_vector_ = updated_link_model_with_geometry_vector_
    | views::transform(get_name)
    | ranges::to<std::vector>();

  // check if this group should actually be a chain
  if (joint_roots_.size() == 1 && !active_joint_model_vector_.empty())
  {
    const auto joint_precedes = [](const auto& sub_view) {
      const auto& values = sub_view | ranges::to<std::vector>();
      return jointPrecedes(values[1], values[0]);
    };
    is_chain_ = ranges::all_of(joint_model_vector_ | views::sliding(2), joint_precedes);
  }

  // clang-format on
}

JointModelGroup::~JointModelGroup() = default;

void JointModelGroup::setSubgroupNames(const std::vector<std::string>& subgroups)
{
  subgroup_names_ = subgroups;
  subgroup_names_set_ = subgroup_names_ | ranges::to<std::set>();
}

std::vector<const JointModelGroup*> JointModelGroup::getSubgroups() const
{
  return subgroup_names_ |
         views::transform([this](const auto& name) { return parent_model_->getJointModelGroup(name); }) |
         ranges::to<std::vector>();
}

bool JointModelGroup::hasJointModel(const std::string& joint) const
{
  return joint_model_map_.find(joint) != joint_model_map_.end();
}

bool JointModelGroup::hasLinkModel(const std::string& link) const
{
  return link_model_map_.find(link) != link_model_map_.end();
}

const LinkModel* JointModelGroup::getLinkModel(const std::string& name) const
{
  LinkModelMapConst::const_iterator it = link_model_map_.find(name);
  if (it == link_model_map_.end())
  {
    RCLCPP_ERROR(LOGGER, "Link '%s' not found in group '%s'", name.c_str(), name_.c_str());
    return nullptr;
  }
  return it->second;
}

const JointModel* JointModelGroup::getJointModel(const std::string& name) const
{
  JointModelMapConst::const_iterator it = joint_model_map_.find(name);
  if (it == joint_model_map_.end())
  {
    RCLCPP_ERROR(LOGGER, "Joint '%s' not found in group '%s'", name.c_str(), name_.c_str());
    return nullptr;
  }
  return it->second;
}

void JointModelGroup::getVariableRandomPositions(random_numbers::RandomNumberGenerator& rng, double* values,
                                                 const JointBoundsVector& active_joint_bounds) const
{
  assert(active_joint_bounds.size() == active_joint_model_vector_.size());
  for (const auto& [i, joint_model] : active_joint_model_vector_ | views::enumerate)
  {
    joint_model->getVariableRandomPositions(rng, values + active_joint_model_start_index_[i], *active_joint_bounds[i]);
  }

  updateMimicJoints(values);
}

void JointModelGroup::getVariableRandomPositionsNearBy(random_numbers::RandomNumberGenerator& rng, double* values,
                                                       const JointBoundsVector& active_joint_bounds, const double* near,
                                                       double distance) const
{
  assert(active_joint_bounds.size() == active_joint_model_vector_.size());
  for (const auto& [i, joint_model] : active_joint_model_vector_ | views::enumerate)
  {
    joint_model->getVariableRandomPositionsNearBy(rng, values + active_joint_model_start_index_[i],
                                                  *active_joint_bounds[i], near + active_joint_model_start_index_[i],
                                                  distance);
  }

  updateMimicJoints(values);
}

void JointModelGroup::getVariableRandomPositionsNearBy(random_numbers::RandomNumberGenerator& rng, double* values,
                                                       const JointBoundsVector& active_joint_bounds, const double* near,
                                                       const std::map<JointModel::JointType, double>& distance_map) const
{
  assert(active_joint_bounds.size() == active_joint_model_vector_.size());
  for (const auto& [i, joint_model] : active_joint_model_vector_ | views::enumerate)
  {
    const double distance = [&]() {
      if (const auto iter = distance_map.find(joint_model->getType()); iter != distance_map.end())
      {
        return iter->second;
      }
      else
      {
        RCLCPP_WARN(LOGGER, "Did not pass in distance for '%s'", joint_model->getName().c_str());
        return 0.0;
      }
    }();
    joint_model->getVariableRandomPositionsNearBy(rng, values + active_joint_model_start_index_[i],
                                                  *active_joint_bounds[i], near + active_joint_model_start_index_[i],
                                                  distance);
  }
  updateMimicJoints(values);
}

void JointModelGroup::getVariableRandomPositionsNearBy(random_numbers::RandomNumberGenerator& rng, double* values,
                                                       const JointBoundsVector& active_joint_bounds, const double* near,
                                                       const std::vector<double>& distances) const
{
  assert(active_joint_bounds.size() == active_joint_model_vector_.size());
  if (distances.size() != active_joint_model_vector_.size())
    throw Exception("When sampling random values nearby for group '" + name_ + "', distances vector should be of size " +
                    boost::lexical_cast<std::string>(active_joint_model_vector_.size()) + ", but it is of size " +
                    boost::lexical_cast<std::string>(distances.size()));
  for (const auto& [i, joint_model] : active_joint_model_vector_ | views::enumerate)
  {
    joint_model->getVariableRandomPositionsNearBy(rng, values + active_joint_model_start_index_[i],
                                                  *active_joint_bounds[i], near + active_joint_model_start_index_[i],
                                                  distances[i]);
  }
  updateMimicJoints(values);
}

bool JointModelGroup::satisfiesPositionBounds(const double* state, const JointBoundsVector& active_joint_bounds,
                                              double margin) const
{
  assert(active_joint_bounds.size() == active_joint_model_vector_.size());
  for (std::size_t i = 0; i < active_joint_model_vector_.size(); ++i)
    for (const auto& [i, joint_model] : active_joint_model_vector_ | views::enumerate)
    {
      if (!joint_model->satisfiesPositionBounds(state + active_joint_model_start_index_[i], *active_joint_bounds[i],
                                                margin))
        return false;
    }
  return true;
}

bool JointModelGroup::enforcePositionBounds(double* state, const JointBoundsVector& active_joint_bounds) const
{
  assert(active_joint_bounds.size() == active_joint_model_vector_.size());
  bool change = false;
  for (const auto& [i, joint_model] : active_joint_model_vector_ | views::enumerate)
  {
    if (joint_model->enforcePositionBounds(state + active_joint_model_start_index_[i], *active_joint_bounds[i]))
      change = true;
  }
  if (change)
    updateMimicJoints(state);
  return change;
}

double JointModelGroup::getMaximumExtent(const JointBoundsVector& active_joint_bounds) const
{
  double max_distance = 0.0;
  for (const auto& [i, joint_model] : active_joint_model_vector_ | views::enumerate)
  {
    max_distance += joint_model->getMaximumExtent(*active_joint_bounds[i]) * joint_model->getDistanceFactor();
  }
  return max_distance;
}

double JointModelGroup::distance(const double* state1, const double* state2) const
{
  double d = 0.0;
  for (const auto& [i, joint_model] : active_joint_model_vector_ | views::enumerate)
  {
    d += joint_model->getDistanceFactor() * joint_model->distance(state1 + active_joint_model_start_index_[i],
                                                                  state2 + active_joint_model_start_index_[i]);
  }
  return d;
}

void JointModelGroup::interpolate(const double* from, const double* to, double t, double* state) const
{
  // we interpolate values only for active joint models (non-mimic)
  for (const auto& [i, joint_model] : active_joint_model_vector_ | views::enumerate)
  {
    joint_model->interpolate(from + active_joint_model_start_index_[i], to + active_joint_model_start_index_[i], t,
                             state + active_joint_model_start_index_[i]);
  }

  // now we update mimic as needed
  updateMimicJoints(state);
}

void JointModelGroup::updateMimicJoints(double* values) const
{
  // update mimic (only local joints as we are dealing with a local group state)
  for (const GroupMimicUpdate& mimic_update : group_mimic_update_)
    values[mimic_update.dest] = values[mimic_update.src] * mimic_update.factor + mimic_update.offset;
}

void JointModelGroup::addDefaultState(const std::string& name, const std::map<std::string, double>& default_state)
{
  default_states_[name] = default_state;
  default_states_names_.push_back(name);
}

bool JointModelGroup::getVariableDefaultPositions(const std::string& name, std::map<std::string, double>& values) const
{
  std::map<std::string, std::map<std::string, double> >::const_iterator it = default_states_.find(name);
  if (it == default_states_.end())
    return false;
  values = it->second;
  return true;
}

void JointModelGroup::getVariableDefaultPositions(double* values) const
{
  for (const auto& [i, joint_model] : active_joint_model_vector_ | views::enumerate)
  {
    joint_model->getVariableDefaultPositions(values + active_joint_model_start_index_[i]);
  }
  updateMimicJoints(values);
}

void JointModelGroup::getVariableDefaultPositions(std::map<std::string, double>& values) const
{
  const auto default_positions = [=]() {
    auto tmp = std::vector<double>(variable_count_);
    getVariableDefaultPositions(&tmp[0]);
    return tmp;
  }();
  for (const auto& [name, default_position] : views::zip(variable_names_, default_positions))
  {
    values[name] = default_position;
  }
}

void JointModelGroup::setEndEffectorName(const std::string& name)
{
  end_effector_name_ = name;
}

void JointModelGroup::setEndEffectorParent(const std::string& group, const std::string& link)
{
  end_effector_parent_.first = group;
  end_effector_parent_.second = link;
}

void JointModelGroup::attachEndEffector(const std::string& eef_name)
{
  attached_end_effector_names_.push_back(eef_name);
}

bool JointModelGroup::getEndEffectorTips(std::vector<std::string>& tips) const
{
  // Get a vector of tip links
  std::vector<const LinkModel*> tip_links;
  if (!getEndEffectorTips(tip_links))
    return false;

  // Convert to string names
  tips = tip_links | views::transform(get_name) | ranges::to<std::vector>();
  return true;
}

bool JointModelGroup::getEndEffectorTips(std::vector<const LinkModel*>& tips) const
{
  tips.clear();
  for (const std::string& name : getAttachedEndEffectorNames())
  {
    const JointModelGroup* eef = parent_model_->getEndEffector(name);
    if (!eef)
    {
      RCLCPP_ERROR(LOGGER, "Unable to find joint model group for eef");
      return false;
    }
    const std::string& eef_parent = eef->getEndEffectorParentGroup().second;

    const LinkModel* eef_link = parent_model_->getLinkModel(eef_parent);
    if (!eef_link)
    {
      RCLCPP_ERROR(LOGGER, "Unable to find end effector link for eef");
      return false;
    }
    // insert eef_link into tips, maintaining a *sorted* vector, thus enabling use of std::lower_bound
    const auto insert_it = std::lower_bound(tips.cbegin(), tips.cend(), eef_link);
    if (insert_it == tips.end() || eef_link != *insert_it)  // only insert if not a duplicate
      tips.insert(insert_it, eef_link);
  }
  return true;
}

const LinkModel* JointModelGroup::getOnlyOneEndEffectorTip() const
{
  std::vector<const LinkModel*> tips;
  getEndEffectorTips(tips);
  if (tips.size() == 1)
    return tips.front();
  else if (tips.size() > 1)
  {
    RCLCPP_ERROR(LOGGER, "More than one end effector tip found for joint model group, so cannot return only one");
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "No end effector tips found in joint model group");
  }
  return nullptr;
}

int JointModelGroup::getVariableGroupIndex(const std::string& variable) const
{
  VariableIndexMap::const_iterator it = joint_variables_index_map_.find(variable);
  if (it == joint_variables_index_map_.end())
  {
    RCLCPP_ERROR(LOGGER, "Variable '%s' is not part of group '%s'", variable.c_str(), name_.c_str());
    return -1;
  }
  return it->second;
}

void JointModelGroup::setDefaultIKTimeout(double ik_timeout)
{
  group_kinematics_.first.default_ik_timeout_ = ik_timeout;
  if (group_kinematics_.first.solver_instance_)
    group_kinematics_.first.solver_instance_->setDefaultTimeout(ik_timeout);
  for (std::pair<const JointModelGroup* const, KinematicsSolver>& it : group_kinematics_.second)
    it.second.default_ik_timeout_ = ik_timeout;
}

bool JointModelGroup::computeIKIndexBijection(const std::vector<std::string>& ik_jnames,
                                              std::vector<unsigned int>& joint_bijection) const
{
  joint_bijection.clear();
  for (const std::string& ik_jname : ik_jnames)
  {
    VariableIndexMap::const_iterator it = joint_variables_index_map_.find(ik_jname);
    if (it == joint_variables_index_map_.end())
    {
      // skip reported fixed joints
      if (hasJointModel(ik_jname) && getJointModel(ik_jname)->getType() == JointModel::FIXED)
        continue;
      RCLCPP_ERROR(LOGGER,
                   "IK solver computes joint values for joint '%s' "
                   "but group '%s' does not contain such a joint.",
                   ik_jname.c_str(), getName().c_str());
      return false;
    }
    const JointModel* jm = getJointModel(ik_jname);
    for (unsigned int k = 0; k < jm->getVariableCount(); ++k)
      joint_bijection.push_back(it->second + k);
  }
  return true;
}

void JointModelGroup::setSolverAllocators(const std::pair<SolverAllocatorFn, SolverAllocatorMapFn>& solvers)
{
  if (solvers.first)
  {
    group_kinematics_.first.allocator_ = solvers.first;
    group_kinematics_.first.solver_instance_ = solvers.first(this);
    if (group_kinematics_.first.solver_instance_)
    {
      group_kinematics_.first.solver_instance_->setDefaultTimeout(group_kinematics_.first.default_ik_timeout_);
      if (!computeIKIndexBijection(group_kinematics_.first.solver_instance_->getJointNames(),
                                   group_kinematics_.first.bijection_))
        group_kinematics_.first.reset();
    }
  }
  else
    // we now compute a joint bijection only if we have a solver map
    for (const std::pair<const JointModelGroup* const, SolverAllocatorFn>& it : solvers.second)
      if (it.first->getSolverInstance())
      {
        KinematicsSolver& ks = group_kinematics_.second[it.first];
        ks.allocator_ = it.second;
        ks.solver_instance_ = const_cast<JointModelGroup*>(it.first)->getSolverInstance();
        ks.default_ik_timeout_ = group_kinematics_.first.default_ik_timeout_;
        if (!computeIKIndexBijection(ks.solver_instance_->getJointNames(), ks.bijection_))
        {
          group_kinematics_.second.clear();
          break;
        }
      }
}

bool JointModelGroup::canSetStateFromIK(const std::string& tip) const
{
  const kinematics::KinematicsBaseConstPtr& solver = getSolverInstance();
  if (!solver || tip.empty())
    return false;

  const std::vector<std::string>& tip_frames = solver->getTipFrames();

  if (tip_frames.empty())
  {
    RCLCPP_WARN(LOGGER, "Group %s has no tip frame(s)", name_.c_str());
    return false;
  }

  // loop through all tip frames supported by the JMG
  for (const std::string& tip_frame : tip_frames)
  {
    // remove frame reference, if specified
    const std::string& tip_local = tip[0] == '/' ? tip.substr(1) : tip;
    const std::string& tip_frame_local = tip_frame[0] == '/' ? tip_frame.substr(1) : tip_frame;
    RCLCPP_DEBUG(LOGGER, "comparing input tip: %s to this groups tip: %s ", tip_local.c_str(), tip_frame_local.c_str());

    // Check if the IK solver's tip is the same as the frame of inquiry
    if (tip_local != tip_frame_local)
    {
      // If not the same, check if this planning group includes the frame of inquiry
      if (hasLinkModel(tip_frame_local))
      {
        const LinkModel* lm = getLinkModel(tip_frame_local);
        const LinkTransformMap& fixed_links = lm->getAssociatedFixedTransforms();
        // Check if our frame of inquiry is located anywhere further down the chain (towards the tip of the arm)
        for (const std::pair<const LinkModel* const, Eigen::Isometry3d>& fixed_link : fixed_links)
        {
          if (fixed_link.first->getName() == tip_local)
            return true;
        }
      }
    }
    else
      return true;
  }

  // Did not find any valid tip frame links to use
  return false;
}

void JointModelGroup::printGroupInfo(std::ostream& out) const
{
  out << "Group '" << name_ << "' using " << variable_count_ << " variables\n";
  out << "  * Joints:\n";
  for (const JointModel* joint_model : joint_model_vector_)
    out << "    '" << joint_model->getName() << "' (" << joint_model->getTypeName() << ")\n";
  out << "  * Variables:\n";
  for (const std::string& variable_name : variable_names_)
  {
    int local_idx = joint_variables_index_map_.find(variable_name)->second;
    const JointModel* jm = parent_model_->getJointOfVariable(variable_name);
    out << "    '" << variable_name << "', index "
        << (jm->getFirstVariableIndex() + jm->getLocalVariableIndex(variable_name)) << " in full state, index "
        << local_idx << " in group state";
    if (jm->getMimic())
      out << ", mimic '" << jm->getMimic()->getName() << "'";
    out << '\n';
    out << "        " << parent_model_->getVariableBounds(variable_name) << '\n';
  }
  out << "  * Variables Index List:\n";
  out << "    ";
  for (int variable_index : variable_index_list_)
    out << variable_index << " ";
  if (is_contiguous_index_list_)
    out << "(contiguous)";
  else
    out << "(non-contiguous)";
  out << '\n';
  if (group_kinematics_.first)
  {
    out << "  * Kinematics solver bijection:\n";
    out << "    ";
    for (unsigned int index : group_kinematics_.first.bijection_)
      out << index << " ";
    out << '\n';
  }
  if (!group_kinematics_.second.empty())
  {
    out << "  * Compound kinematics solver:\n";
    for (const std::pair<const JointModelGroup* const, KinematicsSolver>& it : group_kinematics_.second)
    {
      out << "    " << it.first->getName() << ":";
      for (unsigned int index : it.second.bijection_)
        out << " " << index;
      out << '\n';
    }
  }

  if (!group_mimic_update_.empty())
  {
    out << "  * Local Mimic Updates:\n";
    for (const GroupMimicUpdate& mimic_update : group_mimic_update_)
      out << "    [" << mimic_update.dest << "] = " << mimic_update.factor << " * [" << mimic_update.src << "] + "
          << mimic_update.offset << '\n';
  }
  out << '\n';
}

bool JointModelGroup::isValidVelocityMove(const std::vector<double>& from_joint_pose,
                                          const std::vector<double>& to_joint_pose, double dt) const
{
  // Check for equal sized arrays
  if (from_joint_pose.size() != to_joint_pose.size())
  {
    RCLCPP_ERROR(LOGGER, "To and from joint poses are of different sizes.");
    return false;
  }

  return isValidVelocityMove(&from_joint_pose[0], &to_joint_pose[0], from_joint_pose.size(), dt);
}

bool JointModelGroup::isValidVelocityMove(const double* from_joint_pose, const double* to_joint_pose,
                                          std::size_t array_size, double dt) const
{
  const std::vector<const JointModel::Bounds*>& bounds = getActiveJointModelsBounds();
  const std::vector<unsigned int>& bij = getKinematicsSolverJointBijection();

  for (std::size_t i = 0; i < array_size; ++i)
  {
    double dtheta = std::abs(from_joint_pose[i] - to_joint_pose[i]);
    const std::vector<moveit::core::VariableBounds>* var_bounds = bounds[bij[i]];

    if (var_bounds->size() != 1)
    {
      // TODO(davetcoleman) Support multiple variables
      RCLCPP_ERROR(LOGGER, "Attempting to check velocity bounds for waypoint move with joints that have multiple "
                           "variables");
      return false;
    }
    const double max_velocity = (*var_bounds)[0].max_velocity_;

    double max_dtheta = dt * max_velocity;
    if (dtheta > max_dtheta)
    {
      RCLCPP_DEBUG(LOGGER, "Not valid velocity move because of joint %lu", i);
      return false;
    }
  }

  return true;
}
}  // namespace core
}  // end of namespace moveit
