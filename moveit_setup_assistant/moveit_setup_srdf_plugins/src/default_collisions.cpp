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

#include <moveit_setup_srdf_plugins/default_collisions.hpp>

namespace moveit_setup
{
namespace srdf_setup
{
std::vector<std::string> DefaultCollisions::getCollidingLinks()
{
  return srdf_config_->getPlanningScene()->getRobotModel()->getLinkModelNamesWithCollisionGeometry();
}

// Output Link Pairs to SRDF Format and update the collision matrix
// ******************************************************************************************
void DefaultCollisions::linkPairsToSRDF()
{
  // reset the data in the SRDF Writer class
  auto& disabled_list = srdf_config_->getDisabledCollisions();
  disabled_list.clear();

  // Create temp disabled collision
  srdf::Model::CollisionPair dc;

  // copy the data in this class's LinkPairMap datastructure to srdf::Model::CollisionPair format
  for (LinkPairMap::const_iterator pair_it = link_pairs_.begin(); pair_it != link_pairs_.end(); ++pair_it)
  {
    // Only copy those that are actually disabled
    if (pair_it->second.disable_check)
    {
      dc.link1_ = pair_it->first.first;
      dc.link2_ = pair_it->first.second;
      dc.reason_ = disabledReasonToString(pair_it->second.reason);
      disabled_list.push_back(dc);
    }
  }
  srdf_config_->updateRobotModel(COLLISIONS);  // mark as changed
}

void DefaultCollisions::linkPairsToSRDFSorted(size_t skip_mask)
{
  auto& disabled_list = srdf_config_->getDisabledCollisions();
  // Create temp disabled collision
  srdf::Model::CollisionPair dc;

  std::set<srdf::Model::CollisionPair, CollisionPairLess> disabled_collisions;
  for (auto& p : disabled_list)
  {
    if (p.link1_ >= p.link2_)
      std::swap(p.link1_, p.link2_);  // unify link1, link2 sorting
    disabled_collisions.insert(p);
  }

  // copy the data in this class's LinkPairMap datastructure to srdf::Model::CollisionPair format
  for (const std::pair<const std::pair<std::string, std::string>, LinkPairData>& link_pair : link_pairs_)
  {
    // Only copy those that are actually disabled
    if (link_pair.second.disable_check)
    {
      if ((1 << link_pair.second.reason) & skip_mask)
        continue;

      dc.link1_ = link_pair.first.first;
      dc.link2_ = link_pair.first.second;
      if (dc.link1_ >= dc.link2_)
        std::swap(dc.link1_, dc.link2_);
      dc.reason_ = disabledReasonToString(link_pair.second.reason);

      disabled_collisions.insert(dc);
    }
  }

  disabled_list.assign(disabled_collisions.begin(), disabled_collisions.end());
}

// ******************************************************************************************
// Load Link Pairs from SRDF Format
// ******************************************************************************************
void DefaultCollisions::linkPairsFromSRDF()
{
  // Clear all the previous data in the compute_default_collisions tool
  link_pairs_.clear();

  // Create new instance of planning scene using pointer
  planning_scene::PlanningScenePtr scene = srdf_config_->getPlanningScene()->diff();

  // Populate link_pairs_ list with every possible n choose 2 combination of links
  computeLinkPairs(*scene, link_pairs_);

  // Create temp link pair data struct
  LinkPairData link_pair_data;
  std::pair<std::string, std::string> link_pair;

  // Loop through all disabled collisions in SRDF and update the comprehensive list that has already been created
  for (const auto& disabled_collision : srdf_config_->getDisabledCollisions())
  {
    // Set the link names
    link_pair.first = disabled_collision.link1_;
    link_pair.second = disabled_collision.link2_;
    if (link_pair.first >= link_pair.second)
      std::swap(link_pair.first, link_pair.second);

    // Set the link meta data
    link_pair_data.reason = disabledReasonFromString(disabled_collision.reason_);
    link_pair_data.disable_check = true;  // disable checking the collision btw the 2 links

    // Insert into map
    link_pairs_[link_pair] = link_pair_data;
  }
}

void DefaultCollisions::startGenerationThread(unsigned int num_trials, double min_frac, bool verbose)
{
  progress_ = 0;

  // start worker thread
  worker_ =
      boost::thread([this, num_trials, min_frac, verbose] { generateCollisionTable(num_trials, min_frac, verbose); });
}

// ******************************************************************************************
// The worker function to compute the collision matrix
// ******************************************************************************************
void DefaultCollisions::generateCollisionTable(unsigned int num_trials, double min_frac, bool verbose)
{
  const bool include_never_colliding = true;

  // clear previously loaded collision matrix entries
  srdf_config_->getPlanningScene()->getAllowedCollisionMatrixNonConst().clear();

  // Find the default collision matrix - all links that are allowed to collide
  link_pairs_ = computeDefaultCollisions(srdf_config_->getPlanningScene(), &progress_, include_never_colliding,
                                         num_trials, min_frac, verbose);

  // End the progress bar loop
  progress_ = 100;

  RCLCPP_INFO_STREAM(getLogger(), "Thread complete " << link_pairs_.size());
}

void DefaultCollisions::cancelGenerationThread()
{
  worker_.interrupt();
}

void DefaultCollisions::joinGenerationThread()
{
  worker_.join();
}

int DefaultCollisions::getThreadProgress() const
{
  return progress_;
}

bool DefaultCollisions::setDefault(const std::string& name, bool disabled)
{
  bool changed = false;
  if (disabled)
  {
    // add name to no_default_collision_links_ (if not yet in there)
    auto& links = srdf_config_->getNoDefaultCollisionLinks();
    if (std::find(links.begin(), links.end(), name) == links.end())
    {
      links.push_back(name);
      changed = true;
    }
    // remove-erase disabled pairs that are redundant now
    auto& pairs = srdf_config_->getDisabledCollisions();
    auto last = std::remove_if(pairs.begin(), pairs.end(),
                               [&name](const auto& p) { return p.link1_ == name || p.link2_ == name; });
    changed |= last != pairs.end();
    pairs.erase(last, pairs.end());
  }
  else
  {
    // remove-erase name from no_default_collision_links_
    auto& links = srdf_config_->getNoDefaultCollisionLinks();
    {
      auto last = std::remove(links.begin(), links.end(), name);
      changed |= last != links.end();
      links.erase(last, links.end());
    }

    // remove explicitly enabled pairs
    auto& pairs = srdf_config_->getEnabledCollisions();
    auto last = std::remove_if(pairs.begin(), pairs.end(), [&name, &links](const auto& p) {
      return (p.link1_ == name && std::find(links.begin(), links.end(), p.link2_) == links.end()) ||
             (p.link2_ == name && std::find(links.begin(), links.end(), p.link1_) == links.end());
    });
    pairs.erase(last, pairs.end());
  }

  return changed;
}

struct PairMatcher
{
  PairMatcher(const std::string& link1, const std::string& link2)
    : search(link1 < link2 ? std::make_pair(std::cref(link1), std::cref(link2)) :
                             std::make_pair(std::cref(link2), std::cref(link1)))
  {
  }

  bool operator()(const srdf::Model::CollisionPair& pair) const
  {
    return (pair.link1_ == search.first && pair.link2_ == search.second) ||
           (pair.link2_ == search.first && pair.link1_ == search.second);
  }

  std::pair<const std::string&, const std::string&> search;
};

template <typename Container>
auto find(Container& pairs, const std::string& link1, const std::string& link2)
{
  return std::find_if(pairs.begin(), pairs.end(), PairMatcher(link1, link2));
}

bool DefaultCollisions::disabledByDefault(const std::string& link1, const std::string& link2) const
{
  for (const auto& name : srdf_config_->getNoDefaultCollisionLinks())
    if (name == link1 || name == link2)
      return true;
  return false;
}

std::string DefaultCollisions::getCollisionDisablingReason(const std::string& link1, const std::string& link2) const
{
  auto it = find(srdf_config_->getDisabledCollisions(), link1, link2);
  if (it != srdf_config_->getDisabledCollisions().end())
    return it->reason_;
  else if (find(srdf_config_->getEnabledCollisions(), link1, link2) != srdf_config_->getEnabledCollisions().end())
    return COLLISION_DISABLING_REASON_ENABLED;
  else if (disabledByDefault(link1, link2))
    return COLLISION_DISABLING_REASON_DISABLED;
  return "";
}

bool DefaultCollisions::setDefault(const std::string& link1, const std::string& link2, bool disable)
{
  srdf::Model::CollisionPair p{ link1, link2, std::string() };
  if (p.link1_ > p.link2_)
    std::swap(p.link1_, p.link2_);

  auto enabled = find(srdf_config_->getEnabledCollisions(), p.link1_, p.link2_);
  auto disabled = find(srdf_config_->getDisabledCollisions(), p.link1_, p.link2_);
  bool changed = true;
  if (disabledByDefault(p.link1_, p.link2_))
  {
    assert(disabled == srdf_config_->getDisabledCollisions().end());
    auto& pairs = srdf_config_->getEnabledCollisions();
    if (disable)
    {
      if (enabled != pairs.end())  // delete all matching pairs, starting with enabled
        pairs.erase(std::remove_if(enabled, pairs.end(), PairMatcher(p.link1_, p.link2_)), pairs.end());
      else
        changed = false;
    }
    else
    {
      p.reason_ = disabledReasonToString(NOT_DISABLED);
      if (enabled == pairs.end())
        srdf_config_->getEnabledCollisions().push_back(p);
      else
        changed = false;
    }
  }
  else
  {
    assert(enabled == srdf_config_->getEnabledCollisions().end());
    auto& pairs = srdf_config_->getDisabledCollisions();
    if (disable)
    {
      p.reason_ = disabledReasonToString(USER);
      if (disabled == pairs.end())
        pairs.push_back(p);
      else
        changed = false;
    }
    else
    {
      if (disabled != pairs.end())  // delete all matching pairs, starting with disabled
        pairs.erase(std::remove_if(disabled, pairs.end(), PairMatcher(p.link1_, p.link2_)), pairs.end());
      else
        changed = false;
    }
  }

  return changed;
}

}  // namespace srdf_setup
}  // namespace moveit_setup
