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

}  // namespace srdf_setup
}  // namespace moveit_setup
