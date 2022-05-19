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
#pragma once

#include <moveit_setup_srdf_plugins/srdf_step.hpp>
#include <moveit_setup_srdf_plugins/compute_default_collisions.hpp>
#include <boost/thread/thread.hpp>

// less operation for two CollisionPairs
struct CollisionPairLess
{
  bool operator()(const srdf::Model::CollisionPair& left, const srdf::Model::CollisionPair& right) const
  {
    // use std::pair's operator<: (left.link1_, left.link2_) < (right.link1_, right.link2_)
    return left.link1_ < right.link1_ || (!(right.link1_ < left.link1_) && left.link2_ < right.link2_);
  }
};

namespace moveit_setup
{
namespace srdf_setup
{
class DefaultCollisions : public SRDFStep
{
public:
  std::string getName() const override
  {
    return "Self-Collisions";
  }

  std::vector<std::string> getCollidingLinks();

  /**
   * @brief Output Link Pairs to SRDF Format
   */
  void linkPairsToSRDF();

  /**
   * @brief Output Link Pairs to SRDF Format; sorted; with optional filter
   * @param skip_mask mask of shifted DisabledReason values that will be skipped
   */
  void linkPairsToSRDFSorted(size_t skip_mask = 0);

  /**
   * @brief Load Link Pairs from SRDF Format
   */
  void linkPairsFromSRDF();

  LinkPairMap& getLinkPairs()
  {
    return link_pairs_;
  }

  // For Threaded Operations
  void startGenerationThread(unsigned int num_trials, double min_frac, bool verbose = true);
  void cancelGenerationThread();
  void joinGenerationThread();
  int getThreadProgress() const;

protected:
  void generateCollisionTable(unsigned int num_trials, double min_frac, bool verbose);

  /// main storage of link pair data
  LinkPairMap link_pairs_;

  // For threaded operations
  boost::thread worker_;
  unsigned int progress_;
};
}  // namespace srdf_setup
}  // namespace moveit_setup
