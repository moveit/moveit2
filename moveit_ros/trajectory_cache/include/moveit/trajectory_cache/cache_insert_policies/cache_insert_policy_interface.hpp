// Copyright 2024 Intrinsic Innovation LLC.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/** @file
 * @brief Abstract template class for injecting logic for determining when to prune and insert a
 * cache entry, and what metadata to attach to the cache entry.
 *
 * @author methylDragon
 */

#pragma once

#include <warehouse_ros/message_collection.h>
#include <moveit/move_group_interface/move_group_interface.hpp>

namespace moveit_ros
{
namespace trajectory_cache
{

/** @class CacheInsertPolicyInterface<KeyT, ValueT, CacheEntryT>
 * @headerfile cache_insert_policy_interface.hpp
 * "moveit/trajectory_cache/cache_insert_policies/cache_insert_policy_interface.hpp"
 *
 * @brief Abstract class for injecting logic for determining when to prune and insert a cache entry,
 * and what metadata to attach to the cache entry.
 *
 * @tparam KeyT. The object to extract features from which to key the cache. Typically the plan request.
 * @tparam ValueT. The object that the TrajectoryCache was passed to insert. Typically the plan response.
 * @tparam CacheEntryT. The object stored in the cache entry.
 *
 * Notably, implementations of this interface are used to determine all the logic leading up to, but
 * not including, the insert call itself.
 *
 * Additionally, the choice of which implementation to use will constraint the set of
 * FeaturesInterface<FeatureSourceT> implementations that can be used to fetch cache entries with,
 * with a caveat mentioned later.
 *
 * Users may implement this interface to add additional cache insertion functionality beyond the
 * ones provided by this package (e.g., prioritizing minimum jerk, or minimal path length), and the
 * set of metadata considered when inserting the cache entry.
 *
 * @see TrajectoryCache
 * @see FeaturesInterface<FeatureSourceT>
 *
 * Usage
 * ^^^^^
 * Each policy makes certain decisions about what pieces of metadata to attach to the cache
 * entry for later fetching.
 *
 * As such, an implementation will necessarily constrain the set of FeaturesInterface<FeatureSourceT>
 * implementations that can be used to fetch entries from the cache that were tagged by a policy.
 *
 * There is no way to meaningfully express this coupling at compile time, so users must ensure that
 * they read the documentation of each implementation of the interface to determine what
 * FeaturesInterface<FeatureSourceT> implementations can be used with cache entries processed by the
 * implementation.
 *
 * Two mitigations may be applied to alleviate this:
 *   1. Implementations of this interface may expose helper methods that can be called at runtime
 *      that take in arbitrary configuration parameters to them initialize the features that are
 *      used to fetch cache entries at point of use.
 *
 *   2. The TrajectoryCache class' insertion methods allow the bubbling down of additional
 *      user-specified features to be appended by the TrajectoryCache independent of a policy, which
 *      can be used to support features that are not explicitly supported by the policy.
 *
 *      Care must be taken to ensure that there are no overlaps when doing so, though, because it
 *      could potentially result in duplicate metadata entries otherwise.
 *
 * Consequently, a set of FeaturesInterface<FeatureSourceT> implementations can be used to fetch a
 * cache entry if that set is a subset of the union of:
 *   - The `additional_features` passed to `TrajectoryCache`
 *   - The features used by a policy to append metadata to the cache entry
 *
 * Pruning and Insertion
 * ^^^^^^^^^^^^^^^^^^^^^
 * Pruning is a two step process:
 *   1. Fetch all "matching" cache entries, as defined by the policy
 *   2. From the fetched "matching" entries, subject each to the `shouldPruneMatchingEntry` method,
 *      again defined by the policy.
 *
 * This allows a user to define a policy to match and prune on any arbitrary properties of
 * the cache entries and insertion candidate.
 *
 * Similarly, logic can be injected to determine when the insertion candidate should be inserted.
 *
 * NOTE: The TrajectoryCache class also has some top-level logic to preserve cache entries that
 * would have been pruned.
 *
 * Stateful Policies
 * ^^^^^^^^^^^^^^^^^
 * Finally, as there can be information that must be preserved across the multiple steps of the
 * match-prune-insert process, this interface assumes stateful operation within a single insert call.
 *
 * For example, preserving information in state will be required to propagate aggregated or rollup
 * information about the entire set of matched cache entries to future calls.
 *
 * The TrajectoryCache will call `reset()` on the policy at the end of the insert call.
 *
 * Call Flow
 * ^^^^^^^^^
 * The TrajectoryCache will call the following interface methods in the following order:
 *   1. sanitize, once
 *   2. fetchMatchingEntries, once
 *   3. shouldPruneMatchingEntry, once per fetched entry from fetchMatchingEntries
 *   4. shouldInsert, once
 *   5. appendInsertMetadata, once, if shouldInsert returns true
 *   6. reset, once, at the end.
 */
template <typename KeyT, typename ValueT, typename CacheEntryT>
class CacheInsertPolicyInterface
{
public:
  virtual ~CacheInsertPolicyInterface() = default;

  /** @brief Gets the name of the cache insert policy. */
  virtual std::string getName() const = 0;

  /** @brief Checks inputs to the cache insert call to see if we should abort instead.

   * @param[in] move_group. The manipulator move group, used to get its state.
   * @param[in] coll. The cache database to fetch messages from.
   * @param[in] key. The object used to key the insertion candidate with.
   * @param[in] value. The object that the TrajectoryCache was passed to insert.
   * @returns moveit::core::MoveItErrorCode::SUCCESS if the cache insert should proceed. Otherwise,
   * will return a different error code with the reason why it should not.
   */
  virtual moveit::core::MoveItErrorCode
  checkCacheInsertInputs(const moveit::planning_interface::MoveGroupInterface& move_group,
                         const warehouse_ros::MessageCollection<CacheEntryT>& coll, const KeyT& key,
                         const ValueT& value) = 0;

  /** @brief Fetches all "matching" cache entries for comparison for pruning.
   *
   * This method should be assumed to only return the message metadata without the underlying
   * message data.
   *
   * The policy should also make the decision about how to sort them.
   * The order in which cache entries are presented to the shouldPruneMatchingEntry will be the order of cache
   * entries returned by this function.
   *
   * @see CacheInsertPolicyInterface<KeyT, ValueT, CacheEntryT>#shouldPruneMatchingEntry
   *
   * @param[in] move_group. The manipulator move group, used to get its state.
   * @param[in] coll. The cache database to fetch messages from.
   * @param[in] key. The object used to key the insertion candidate with.
   * @param[in] value. The object that the TrajectoryCache was passed to insert.
   * @param[in] exact_match_precision. Tolerance for float precision comparison for what counts as an exact match.
   * @returns A vector of only the metadata of matching cache entries for use by the other methods.
   */
  virtual std::vector<typename warehouse_ros::MessageWithMetadata<CacheEntryT>::ConstPtr>
  fetchMatchingEntries(const moveit::planning_interface::MoveGroupInterface& move_group,
                       const warehouse_ros::MessageCollection<CacheEntryT>& coll, const KeyT& key, const ValueT& value,
                       double exact_match_precision) = 0;

  /** @brief Returns whether a matched cache entry should be pruned.
   *
   * NOTE: The TrajectoryCache class also has some top-level logic to preserve cache entries that
   * would have been pruned.
   *
   * @param[in] move_group. The manipulator move group, used to get its state.
   * @param[in] key. The object used to key the insertion candidate with.
   * @param[in] value. The object that the TrajectoryCache was passed to insert.
   * @param[in] matching_entry. The matched cache entry to be possibly pruned.
   * @param[out] reason. The reason for the returned result.
   * @returns True if the cache entry should be pruned.
   */
  virtual bool
  shouldPruneMatchingEntry(const moveit::planning_interface::MoveGroupInterface& move_group, const KeyT& key,
                           const ValueT& value,
                           const typename warehouse_ros::MessageWithMetadata<CacheEntryT>::ConstPtr& matching_entry,
                           std::string* reason) = 0;

  /** @brief Returns whether the insertion candidate should be inserted into the cache.
   *
   * NOTE: The TrajectoryCache class executes the insert, but this class informs it on whether the
   * insert should happen or not.
   *
   * @param[in] move_group. The manipulator move group, used to get its state.
   * @param[in] key. The object used to key the insertion candidate with.
   * @param[in] value. The object that the TrajectoryCache was passed to insert.
   * @param[out] reason. The reason for the returned result.
   * @returns True if the insertion candidate should be inserted into the cache.
   */
  virtual bool shouldInsert(const moveit::planning_interface::MoveGroupInterface& move_group, const KeyT& key,
                            const ValueT& value, std::string* reason) = 0;

  /** @brief Appends the insert metadata with the features supported by the policy.
   *
   * See notes in docstrings regarding the feature contract.
   *
   * @param[in,out] metadata. The metadata to add features to.
   * @param[in] move_group. The manipulator move group, used to get its state.
   * @param[in] key. The object used to key the insertion candidate with.
   * @param[in] value. The object that the TrajectoryCache was passed to insert.
   * @returns moveit::core::MoveItErrorCode::SUCCESS if successfully appended. Otherwise, will
   * return a different error code, in which case the metadata should not be reused, and the
   * TrajectoryCache will abort the insert.
   */
  virtual moveit::core::MoveItErrorCode
  appendInsertMetadata(warehouse_ros::Metadata& metadata,
                       const moveit::planning_interface::MoveGroupInterface& move_group, const KeyT& key,
                       const ValueT& value) = 0;

  /** @brief Resets the state of the policy. */
  virtual void reset() = 0;
};

}  // namespace trajectory_cache
}  // namespace moveit_ros
