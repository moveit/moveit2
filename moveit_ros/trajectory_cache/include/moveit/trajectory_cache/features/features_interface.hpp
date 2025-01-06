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
 * @brief Abstract template class for extracting features from some FeatureSourceT.
 * @author methylDragon
 */

#pragma once

#include <warehouse_ros/message_collection.h>
#include <moveit/move_group_interface/move_group_interface.hpp>

namespace moveit_ros
{
namespace trajectory_cache
{

/** @interface FeaturesInterface<FeatureSourceT>
 * @headerfile features_interface.hpp "moveit/trajectory_cache/features/features_interface.hpp"
 *
 * @brief Abstract class for extracting features from arbitrary type FeatureSourceT to append to
 * warehouse_ros::Query and warehouse_ros::Metadata for keying TrajectoryCache entries with.
 *
 * @tparam FeatureSourceT. The object to extract features from.
 *
 * The features that are extracted from the FeatureSourceT can be used to key the TrajectoryCache
 * cache entries in a fuzzy and exact manner.
 *
 * Users may implement this interface to add additional keying functionality to the cache beyond the
 * ones provided by this package.
 *
 * @see TrajectoryCache
 *
 * Usage
 * ^^^^^
 * In order for a cache entry to be fetched using an implementation of FeaturesInterface<FeatureSourceT>,
 * it must also have been put with the same implementation, as fetching a cache entry via some
 * features requires that the features were added to the cache entry's metadata.
 *
 * This typically means adding implementations of FeaturesInterface<FeatureSourceT> as arguments to
 * the TrajectoryCache class's insertion methods. Or by using an appropriate CacheInsertPolicyInterface<KeyT, ValueT,
 * CacheEntryT> that composes the set of FeaturesInterface<FeatureSourceT> instances you are concerned with.
 *
 * Be sure to check the appropriate CacheInsertPolicyInterface<KeyT, ValueT, CacheEntryT>
 * implementations' docstrings to see what FeaturesInterface<FeatureSourceT> they support, and make
 * sure to only use a subset of those, unless you explicitly add additional metadata by providing
 * the appropriate additional FeaturesInterface<FeatureSourceT> instances on cache insertion.
 *
 * @see CacheInsertPolicyInterface<KeyT, ValueT, CacheEntryT>
 *
 * Composite Keys
 * ^^^^^^^^^^^^^^
 * Multiple unique instances of FeaturesInterface<FeatureSourceT> can be used together to express
 * composite key expressions, allowing you to constrain your match on a larger set of metadata
 * (provided the metadata was added to the cache entry by a superset of the same set of
 * FeaturesInterface<FeatureSourceT> instances used to fetch).
 *
 * For example, the following can be used together to more completely key a cache entry:
 *   - A FeaturesInterface that appends workspace information
 *   - A FeaturesInterface that appends robot joint state
 *   - A FeaturesInterface that appends the final pose goal
 *
 * You may then fetch that cache entry with a query formed from a subset of the instances used to key it, e.g.:
 *   - A FeaturesInterface that appends workspace information
 *   - A FeaturesInterface that appends robot joint state
 *
 * WARNING:
 *   Care must be taken to ensure that there are no collisions between the names of the query or
 *   metadata being added amongst the different FeaturesInterface<FeatureSourceT> interfaces being
 *   used together.
 *
 *   A good way of preventing this is adding a prefix.
 *
 * User-Specified Keys
 * ^^^^^^^^^^^^^^^^^^^
 * You may also implement this interface to constrain a fetch query or tag cache entry metadata with
 * user-specified parameters that do not depend on FeatureSourceT or any other arguments.
 *
 * Simply ignore any passed arguments as necessary, and ensure that the correct information is
 * appended in the append implementations as appropriate.
 *
 * @see constant_features.hpp
 */
template <typename FeatureSourceT>
class FeaturesInterface
{
public:
  virtual ~FeaturesInterface() = default;

  /** @brief Gets the name of the features implementation. */
  virtual std::string getName() const = 0;

  /**
   * @brief Extracts relevant features from FeatureSourceT, to be appended to a fetch query, with fuzzy matching.
   *
   * These parameters will be used key the cache element in a fuzzy manner.
   *
   * @param[in,out] query. The query to add features to.
   * @param[in] source. A FeatureSourceT to extract features from.
   * @param[in] move_group. The manipulator move group, used to get its state.
   * @param[in] exact_match_precision. Tolerance for float precision comparison for what counts as an exact match.
   * @returns moveit::core::MoveItErrorCode::SUCCESS if successfully appended. Otherwise, will return a different error
   * code, in which case the query should not be reused.
   */
  virtual moveit::core::MoveItErrorCode
  appendFeaturesAsFuzzyFetchQuery(warehouse_ros::Query& query, const FeatureSourceT& source,
                                  const moveit::planning_interface::MoveGroupInterface& move_group,
                                  double exact_match_precision) const = 0;

  /**
   * @brief Extracts relevant features from FeatureSourceT, to be appended to a fetch query, with exact matching.
   *
   * These parameters will be used key the cache element in an exact manner.
   *
   * @param[in,out] query. The query to add features to.
   * @param[in] source. A FeatureSourceT to extract features from.
   * @param[in] move_group. The manipulator move group, used to get its state.
   * @param[in] exact_match_precision. Tolerance for float precision comparison for what counts as an exact match.
   * @returns moveit::core::MoveItErrorCode::SUCCESS if successfully appended. Otherwise, will return a different error
   * code, in which case the query should not be reused.
   */
  virtual moveit::core::MoveItErrorCode
  appendFeaturesAsExactFetchQuery(warehouse_ros::Query& query, const FeatureSourceT& source,
                                  const moveit::planning_interface::MoveGroupInterface& move_group,
                                  double exact_match_precision) const = 0;

  /**
   * @brief Extracts relevant features from FeatureSourceT, to be appended to a cache entry's metadata.
   *
   * These parameters will be used key the cache element.
   *
   * @param[in,out] metadata. The metadata to add features to.
   * @param[in] source. A FeatureSourceT to extract features from.
   * @param[in] move_group. The manipulator move group, used to get its state.
   * @returns moveit::core::MoveItErrorCode::SUCCESS if successfully appended. Otherwise, will return a different error
   * code, in which case the metadata should not be reused.
   */
  virtual moveit::core::MoveItErrorCode
  appendFeaturesAsInsertMetadata(warehouse_ros::Metadata& metadata, const FeatureSourceT& source,
                                 const moveit::planning_interface::MoveGroupInterface& move_group) const = 0;
};

}  // namespace trajectory_cache
}  // namespace moveit_ros
