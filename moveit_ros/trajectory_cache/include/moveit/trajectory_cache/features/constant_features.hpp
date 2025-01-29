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

/** @file constant_features.hpp
 * @brief User-specified constant features to key the trajectory cache on.
 *
 * This allows a user to specify custom information to populate a fetch query or insert metadata that might not have
 * been obtained via extracting from a FeatureSourceT.
 *
 * @see FeaturesInterface<FeatureSourceT>
 *
 * @author methylDragon
 */

#pragma once

#include <utility>

#include <moveit/trajectory_cache/features/features_interface.hpp>

namespace moveit_ros
{
namespace trajectory_cache
{

// Queries. ========================================================================================

/** @class QueryOnlyEqFeature<AppendT, FeatureSourceT>
 * @brief Appends an equals query, with no metadata.
 */
template <typename AppendT, typename FeatureSourceT>
class QueryOnlyEqFeature final : public FeaturesInterface<FeatureSourceT>
{
public:
  QueryOnlyEqFeature(std::string name, AppendT value) : name_(std::move(name)), value_(value)
  {
  }

  std::string getName() const override
  {
    return "QueryOnlyEqFeature." + name_;
  }

  moveit::core::MoveItErrorCode
  appendFeaturesAsFuzzyFetchQuery(warehouse_ros::Query& query, const FeatureSourceT& source,
                                  const moveit::planning_interface::MoveGroupInterface& move_group,
                                  double exact_match_precision) const override
  {
    return appendFeaturesAsExactFetchQuery(query, source, move_group, exact_match_precision);
  }

  moveit::core::MoveItErrorCode
  appendFeaturesAsExactFetchQuery(warehouse_ros::Query& query, const FeatureSourceT& /*source*/,
                                  const moveit::planning_interface::MoveGroupInterface& /*move_group*/,
                                  double /*exact_match_precision*/) const override
  {
    query.append(name_, value_);
    return moveit::core::MoveItErrorCode::SUCCESS;
  }

  moveit::core::MoveItErrorCode
  appendFeaturesAsInsertMetadata(warehouse_ros::Metadata& /*metadata*/, const FeatureSourceT& /*source*/,
                                 const moveit::planning_interface::MoveGroupInterface& /*move_group*/) const override
  {
    return moveit::core::MoveItErrorCode::SUCCESS;  // No-op.
  }

private:
  std::string name_;
  AppendT value_;
};

/** @class QueryOnlyLTEFeature<AppendT, FeatureSourceT>
 * @brief Appends a less-than or equal-to query, with no metadata.
 */
template <typename AppendT, typename FeatureSourceT>
class QueryOnlyLTEFeature final : public FeaturesInterface<FeatureSourceT>
{
public:
  QueryOnlyLTEFeature(std::string name, AppendT value) : name_(std::move(name)), value_(value)
  {
  }

  std::string getName() const override
  {
    return "QueryOnlyLTEFeature." + name_;
  }

  moveit::core::MoveItErrorCode
  appendFeaturesAsFuzzyFetchQuery(warehouse_ros::Query& query, const FeatureSourceT& source,
                                  const moveit::planning_interface::MoveGroupInterface& move_group,
                                  double exact_match_precision) const override
  {
    return appendFeaturesAsExactFetchQuery(query, source, move_group, exact_match_precision);
  }

  moveit::core::MoveItErrorCode
  appendFeaturesAsExactFetchQuery(warehouse_ros::Query& query, const FeatureSourceT& /*source*/,
                                  const moveit::planning_interface::MoveGroupInterface& /*move_group*/,
                                  double /*exact_match_precision*/) const override
  {
    query.appendLTE(name_, value_);
    return moveit::core::MoveItErrorCode::SUCCESS;
  }

  moveit::core::MoveItErrorCode
  appendFeaturesAsInsertMetadata(warehouse_ros::Metadata& /*metadata*/, const FeatureSourceT& /*source*/,
                                 const moveit::planning_interface::MoveGroupInterface& /*move_group*/) const override
  {
    return moveit::core::MoveItErrorCode::SUCCESS;  // No-op.
  }

private:
  std::string name_;
  AppendT value_;
};

/** @class QueryOnlyGTEFeature<AppendT, FeatureSourceT>
 * @brief Appends a less-than or equal-to query, with no metadata.
 */
template <typename AppendT, typename FeatureSourceT>
class QueryOnlyGTEFeature final : public FeaturesInterface<FeatureSourceT>
{
public:
  QueryOnlyGTEFeature(std::string name, AppendT value) : name_(std::move(name)), value_(value)
  {
  }

  std::string getName() const override
  {
    return "QueryOnlyGTEFeature." + name_;
  }

  moveit::core::MoveItErrorCode
  appendFeaturesAsFuzzyFetchQuery(warehouse_ros::Query& query, const FeatureSourceT& source,
                                  const moveit::planning_interface::MoveGroupInterface& move_group,
                                  double exact_match_precision) const override
  {
    return appendFeaturesAsExactFetchQuery(query, source, move_group, exact_match_precision);
  }

  moveit::core::MoveItErrorCode
  appendFeaturesAsExactFetchQuery(warehouse_ros::Query& query, const FeatureSourceT& /*source*/,
                                  const moveit::planning_interface::MoveGroupInterface& /*move_group*/,
                                  double /*exact_match_precision*/) const override
  {
    query.appendGTE(name_, value_);
    return moveit::core::MoveItErrorCode::SUCCESS;
  }

  moveit::core::MoveItErrorCode
  appendFeaturesAsInsertMetadata(warehouse_ros::Metadata& /*metadata*/, const FeatureSourceT& /*source*/,
                                 const moveit::planning_interface::MoveGroupInterface& /*move_group*/) const override
  {
    return moveit::core::MoveItErrorCode::SUCCESS;  // No-op.
  }

private:
  std::string name_;
  AppendT value_;
};

/** @class QueryOnlyRangeInclusiveWithToleranceFeature<AppendT, FeatureSourceT>
 * @brief Appends a less-than or equal-to query, with no metadata.
 */
template <typename AppendT, typename FeatureSourceT>
class QueryOnlyRangeInclusiveWithToleranceFeature final : public FeaturesInterface<FeatureSourceT>
{
public:
  QueryOnlyRangeInclusiveWithToleranceFeature(std::string name, AppendT lower, AppendT upper)
    : name_(std::move(name)), lower_(lower), upper_(upper)
  {
  }

  std::string getName() const override
  {
    return "QueryOnlyRangeInclusiveWithToleranceFeature." + name_;
  }

  moveit::core::MoveItErrorCode
  appendFeaturesAsFuzzyFetchQuery(warehouse_ros::Query& query, const FeatureSourceT& source,
                                  const moveit::planning_interface::MoveGroupInterface& move_group,
                                  double exact_match_precision) const override
  {
    return appendFeaturesAsExactFetchQuery(query, source, move_group, exact_match_precision);
  }

  moveit::core::MoveItErrorCode
  appendFeaturesAsExactFetchQuery(warehouse_ros::Query& query, const FeatureSourceT& /*source*/,
                                  const moveit::planning_interface::MoveGroupInterface& /*move_group*/,
                                  double /*exact_match_precision*/) const override
  {
    query.appendRangeInclusive(name_, lower_, upper_);
    return moveit::core::MoveItErrorCode::SUCCESS;
  }

  moveit::core::MoveItErrorCode
  appendFeaturesAsInsertMetadata(warehouse_ros::Metadata& /*metadata*/, const FeatureSourceT& /*source*/,
                                 const moveit::planning_interface::MoveGroupInterface& /*move_group*/) const override
  {
    return moveit::core::MoveItErrorCode::SUCCESS;  // No-op.
  }

private:
  std::string name_;
  AppendT lower_;
  AppendT upper_;
};

// Metadata. =======================================================================================

/** @class MetadataOnlyFeature<AppendT, FeatureSourceT>
 * @brief Appends a single metadata value, with no query.
 */
template <typename AppendT, typename FeatureSourceT>
class MetadataOnlyFeature final : public FeaturesInterface<FeatureSourceT>
{
public:
  MetadataOnlyFeature(std::string name, AppendT value) : name_(std::move(name)), value_(value)
  {
  }

  std::string getName() const override
  {
    return "MetadataOnlyFeature." + name_;
  }

  moveit::core::MoveItErrorCode
  appendFeaturesAsFuzzyFetchQuery(warehouse_ros::Query& /*query*/, const FeatureSourceT& /*source*/,
                                  const moveit::planning_interface::MoveGroupInterface& /*move_group*/,
                                  double /*exact_match_precision*/) const override
  {
    return moveit::core::MoveItErrorCode::SUCCESS;  // No-op.
  }

  moveit::core::MoveItErrorCode
  appendFeaturesAsExactFetchQuery(warehouse_ros::Query& /*query*/, const FeatureSourceT& /*source*/,
                                  const moveit::planning_interface::MoveGroupInterface& /*move_group*/,
                                  double /*exact_match_precision*/) const override
  {
    return moveit::core::MoveItErrorCode::SUCCESS;  // No-op.
  }

  moveit::core::MoveItErrorCode
  appendFeaturesAsInsertMetadata(warehouse_ros::Metadata& metadata, const FeatureSourceT& /*source*/,
                                 const moveit::planning_interface::MoveGroupInterface& /*move_group*/) const override
  {
    metadata.append(name_, value_);
    return moveit::core::MoveItErrorCode::SUCCESS;
  }

private:
  std::string name_;
  AppendT value_;
};

}  // namespace trajectory_cache
}  // namespace moveit_ros
