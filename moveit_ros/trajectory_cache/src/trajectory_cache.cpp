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
 * @brief Implementation of the Fuzzy-Matching Trajectory Cache.
 * @author methylDragon
 */

#include <chrono>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>

#include <warehouse_ros/message_collection.h>
#include <warehouse_ros/database_connection.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/warehouse/moveit_message_storage.hpp>
#include <moveit/trajectory_cache/utils/utils.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/warehouse/moveit_message_storage.hpp>

// Cache insert policies.
#include <moveit/trajectory_cache/cache_insert_policies/best_seen_execution_time_policy.hpp>
#include <moveit/trajectory_cache/cache_insert_policies/cache_insert_policy_interface.hpp>

// Features.
#include <moveit/trajectory_cache/features/features_interface.hpp>
#include <moveit/trajectory_cache/features/constant_features.hpp>
#include <moveit/trajectory_cache/features/get_cartesian_path_request_features.hpp>
#include <moveit/trajectory_cache/features/motion_plan_request_features.hpp>

#include <moveit/trajectory_cache/trajectory_cache.hpp>

namespace moveit_ros
{
namespace trajectory_cache
{

using ::warehouse_ros::MessageCollection;
using ::warehouse_ros::MessageWithMetadata;
using ::warehouse_ros::Metadata;
using ::warehouse_ros::Query;

using ::moveit::core::MoveItErrorCode;
using ::moveit::planning_interface::MoveGroupInterface;

using ::moveit_msgs::msg::MotionPlanRequest;
using ::moveit_msgs::msg::RobotTrajectory;
using ::moveit_msgs::srv::GetCartesianPath;

using ::moveit_ros::trajectory_cache::BestSeenExecutionTimePolicy;
using ::moveit_ros::trajectory_cache::CacheInsertPolicyInterface;
using ::moveit_ros::trajectory_cache::CartesianBestSeenExecutionTimePolicy;

using ::moveit_ros::trajectory_cache::FeaturesInterface;

namespace
{

const std::string EXECUTION_TIME = "execution_time_s";
const std::string FRACTION = "fraction";
const std::string PLANNING_TIME = "planning_time_s";

}  // namespace

// =================================================================================================
// Default Behavior Helpers.
// =================================================================================================

std::vector<std::unique_ptr<FeaturesInterface<MotionPlanRequest>>>
TrajectoryCache::getDefaultFeatures(double start_tolerance, double goal_tolerance)
{
  return BestSeenExecutionTimePolicy::getSupportedFeatures(start_tolerance, goal_tolerance);
}

std::unique_ptr<CacheInsertPolicyInterface<MotionPlanRequest, MoveGroupInterface::Plan, RobotTrajectory>>
TrajectoryCache::getDefaultCacheInsertPolicy()
{
  return std::make_unique<BestSeenExecutionTimePolicy>();
}

std::vector<std::unique_ptr<FeaturesInterface<GetCartesianPath::Request>>>
TrajectoryCache::getDefaultCartesianFeatures(double start_tolerance, double goal_tolerance, double min_fraction)
{
  return CartesianBestSeenExecutionTimePolicy::getSupportedFeatures(start_tolerance, goal_tolerance, min_fraction);
}

std::unique_ptr<CacheInsertPolicyInterface<GetCartesianPath::Request, GetCartesianPath::Response, RobotTrajectory>>
TrajectoryCache::getDefaultCartesianCacheInsertPolicy()
{
  return std::make_unique<CartesianBestSeenExecutionTimePolicy>();
}

std::string TrajectoryCache::getDefaultSortFeature()
{
  return EXECUTION_TIME;
}

// =================================================================================================
// Cache Configuration.
// =================================================================================================

TrajectoryCache::TrajectoryCache(const rclcpp::Node::SharedPtr& node)
  : node_(node), logger_(moveit::getLogger("moveit.ros.trajectory_cache"))
{
}

bool TrajectoryCache::init(const TrajectoryCache::Options& options)
{
  RCLCPP_DEBUG(logger_, "Opening trajectory cache database at: %s (Port: %d, Precision: %f)", options.db_path.c_str(),
               options.db_port, options.exact_match_precision);

  // If the `warehouse_plugin` parameter isn't set, defaults to warehouse_ros'
  // default.
  db_ = moveit_warehouse::loadDatabase(node_);
  options_ = options;

  db_->setParams(options.db_path, options.db_port);
  return db_->connect();
}

// =================================================================================================
// Getters and Setters.
// =================================================================================================

unsigned TrajectoryCache::countTrajectories(const std::string& cache_namespace)
{
  MessageCollection<RobotTrajectory> coll =
      db_->openCollection<RobotTrajectory>("move_group_trajectory_cache", cache_namespace);
  return coll.count();
}

unsigned TrajectoryCache::countCartesianTrajectories(const std::string& cache_namespace)
{
  MessageCollection<RobotTrajectory> coll =
      db_->openCollection<RobotTrajectory>("move_group_cartesian_trajectory_cache", cache_namespace);
  return coll.count();
}

std::string TrajectoryCache::getDbPath() const
{
  return options_.db_path;
}

uint32_t TrajectoryCache::getDbPort() const
{
  return options_.db_port;
}

double TrajectoryCache::getExactMatchPrecision() const
{
  return options_.exact_match_precision;
}

void TrajectoryCache::setExactMatchPrecision(double exact_match_precision)
{
  options_.exact_match_precision = exact_match_precision;
}

size_t TrajectoryCache::getNumAdditionalTrajectoriesToPreserveWhenPruningWorse() const
{
  return options_.num_additional_trajectories_to_preserve_when_pruning_worse;
}

void TrajectoryCache::setNumAdditionalTrajectoriesToPreserveWhenPruningWorse(
    size_t num_additional_trajectories_to_preserve_when_pruning_worse)
{
  options_.num_additional_trajectories_to_preserve_when_pruning_worse =
      num_additional_trajectories_to_preserve_when_pruning_worse;
}

// =================================================================================================
// Motion Plan Trajectory Caching.
// =================================================================================================

std::vector<MessageWithMetadata<RobotTrajectory>::ConstPtr> TrajectoryCache::fetchAllMatchingTrajectories(
    const MoveGroupInterface& move_group, const std::string& cache_namespace, const MotionPlanRequest& plan_request,
    const std::vector<std::unique_ptr<FeaturesInterface<MotionPlanRequest>>>& features, const std::string& sort_by,
    bool ascending, bool metadata_only) const
{
  MessageCollection<RobotTrajectory> coll =
      db_->openCollection<RobotTrajectory>("move_group_trajectory_cache", cache_namespace);

  Query::Ptr query = coll.createQuery();
  for (const auto& feature : features)
  {
    if (MoveItErrorCode ret =
            feature->appendFeaturesAsFuzzyFetchQuery(*query, plan_request, move_group,
                                                     /*exact_match_precision=*/options_.exact_match_precision);
        !ret)
    {
      RCLCPP_ERROR_STREAM(logger_, "Could not construct trajectory query: " << ret.message);
      return {};
    }
  }
  return coll.queryList(query, metadata_only, sort_by, ascending);
}

MessageWithMetadata<RobotTrajectory>::ConstPtr TrajectoryCache::fetchBestMatchingTrajectory(
    const MoveGroupInterface& move_group, const std::string& cache_namespace, const MotionPlanRequest& plan_request,
    const std::vector<std::unique_ptr<FeaturesInterface<MotionPlanRequest>>>& features, const std::string& sort_by,
    bool ascending, bool metadata_only) const
{
  // Find all matching, with metadata only. We'll use the ID of the best trajectory to pull it.
  std::vector<MessageWithMetadata<RobotTrajectory>::ConstPtr> matching_trajectories =
      this->fetchAllMatchingTrajectories(move_group, cache_namespace, plan_request, features, sort_by, ascending,
                                         /*metadata_only=*/true);
  if (matching_trajectories.empty())
  {
    RCLCPP_DEBUG(logger_, "No matching trajectories found.");
    return nullptr;
  }

  MessageCollection<RobotTrajectory> coll =
      db_->openCollection<RobotTrajectory>("move_group_trajectory_cache", cache_namespace);

  // Best trajectory is at first index, since the lookup query was sorted.
  int best_trajectory_id = matching_trajectories.at(0)->lookupInt("id");
  Query::Ptr best_query = coll.createQuery();
  best_query->append("id", best_trajectory_id);

  return coll.findOne(best_query, metadata_only);
}

bool TrajectoryCache::insertTrajectory(
    const MoveGroupInterface& move_group, const std::string& cache_namespace, const MotionPlanRequest& plan_request,
    const MoveGroupInterface::Plan& plan,
    CacheInsertPolicyInterface<MotionPlanRequest, MoveGroupInterface::Plan, RobotTrajectory>& cache_insert_policy,
    bool prune_worse_trajectories,
    const std::vector<std::unique_ptr<FeaturesInterface<MotionPlanRequest>>>& additional_features)
{
  MessageCollection<RobotTrajectory> coll =
      db_->openCollection<RobotTrajectory>("move_group_trajectory_cache", cache_namespace);

  // Check pre-preconditions.
  if (MoveItErrorCode ret = cache_insert_policy.checkCacheInsertInputs(move_group, coll, plan_request, plan); !ret)
  {
    RCLCPP_ERROR_STREAM(logger_, "Skipping trajectory insert, invalid inputs: " << ret.message);
    cache_insert_policy.reset();
    return false;
  }

  std::vector<MessageWithMetadata<RobotTrajectory>::ConstPtr> matching_entries =
      cache_insert_policy.fetchMatchingEntries(move_group, coll, plan_request, plan, options_.exact_match_precision);

  // Prune.
  if (prune_worse_trajectories)
  {
    size_t preserved_count = 0;
    for (const auto& matching_entry : matching_entries)
    {
      std::string prune_reason;
      if (++preserved_count > options_.num_additional_trajectories_to_preserve_when_pruning_worse &&
          cache_insert_policy.shouldPruneMatchingEntry(move_group, plan_request, plan, matching_entry, &prune_reason))
      {
        int delete_id = matching_entry->lookupInt("id");
        RCLCPP_DEBUG_STREAM(logger_, "Pruning plan (id: `" << delete_id << "`): " << prune_reason);

        Query::Ptr delete_query = coll.createQuery();
        delete_query->append("id", delete_id);
        coll.removeMessages(delete_query);
      }
    }
  }

  // Insert.
  std::string insert_reason;
  if (cache_insert_policy.shouldInsert(move_group, plan_request, plan, &insert_reason))
  {
    Metadata::Ptr insert_metadata = coll.createMetadata();

    if (MoveItErrorCode ret = cache_insert_policy.appendInsertMetadata(*insert_metadata, move_group, plan_request, plan);
        !ret)
    {
      RCLCPP_ERROR_STREAM(logger_,
                          "Skipping trajectory insert: Could not construct insert metadata from cache_insert_policy: "
                              << cache_insert_policy.getName() << ": " << ret.message);
      cache_insert_policy.reset();
      return false;
    }

    for (const auto& additional_feature : additional_features)
    {
      if (MoveItErrorCode ret =
              additional_feature->appendFeaturesAsInsertMetadata(*insert_metadata, plan_request, move_group);
          !ret)
      {
        RCLCPP_ERROR_STREAM(logger_,
                            "Skipping trajectory insert: Could not construct insert metadata additional_feature: "
                                << additional_feature->getName() << ": " << ret.message);
        cache_insert_policy.reset();
        return false;
      }
    }

    RCLCPP_DEBUG_STREAM(logger_, "Inserting trajectory:" << insert_reason);
    coll.insert(plan.trajectory, insert_metadata);
    cache_insert_policy.reset();
    return true;
  }
  else
  {
    RCLCPP_DEBUG_STREAM(logger_, "Skipping trajectory insert:" << insert_reason);
    cache_insert_policy.reset();
    return false;
  }
}

// =================================================================================================
// Cartesian Trajectory Caching.
// =================================================================================================

std::vector<MessageWithMetadata<RobotTrajectory>::ConstPtr> TrajectoryCache::fetchAllMatchingCartesianTrajectories(
    const MoveGroupInterface& move_group, const std::string& cache_namespace,
    const GetCartesianPath::Request& plan_request,
    const std::vector<std::unique_ptr<FeaturesInterface<GetCartesianPath::Request>>>& features,
    const std::string& sort_by, bool ascending, bool metadata_only) const
{
  MessageCollection<RobotTrajectory> coll =
      db_->openCollection<RobotTrajectory>("move_group_cartesian_trajectory_cache", cache_namespace);

  Query::Ptr query = coll.createQuery();
  for (const auto& feature : features)
  {
    if (MoveItErrorCode ret =
            feature->appendFeaturesAsFuzzyFetchQuery(*query, plan_request, move_group,
                                                     /*exact_match_precision=*/options_.exact_match_precision);
        !ret)
    {
      RCLCPP_ERROR_STREAM(logger_, "Could not construct cartesian trajectory query: " << ret.message);
      return {};
    }
  }
  return coll.queryList(query, metadata_only, sort_by, ascending);
}

MessageWithMetadata<RobotTrajectory>::ConstPtr TrajectoryCache::fetchBestMatchingCartesianTrajectory(
    const MoveGroupInterface& move_group, const std::string& cache_namespace,
    const GetCartesianPath::Request& plan_request,
    const std::vector<std::unique_ptr<FeaturesInterface<GetCartesianPath::Request>>>& features,
    const std::string& sort_by, bool ascending, bool metadata_only) const
{
  // Find all matching, with metadata only. We'll use the ID of the best trajectory to pull it.
  std::vector<MessageWithMetadata<RobotTrajectory>::ConstPtr> matching_trajectories =
      this->fetchAllMatchingCartesianTrajectories(move_group, cache_namespace, plan_request, features, sort_by,
                                                  ascending, /*metadata_only=*/true);
  if (matching_trajectories.empty())
  {
    RCLCPP_DEBUG(logger_, "No matching cartesian trajectories found.");
    return nullptr;
  }

  MessageCollection<RobotTrajectory> coll =
      db_->openCollection<RobotTrajectory>("move_group_cartesian_trajectory_cache", cache_namespace);

  // Best trajectory is at first index, since the lookup query was sorted.
  int best_trajectory_id = matching_trajectories.at(0)->lookupInt("id");
  Query::Ptr best_query = coll.createQuery();
  best_query->append("id", best_trajectory_id);

  return coll.findOne(best_query, metadata_only);
}

bool TrajectoryCache::insertCartesianTrajectory(
    const MoveGroupInterface& move_group, const std::string& cache_namespace,
    const GetCartesianPath::Request& plan_request, const GetCartesianPath::Response& plan,
    CacheInsertPolicyInterface<GetCartesianPath::Request, GetCartesianPath::Response, RobotTrajectory>&
        cache_insert_policy,
    bool prune_worse_trajectories,
    const std::vector<std::unique_ptr<FeaturesInterface<GetCartesianPath::Request>>>& additional_features)
{
  MessageCollection<RobotTrajectory> coll =
      db_->openCollection<RobotTrajectory>("move_group_cartesian_trajectory_cache", cache_namespace);

  // Check pre-preconditions.
  if (MoveItErrorCode ret = cache_insert_policy.checkCacheInsertInputs(move_group, coll, plan_request, plan); !ret)
  {
    RCLCPP_ERROR_STREAM(logger_, "Skipping cartesian trajectory insert, invalid inputs: " << ret.message);
    cache_insert_policy.reset();
    return false;
  }

  std::vector<MessageWithMetadata<RobotTrajectory>::ConstPtr> matching_entries =
      cache_insert_policy.fetchMatchingEntries(move_group, coll, plan_request, plan, options_.exact_match_precision);

  // Prune.
  if (prune_worse_trajectories)
  {
    size_t preserved_count = 0;
    for (const auto& matching_entry : matching_entries)
    {
      std::string prune_reason;
      if (++preserved_count > options_.num_additional_trajectories_to_preserve_when_pruning_worse &&
          cache_insert_policy.shouldPruneMatchingEntry(move_group, plan_request, plan, matching_entry, &prune_reason))
      {
        int delete_id = matching_entry->lookupInt("id");
        RCLCPP_DEBUG_STREAM(logger_, "Pruning cartesian trajectory (id: `" << delete_id << "`): " << prune_reason);

        Query::Ptr delete_query = coll.createQuery();
        delete_query->append("id", delete_id);
        coll.removeMessages(delete_query);
      }
    }
  }

  // Insert.
  std::string insert_reason;
  if (cache_insert_policy.shouldInsert(move_group, plan_request, plan, &insert_reason))
  {
    Metadata::Ptr insert_metadata = coll.createMetadata();

    if (MoveItErrorCode ret = cache_insert_policy.appendInsertMetadata(*insert_metadata, move_group, plan_request, plan);
        !ret)
    {
      RCLCPP_ERROR_STREAM(logger_, "Skipping cartesian trajectory insert: Could not construct insert metadata from "
                                   "cache_insert_policy: "
                                       << cache_insert_policy.getName() << ": " << ret.message);
      cache_insert_policy.reset();
      return false;
    }

    for (const auto& additional_feature : additional_features)
    {
      if (MoveItErrorCode ret =
              additional_feature->appendFeaturesAsInsertMetadata(*insert_metadata, plan_request, move_group);
          !ret)
      {
        RCLCPP_ERROR_STREAM(
            logger_, "Skipping cartesian trajectory insert: Could not construct insert metadata additional_feature: "
                         << additional_feature->getName() << ": " << ret.message);
        cache_insert_policy.reset();
        return false;
      }
    }

    RCLCPP_DEBUG_STREAM(logger_, "Inserting cartesian trajectory:" << insert_reason);
    coll.insert(plan.solution, insert_metadata);
    cache_insert_policy.reset();
    return true;
  }
  else
  {
    RCLCPP_DEBUG_STREAM(logger_, "Skipping cartesian insert:" << insert_reason);
    cache_insert_policy.reset();
    return false;
  }
}

}  // namespace trajectory_cache
}  // namespace moveit_ros
