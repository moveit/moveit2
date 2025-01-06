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
 * @brief Fuzzy-Matching Trajectory Cache.
 * @author methylDragon
 */

#pragma once

#include <chrono>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <warehouse_ros/message_collection.h>
#include <warehouse_ros/database_connection.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// moveit modules
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>

#include <moveit/trajectory_cache/cache_insert_policies/cache_insert_policy_interface.hpp>
#include <moveit/trajectory_cache/features/features_interface.hpp>

namespace moveit_ros
{
namespace trajectory_cache
{

/** @class TrajectoryCache trajectory_cache.hpp moveit/trajectory_cache/trajectory_cache.hpp
 *
 * @brief Trajectory Cache manager for MoveIt.
 *
 * This manager facilitates cache management for MoveIt 2's `MoveGroupInterface`
 * by using `warehouse_ros` to manage a database of executed trajectories, keyed
 * with injectable feature extractors, and pruned and inserted by cache insert
 * policies. This allows for the lookup and reuse of the best performing
 * trajectories found so far in a user-specified manner.
 *
 * Trajectories may be looked up with some tolerance at call time.
 *
 * The following ROS Parameters MUST be set:
 *   - `warehouse_plugin`: What database to use
 *
 * This class supports trajectories planned from move_group MotionPlanRequests
 * as well as GetCartesianPath requests. That is, both normal motion plans and
 * cartesian plans are supported.
 *
 * A cache fetch is intended to be usable as a stand-in for the
 * MoveGroupInterface `plan` and `computeCartesianPath` methods.
 *
 * WARNING: RFE:
 *   !!! The default set of feature extractors and cache insert policies do
 *   NOT support collision detection!
 *
 *   Trajectories using them will be inserted into and fetched from the cache
 *   IGNORING collision.
 *
 *   If your planning scene is expected to change between cache lookups, do NOT
 *   use this cache, fetched trajectories are likely to result in collision
 *   then.
 *
 *   To handle collisions this class will need to hash the planning scene world
 *   msg (after zeroing out std_msgs/Header timestamps and sequences) and do an
 *   appropriate lookup, or do more complicated checks to see if the scene world
 *   is "close enough" or is a less obstructed version of the scene in the cache
 *   entry.
 *
 *   Alternatively, use your planning scene after fetching the cache entry to
 *   validate if the cached trajectory will result in collisions or not.
 *
 *   !!! They also do NOT support keying on joint velocities and efforts.
 *   The cache only keys on joint positions.
 *
 *   !!! They also do NOT support multi-DOF joints.

 *   !!! They also do NOT support certain constraints
 *   Including: constraint regions, everything related to collision.
 *
 *   This is because they are difficult (but not impossible) to implement key
 *   logic for.
 *
 * Thread-Safety
 * ^^^^^^^^^^^^^
 * This class is NOT thread safe. Synchronize use of it if you need it in
 * multi-threaded contexts.
 *
 * Injectable Feature Extraction and Cache Insert Policies
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 * The specific features of cache entries and cache insertion candidates is
 * determined by passing the TrajectoryCache's insert and fetch methods with the
 * appropriate FeaturesInterface<FeatureSourceT> implementations, which will
 * extract and append the appropriate features to the query.
 *
 * Similarly, a CacheInsertPolicyInterface<KeyT, ValueT, CacheEntryT>
 * implementation must be passed to the insert method to determine what pruning
 * and insertion logic to apply.
 *
 * Each cache insert policy implementation constrains what features can be used
 * to fetch cache entries inserted with them. See the related interface classes
 * for more information.
 *
 * @see FeaturesInterface<FeatureSourceT>
 * @see CacheInsertPolicyInterface<KeyT, ValueT, CacheEntryT>
 *
 * This class provides a few helper methods to have "default" behavior of:
 * - Fetching and caching on "start" and "goal" parameters
 * - Pruning on `execution_time_s`
 * - Sorting on `execution_time_s`
 *
 * @see TrajectoryCache::getDefaultFeatures
 * @see TrajectoryCache::getDefaultCacheInsertPolicy
 *
 * @see TrajectoryCache::getDefaultCartesianFeatures
 * @see TrajectoryCache::getDefaultCartesianCacheInsertPolicy
 *
 * @see TrajectoryCache::getDefaultSortFeature
 *
 * Cache Sections
 * ^^^^^^^^^^^^^^
 * Motion plan trajectories are stored in the `move_group_trajectory_cache`
 * database within the database file, with trajectories for each move group
 * stored in a collection named after the relevant move group's name.
 *
 * For example, the "my_move_group" move group will have its cache stored in
 * `move_group_trajectory_cache@my_move_group`.
 *
 * Similarly, the cartesian trajectories are stored in the
 * `move_group_cartesian_trajectory_cache` database within the database file,
 * with trajectories for each move group stored in a collection named after the
 * relevant move group's name.
 */
class TrajectoryCache
{
public:
  /**
   * @name Default cache behavior helpers.
   */
  /**@{*/

  /** @brief Gets the default features for MotionPlanRequest messages.
   * @see BestSeenExecutionTimePolicy::getDefaultFeatures
   */
  static std::vector<std::unique_ptr<FeaturesInterface<moveit_msgs::msg::MotionPlanRequest>>>
  getDefaultFeatures(double start_tolerance, double goal_tolerance);

  /** @brief Gets the default cache insert policy for MotionPlanRequest messages.
   * @see BestSeenExecutionTimePolicy
   */
  static std::unique_ptr<CacheInsertPolicyInterface<moveit_msgs::msg::MotionPlanRequest,
                                                    moveit::planning_interface::MoveGroupInterface::Plan,
                                                    moveit_msgs::msg::RobotTrajectory>>
  getDefaultCacheInsertPolicy();

  /** @brief Gets the default features for GetCartesianPath requests.
   * @see CartesianBestSeenExecutionTimePolicy::getDefaultFeatures
   */
  static std::vector<std::unique_ptr<FeaturesInterface<moveit_msgs::srv::GetCartesianPath::Request>>>
  getDefaultCartesianFeatures(double start_tolerance, double goal_tolerance, double min_fraction);

  /** @brief Gets the default cache insert policy for GetCartesianPath requests.
   * @see CartesianBestSeenExecutionTimePolicy
   */
  static std::unique_ptr<
      CacheInsertPolicyInterface<moveit_msgs::srv::GetCartesianPath::Request,
                                 moveit_msgs::srv::GetCartesianPath::Response, moveit_msgs::msg::RobotTrajectory>>
  getDefaultCartesianCacheInsertPolicy();

  /** @brief Gets the default sort feature. */
  static std::string getDefaultSortFeature();

  /**@}*/

  /**
   * @name Cache configuration.
   */
  /**@{*/

  /**
   * @brief Constructs a TrajectoryCache.
   *
   * @param[in] node. An rclcpp::Node::SharedPtr, which will be used to lookup warehouse_ros parameters and log.
   *
   * TODO: methylDragon -
   *   We explicitly need a Node::SharedPtr because warehouse_ros ONLY supports it...
   *   Use rclcpp::node_interfaces::NodeInterfaces<> once warehouse_ros does.
   */
  explicit TrajectoryCache(const rclcpp::Node::SharedPtr& node);

  /**
   * @brief Options struct for TrajectoryCache.
   *
   * @property db_path. The database path.
   * @property db_port. The database port.
   * @property exact_match_precision. Tolerance for float precision comparison for what counts as an exact match.
   *   An exact match is when:
   *     (candidate >= value - (exact_match_precision / 2)
   *      && candidate <= value + (exact_match_precision / 2))
   * @property num_additional_trajectories_to_preserve_when_pruning_worse. The number of additional cached trajectories
   * to preserve when `prune_worse_trajectories` is true. It is useful to keep more than one matching trajectory to
   * have alternative trajectories to handle obstacles.
   */
  struct Options
  {
    std::string db_path = ":memory:";
    uint32_t db_port = 0;

    double exact_match_precision = 1e-6;
    size_t num_additional_trajectories_to_preserve_when_pruning_worse = 1;
  };

  /**
   * @brief Initializes the TrajectoryCache.
   *
   * This sets up the database connection, and sets any configuration parameters.
   * You must call this before calling any other method of the trajectory cache.
   *
   * @param[in] options. An instance of TrajectoryCache::Options to initialize the cache with.
   *   @see TrajectoryCache::Options
   * @returns True if the database was successfully connected to.
   * */
  bool init(const Options& options);

  /**@}*/

  /**
   * @name Getters and setters.
   */
  /**@{*/

  /**
   * @brief Count the number of non-cartesian trajectories for a particular cache namespace.
   *
   * @param[in] cache_namespace. A namespace to separate cache entries by. The name of the robot is a good choice.
   * @returns The number of non-cartesian trajectories for the cache namespace.
   */
  unsigned countTrajectories(const std::string& cache_namespace);

  /**
   * @brief Count the number of cartesian trajectories for a particular cache namespace.
   *
   * @param[in] cache_namespace. A namespace to separate cache entries by. The name of the robot is a good choice.
   * @returns The number of cartesian trajectories for the cache namespace.
   */
  unsigned countCartesianTrajectories(const std::string& cache_namespace);

  /** @brief Gets the database path. */
  std::string getDbPath() const;

  /** @brief Gets the database port. */
  uint32_t getDbPort() const;

  /** @brief Gets the exact match precision. */
  double getExactMatchPrecision() const;

  /** @brief Sets the exact match precision. */
  void setExactMatchPrecision(double exact_match_precision);

  /** @brief Get the number of trajectories to preserve when pruning worse trajectories. */
  size_t getNumAdditionalTrajectoriesToPreserveWhenPruningWorse() const;

  /** @brief Set the number of additional trajectories to preserve when pruning worse trajectories. */
  void setNumAdditionalTrajectoriesToPreserveWhenPruningWorse(
      size_t num_additional_trajectories_to_preserve_when_pruning_worse);

  /**@}*/

  /**
   * @name Motion plan trajectory caching
   */
  /**@{*/

  /**
   * @brief Fetches all trajectories keyed on user-specified features, returning them as a vector,
   * sorted by some cache feature.
   *
   * @see FeaturesInterface<FeatureSourceT>
   *
   * @param[in] move_group. The manipulator move group, used to get its state.
   * @param[in] cache_namespace. A namespace to separate cache entries by. The name of the robot is a good choice.
   * @param[in] plan_request. The motion plan request to extract features from to key the cache with.
   * @param[in] features. The features to key the cache with.
   * @param[in] sort_by. The cache feature to sort by.
   * @param[in] ascending. If true, sorts in ascending order. If false, sorts in descending order.
   * @param[in] metadata_only. If true, returns only the cache entry metadata.
   * @returns A vector of cache hits, sorted by the `sort_by` parameter.
   */
  std::vector<warehouse_ros::MessageWithMetadata<moveit_msgs::msg::RobotTrajectory>::ConstPtr>
  fetchAllMatchingTrajectories(
      const moveit::planning_interface::MoveGroupInterface& move_group, const std::string& cache_namespace,
      const moveit_msgs::msg::MotionPlanRequest& plan_request,
      const std::vector<std::unique_ptr<FeaturesInterface<moveit_msgs::msg::MotionPlanRequest>>>& features,
      const std::string& sort_by, bool ascending = true, bool metadata_only = false) const;

  /**
   * @brief Fetches the best trajectory keyed on user-specified features, with respect to some cache feature.
   *
   * @see FeaturesInterface<FeatureSourceT>
   *
   * @param[in] move_group. The manipulator move group, used to get its state.
   * @param[in] cache_namespace. A namespace to separate cache entries by. The name of the robot is a good choice.
   * @param[in] plan_request. The motion plan request to extract features from to key the cache with.
   * @param[in] features. The features to key the cache with.
   * @param[in] metadata_only. If true, returns only the cache entry metadata.
   * @param[in] sort_by. The cache feature to sort by.
   * @param[in] ascending. If true, sorts in ascending order. If false, sorts in descending order.
   * @returns The best cache hit, with respect to the `sort_by` parameter.
   */
  warehouse_ros::MessageWithMetadata<moveit_msgs::msg::RobotTrajectory>::ConstPtr fetchBestMatchingTrajectory(
      const moveit::planning_interface::MoveGroupInterface& move_group, const std::string& cache_namespace,
      const moveit_msgs::msg::MotionPlanRequest& plan_request,
      const std::vector<std::unique_ptr<FeaturesInterface<moveit_msgs::msg::MotionPlanRequest>>>& features,
      const std::string& sort_by, bool ascending = true, bool metadata_only = false) const;

  /**
   * @brief Inserts a trajectory into the database, with user-specified insert policy.
   *
   * Optionally deletes all worse trajectories by default to prune the cache.
   *
   * @see FeaturesInterface<FeatureSourceT>
   * @see CacheInsertPolicyInterface<KeyT, ValueT, CacheEntryT>
   *
   * @param[in] move_group. The manipulator move group, used to get its state.
   * @param[in] cache_namespace. A namespace to separate cache entries by. The name of the robot is a good choice.
   * @param[in] plan_request. The motion plan request to extract features from to key the cache with.
   * @param[in] plan. The plan containing the trajectory to insert.
   * @param[in,out] cache_insert_policy. The cache insert policy to use. Will determine what features can be used to
   * fetch entries, and pruning and insertion logic. Will be reset at the end of the call.
   * @param[in] prune_worse_trajectories. If true, will prune the cache according to the `cache_insert_policy`'s pruning
   * logic.
   * @param[in] additional_features. Additional features to key the cache with. Must not intersect with the set of
   * features supported by the `cache_insert_policy`.
   * @returns True if the trajectory was inserted into the cache.
   */
  bool insertTrajectory(const moveit::planning_interface::MoveGroupInterface& move_group,
                        const std::string& cache_namespace, const moveit_msgs::msg::MotionPlanRequest& plan_request,
                        const moveit::planning_interface::MoveGroupInterface::Plan& plan,
                        CacheInsertPolicyInterface<moveit_msgs::msg::MotionPlanRequest,
                                                   moveit::planning_interface::MoveGroupInterface::Plan,
                                                   moveit_msgs::msg::RobotTrajectory>& cache_insert_policy,
                        bool prune_worse_trajectories = true,
                        const std::vector<std::unique_ptr<FeaturesInterface<moveit_msgs::msg::MotionPlanRequest>>>&
                            additional_features = {});

  /**@}*/

  /**
   * @name Cartesian trajectory caching
   */
  /**@{*/

  /**
   * @brief Fetches all cartesian trajectories keyed on user-specified features, returning them as a
   * vector, sorted by some cache feature.
   *
   * @see FeaturesInterface<FeatureSourceT>
   *
   * @param[in] move_group. The manipulator move group, used to get its state.
   * @param[in] cache_namespace. A namespace to separate cache entries by. The name of the robot is a good choice.
   * @param[in] plan_request. The cartesian plan request to extract features from to key the cache with.
   * @param[in] features. The features to key the cache with.
   * @param[in] sort_by. The cache feature to sort by, defaults to execution time.
   * @param[in] ascending. If true, sorts in ascending order. If false, sorts in descending order.
   * @param[in] metadata_only. If true, returns only the cache entry metadata.
   * @returns A vector of cache hits, sorted by the `sort_by` parameter.
   */
  std::vector<warehouse_ros::MessageWithMetadata<moveit_msgs::msg::RobotTrajectory>::ConstPtr>
  fetchAllMatchingCartesianTrajectories(
      const moveit::planning_interface::MoveGroupInterface& move_group, const std::string& cache_namespace,
      const moveit_msgs::srv::GetCartesianPath::Request& plan_request,
      const std::vector<std::unique_ptr<FeaturesInterface<moveit_msgs::srv::GetCartesianPath::Request>>>& features,
      const std::string& sort_by, bool ascending = true, bool metadata_only = false) const;

  /**
   * @brief Fetches the best cartesian trajectory keyed on user-specified features, with respect to some cache feature.
   *
   * @see FeaturesInterface<FeatureSourceT>
   *
   * @param[in] move_group. The manipulator move group, used to get its state.
   * @param[in] cache_namespace. A namespace to separate cache entries by. The name of the robot is a good choice.
   * @param[in] plan_request. The cartesian plan request to extract features from to key the cache with.
   * @param[in] features. The features to key the cache with.
   * @param[in] sort_by. The cache feature to sort by.
   * @param[in] ascending. If true, sorts in ascending order. If false, sorts in descending order.
   * @param[in] metadata_only. If true, returns only the cache entry metadata.
   * @returns The best cache hit, with respect to the `sort_by` parameter.
   */
  warehouse_ros::MessageWithMetadata<moveit_msgs::msg::RobotTrajectory>::ConstPtr fetchBestMatchingCartesianTrajectory(
      const moveit::planning_interface::MoveGroupInterface& move_group, const std::string& cache_namespace,
      const moveit_msgs::srv::GetCartesianPath::Request& plan_request,
      const std::vector<std::unique_ptr<FeaturesInterface<moveit_msgs::srv::GetCartesianPath::Request>>>& features,
      const std::string& sort_by, bool ascending = true, bool metadata_only = false) const;

  /**
   * @brief Inserts a cartesian trajectory into the database, with user-specified insert policy.
   *
   * Optionally deletes all worse cartesian trajectories by default to prune the cache.
   *
   * @see FeaturesInterface<FeatureSourceT>
   * @see CacheInsertPolicyInterface<KeyT, ValueT, CacheEntryT>
   *
   * @param[in] move_group. The manipulator move group, used to get its state.
   * @param[in] cache_namespace. A namespace to separate cache entries by. The name of the robot is a good choice.
   * @param[in] plan_request. The cartesian path plan request to extract features from to key the cache with.
   * @param[in] plan. The plan containing the trajectory to insert.
   * @param[in,out] cache_insert_policy. The cache insert policy to use. Will determine what features can be used to
   * fetch entries, and pruning and insertion logic. Will be reset at the end of the call.
   * @param[in] prune_worse_trajectories. If true, will prune the cache according to the `cache_insert_policy`'s pruning
   * logic.
   * @param[in] additional_features. Additional features to key the cache with, must not intersect with the set of
   * features supported by the `cache_insert_policy`.
   * @returns True if the trajectory was inserted into the cache.
   */
  bool insertCartesianTrajectory(
      const moveit::planning_interface::MoveGroupInterface& move_group, const std::string& cache_namespace,
      const moveit_msgs::srv::GetCartesianPath::Request& plan_request,
      const moveit_msgs::srv::GetCartesianPath::Response& plan,
      CacheInsertPolicyInterface<moveit_msgs::srv::GetCartesianPath::Request,
                                 moveit_msgs::srv::GetCartesianPath::Response, moveit_msgs::msg::RobotTrajectory>&
          cache_insert_policy,
      bool prune_worse_trajectories = true,
      const std::vector<std::unique_ptr<FeaturesInterface<moveit_msgs::srv::GetCartesianPath::Request>>>&
          additional_features = {});

  /**@}*/

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  warehouse_ros::DatabaseConnection::Ptr db_;

  Options options_;
};

}  // namespace trajectory_cache
}  // namespace moveit_ros
