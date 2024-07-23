// Copyright 2022 Johnson & Johnson
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

#pragma once

#include <chrono>
#include <memory>
#include <optional>

#include <rclcpp/rclcpp.hpp>

#include <warehouse_ros/message_collection.h>
#include <warehouse_ros/database_connection.h>

// TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// ROS2 Messages
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// moveit modules
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/srv/get_cartesian_path.hpp>

namespace moveit_ros
{
namespace trajectory_cache
{

/**
 * Trajectory Cache manager for MoveIt.
 *
 * This manager facilitates cache management for MoveIt 2's `MoveGroupInterface`
 * by using `warehouse_ros` to manage a database of executed trajectories, keyed
 * by the start and goal conditions, and sorted by how long the trajectories
 * took to execute. This allows for the lookup and reuse of the best performing
 * trajectories found so far.
 *
 * WARNING: RFE:
 *   !!! This cache does NOT support collision detection!
 *   Trajectories will be put into and fetched from the cache IGNORING
 *   collision.
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
 *   !!! This cache does NOT support keying on joint velocities and efforts.
 *   The cache only keys on joint positions.
 *
 *   !!! This cache does NOT support multi-DOF joints.

 *   !!! This cache does NOT support certain constraints
 *   Including: path, constraint regions, everything related to collision.
 *
 *   This is because they are difficult (but not impossible) to implement key
 *   logic for.
 *
 * Relevant ROS Parameters:
 *   - `warehouse_plugin`: What database to use
 *   - `warehouse_host`: Where the database should be created
 *   - `warehouse_port`: The port used for the database
 *
 * This class supports trajectories planned from move_group MotionPlanRequests
 * as well as GetCartesianPath requests. That is, both normal motion plans and
 * cartesian plans are supported.
 *
 * Motion plan trajectories are stored in the `move_group_trajectory_cache`
 * database within the database file, with trajectories for each move group
 * stored in a collection named after the relevant move group's name.
 *
 * For example, the "my_move_group" move group will have its cache stored in
 * `move_group_trajectory_cache@my_move_group`
 *
 * Motion Plan Trajectories are keyed on:
 *   - Plan Start: robot joint state
 *   - Plan Goal (either of):
 *     - Final pose (wrt. `planning_frame` (usually `base_link`))
 *     - Final robot joint states
 *   - Plan Constraints (but not collision)
 *
 * Trajectories may be looked up with some tolerance at call time.
 *
 * Similarly, the cartesian trajectories are stored in the
 * `move_group_cartesian_trajectory_cache` database within the database file,
 * with trajectories for each move group stored in a collection named after the
 * relevant move group's name.
 *
 * Cartesian Trajectories are keyed on:
 *   - Plan Start: robot joint state
 *   - Plan Goal:
 *     - Pose waypoints
 */
class TrajectoryCache
{
public:
  // We explicitly need a Node::SharedPtr because warehouse_ros ONLY supports
  // it...
  //
  // TODO: methylDragon -
  // Use rclcpp::node_interfaces::NodeInterfaces<> once warehouse_ros does.
  explicit TrajectoryCache(const rclcpp::Node::SharedPtr& node);

  bool init(const std::string& db_path = ":memory:", uint32_t db_port = 0, double exact_match_precision = 1e-6);

  unsigned count_trajectories(const std::string& move_group_namespace);

  unsigned count_cartesian_trajectories(const std::string& move_group_namespace);

  // ===========================================================================
  // MOTION PLAN TRAJECTORY CACHING
  // ===========================================================================
  // TOP LEVEL OPS

  // Fetches all plans that fit within the requested tolerances for start and
  // goal conditions, returning them as a vector, sorted by some db column.
  std::vector<warehouse_ros::MessageWithMetadata<moveit_msgs::msg::RobotTrajectory>::ConstPtr>
  fetch_all_matching_trajectories(const moveit::planning_interface::MoveGroupInterface& move_group,
                                  const std::string& move_group_namespace,
                                  const moveit_msgs::msg::MotionPlanRequest& plan_request, double start_tolerance,
                                  double goal_tolerance, bool metadata_only = false,
                                  const std::string& sort_by = "execution_time_s");

  // Fetches the best trajectory that fits within the requested tolerances for start
  // and goal conditions, by some db column.
  warehouse_ros::MessageWithMetadata<moveit_msgs::msg::RobotTrajectory>::ConstPtr fetch_best_matching_trajectory(
      const moveit::planning_interface::MoveGroupInterface& move_group, const std::string& move_group_namespace,
      const moveit_msgs::msg::MotionPlanRequest& plan_request, double start_tolerance, double goal_tolerance,
      bool metadata_only = false, const std::string& sort_by = "execution_time_s");

  // Put a trajectory into the database if it is the best matching trajectory seen so far.
  //
  // Trajectories are matched based off their start and goal states.
  // And are considered "better" if they have a smaller planned execution time
  // than matching trajectories.
  //
  // Optionally deletes all worse trajectories by default to prune the cache.
  bool put_trajectory(const moveit::planning_interface::MoveGroupInterface& move_group,
                      const std::string& move_group_namespace, const moveit_msgs::msg::MotionPlanRequest& plan_request,
                      const moveit_msgs::msg::RobotTrajectory& trajectory, double execution_time_s,
                      double planning_time_s, bool delete_worse_trajectories = true);

  // QUERY CONSTRUCTION
  bool extract_and_append_trajectory_start_to_query(warehouse_ros::Query& query,
                                                    const moveit::planning_interface::MoveGroupInterface& move_group,
                                                    const moveit_msgs::msg::MotionPlanRequest& plan_request,
                                                    double match_tolerance);

  bool extract_and_append_trajectory_goal_to_query(warehouse_ros::Query& query,
                                                   const moveit::planning_interface::MoveGroupInterface& move_group,
                                                   const moveit_msgs::msg::MotionPlanRequest& plan_request,
                                                   double match_tolerance);

  // METADATA CONSTRUCTION
  bool extract_and_append_trajectory_start_to_metadata(warehouse_ros::Metadata& metadata,
                                                       const moveit::planning_interface::MoveGroupInterface& move_group,
                                                       const moveit_msgs::msg::MotionPlanRequest& plan_request);

  bool extract_and_append_trajectory_goal_to_metadata(warehouse_ros::Metadata& metadata,
                                                      const moveit::planning_interface::MoveGroupInterface& move_group,
                                                      const moveit_msgs::msg::MotionPlanRequest& plan_request);

  // ===========================================================================
  // CARTESIAN TRAJECTORY CACHING
  // ===========================================================================
  // TOP LEVEL OPS

  // This mimics the move group computeCartesianPath signature (without path
  // constraints).
  moveit_msgs::srv::GetCartesianPath::Request
  construct_get_cartesian_path_request(moveit::planning_interface::MoveGroupInterface& move_group,
                                       const std::vector<geometry_msgs::msg::Pose>& waypoints, double step,
                                       double jump_threshold, bool avoid_collisions = true);

  // Fetches all cartesian trajectories that fit within the requested tolerances
  // for start and goal conditions, returning them as a vector, sorted by some
  // db column.
  std::vector<warehouse_ros::MessageWithMetadata<moveit_msgs::msg::RobotTrajectory>::ConstPtr>
  fetch_all_matching_cartesian_trajectories(const moveit::planning_interface::MoveGroupInterface& move_group,
                                            const std::string& move_group_namespace,
                                            const moveit_msgs::srv::GetCartesianPath::Request& plan_request,
                                            double min_fraction, double start_tolerance, double goal_tolerance,
                                            bool metadata_only = false,
                                            const std::string& sort_by = "execution_time_s");

  // Fetches the best cartesian trajectory that fits within the requested tolerances
  // for start and goal conditions, by some db column.
  warehouse_ros::MessageWithMetadata<moveit_msgs::msg::RobotTrajectory>::ConstPtr
  fetch_best_matching_cartesian_trajectory(const moveit::planning_interface::MoveGroupInterface& move_group,
                                           const std::string& move_group_namespace,
                                           const moveit_msgs::srv::GetCartesianPath::Request& plan_request,
                                           double min_fraction, double start_tolerance, double goal_tolerance,
                                           bool metadata_only = false, const std::string& sort_by = "execution_time_s");

  // Put a cartesian trajectory into the database if it is the best matching
  // cartesian trajectory seen so far.
  //
  // Cartesian trajectories are matched based off their start and goal states.
  // And are considered "better" if they have a smaller planned execution time
  // than matching cartesian trajectories.
  //
  // Optionally deletes all worse cartesian trajectories by default to prune the
  // cache.
  bool put_cartesian_trajectory(const moveit::planning_interface::MoveGroupInterface& move_group,
                                const std::string& move_group_namespace,
                                const moveit_msgs::srv::GetCartesianPath::Request& plan_request,
                                const moveit_msgs::msg::RobotTrajectory& trajectory, double execution_time_s,
                                double planning_time_s, double fraction, bool delete_worse_trajectories = true);

  // QUERY CONSTRUCTION
  bool extract_and_append_cartesian_trajectory_start_to_query(
      warehouse_ros::Query& query, const moveit::planning_interface::MoveGroupInterface& move_group,
      const moveit_msgs::srv::GetCartesianPath::Request& plan_request, double match_tolerance);

  bool extract_and_append_cartesian_trajectory_goal_to_query(
      warehouse_ros::Query& query, const moveit::planning_interface::MoveGroupInterface& move_group,
      const moveit_msgs::srv::GetCartesianPath::Request& plan_request, double match_tolerance);

  // METADATA CONSTRUCTION
  bool extract_and_append_cartesian_trajectory_start_to_metadata(
      warehouse_ros::Metadata& metadata, const moveit::planning_interface::MoveGroupInterface& move_group,
      const moveit_msgs::srv::GetCartesianPath::Request& plan_request);

  bool extract_and_append_cartesian_trajectory_goal_to_metadata(
      warehouse_ros::Metadata& metadata, const moveit::planning_interface::MoveGroupInterface& move_group,
      const moveit_msgs::srv::GetCartesianPath::Request& plan_request);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  warehouse_ros::DatabaseConnection::Ptr db_;

  double exact_match_precision_ = 1e-6;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace trajectory_cache
}  // namespace moveit_ros