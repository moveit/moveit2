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

#include <memory>
#include <vector>

#include "moveit/robot_state/conversions.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit/warehouse/moveit_message_storage.h"
#include "warehouse_ros/message_collection.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "moveit/trajectory_cache/trajectory_cache.hpp"

namespace moveit_ros
{
namespace trajectory_cache
{

using warehouse_ros::MessageWithMetadata;
using warehouse_ros::Metadata;
using warehouse_ros::Query;

// Utils =======================================================================

// Append a range inclusive query with some tolerance about some center value.
void query_append_range_inclusive_with_tolerance(Query& query, const std::string& name, double center, double tolerance)
{
  query.appendRangeInclusive(name, center - tolerance / 2, center + tolerance / 2);
}

// Sort constraint components by joint or link name.
void sort_constraints(std::vector<moveit_msgs::msg::JointConstraint>& joint_constraints,
                      std::vector<moveit_msgs::msg::PositionConstraint>& position_constraints,
                      std::vector<moveit_msgs::msg::OrientationConstraint>& orientation_constraints)
{
  std::sort(joint_constraints.begin(), joint_constraints.end(),
            [](const moveit_msgs::msg::JointConstraint& l, const moveit_msgs::msg::JointConstraint& r) {
              return l.joint_name < r.joint_name;
            });

  std::sort(position_constraints.begin(), position_constraints.end(),
            [](const moveit_msgs::msg::PositionConstraint& l, const moveit_msgs::msg::PositionConstraint& r) {
              return l.link_name < r.link_name;
            });

  std::sort(orientation_constraints.begin(), orientation_constraints.end(),
            [](const moveit_msgs::msg::OrientationConstraint& l, const moveit_msgs::msg::OrientationConstraint& r) {
              return l.link_name < r.link_name;
            });
}

// Trajectory Cache ============================================================

TrajectoryCache::TrajectoryCache(const rclcpp::Node::SharedPtr& node) : node_(node), logger_(moveit::getLogger("moveit.ros.trajectory_cache"))
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

bool TrajectoryCache::init(const std::string& db_path, uint32_t db_port, double exact_match_precision)
{
  RCLCPP_DEBUG(logger_, "Opening trajectory cache database at: %s (Port: %d, Precision: %f)",
              db_path.c_str(), db_port, exact_match_precision);

  // If the `warehouse_plugin` parameter isn't set, defaults to warehouse_ros'
  // default.
  db_ = moveit_warehouse::loadDatabase(node_);

  exact_match_precision_ = exact_match_precision;
  db_->setParams(db_path, db_port);
  return db_->connect();
}

unsigned TrajectoryCache::count_trajectories(const std::string& move_group_namespace)
{
  auto coll =
      db_->openCollection<moveit_msgs::msg::RobotTrajectory>("move_group_trajectory_cache", move_group_namespace);
  return coll.count();
}

unsigned TrajectoryCache::count_cartesian_trajectories(const std::string& move_group_namespace)
{
  auto coll = db_->openCollection<moveit_msgs::msg::RobotTrajectory>("move_group_cartesian_trajectory_cache",
                                                                     move_group_namespace);
  return coll.count();
}

// =============================================================================
// MOTION PLAN TRAJECTORY CACHING
// =============================================================================
// MOTION PLAN TRAJECTORY CACHING: TOP LEVEL OPS
std::vector<MessageWithMetadata<moveit_msgs::msg::RobotTrajectory>::ConstPtr>
TrajectoryCache::fetch_all_matching_trajectories(const moveit::planning_interface::MoveGroupInterface& move_group,
                                                 const std::string& move_group_namespace,
                                                 const moveit_msgs::msg::MotionPlanRequest& plan_request,
                                                 double start_tolerance, double goal_tolerance, bool metadata_only,
                                                 const std::string& sort_by)
{
  auto coll =
      db_->openCollection<moveit_msgs::msg::RobotTrajectory>("move_group_trajectory_cache", move_group_namespace);

  Query::Ptr query = coll.createQuery();

  bool start_ok = this->extract_and_append_trajectory_start_to_query(*query, move_group, plan_request, start_tolerance);
  bool goal_ok = this->extract_and_append_trajectory_goal_to_query(*query, move_group, plan_request, goal_tolerance);

  if (!start_ok || !goal_ok)
  {
    RCLCPP_ERROR(logger_, "Could not construct trajectory query.");
    return {};
  }

  return coll.queryList(query, metadata_only, sort_by, true);
}

MessageWithMetadata<moveit_msgs::msg::RobotTrajectory>::ConstPtr TrajectoryCache::fetch_best_matching_trajectory(
    const moveit::planning_interface::MoveGroupInterface& move_group, const std::string& move_group_namespace,
    const moveit_msgs::msg::MotionPlanRequest& plan_request, double start_tolerance, double goal_tolerance,
    bool metadata_only, const std::string& sort_by)
{
  // First find all matching, but metadata only.
  // Then use the ID metadata of the best plan to pull the actual message.
  auto matching_trajectories = this->fetch_all_matching_trajectories(move_group, move_group_namespace, plan_request,
                                                                     start_tolerance, goal_tolerance, true, sort_by);

  if (matching_trajectories.empty())
  {
    RCLCPP_DEBUG(logger_, "No matching trajectories found.");
    return nullptr;
  }

  auto coll =
      db_->openCollection<moveit_msgs::msg::RobotTrajectory>("move_group_trajectory_cache", move_group_namespace);

  // Best plan is at first index, since the lookup query was sorted by
  // execution_time.
  int best_trajectory_id = matching_trajectories.at(0)->lookupInt("id");
  Query::Ptr best_query = coll.createQuery();
  best_query->append("id", best_trajectory_id);

  return coll.findOne(best_query, metadata_only);
}

bool TrajectoryCache::put_trajectory(const moveit::planning_interface::MoveGroupInterface& move_group,
                                     const std::string& move_group_namespace,
                                     const moveit_msgs::msg::MotionPlanRequest& plan_request,
                                     const moveit_msgs::msg::RobotTrajectory& trajectory, double execution_time_s,
                                     double planning_time_s, bool delete_worse_trajectories)
{
  // Check pre-conditions
  if (!trajectory.multi_dof_joint_trajectory.points.empty())
  {
    RCLCPP_ERROR(logger_, "Skipping plan insert: Multi-DOF trajectory plans are not supported.");
    return false;
  }
  if (plan_request.workspace_parameters.header.frame_id.empty() || trajectory.joint_trajectory.header.frame_id.empty())
  {
    RCLCPP_ERROR(logger_, "Skipping plan insert: Frame IDs cannot be empty.");
    return false;
  }
  if (plan_request.workspace_parameters.header.frame_id != trajectory.joint_trajectory.header.frame_id)
  {
    RCLCPP_ERROR(logger_,
                 "Skipping plan insert: "
                 "Plan request frame (%s) does not match plan frame (%s).",
                 plan_request.workspace_parameters.header.frame_id.c_str(),
                 trajectory.joint_trajectory.header.frame_id.c_str());
    return false;
  }

  auto coll =
      db_->openCollection<moveit_msgs::msg::RobotTrajectory>("move_group_trajectory_cache", move_group_namespace);

  // Pull out trajectories "exactly" keyed by request in cache.
  Query::Ptr exact_query = coll.createQuery();

  bool start_query_ok = this->extract_and_append_trajectory_start_to_query(*exact_query, move_group, plan_request, 0);
  bool goal_query_ok = this->extract_and_append_trajectory_goal_to_query(*exact_query, move_group, plan_request, 0);

  if (!start_query_ok || !goal_query_ok)
  {
    RCLCPP_ERROR(logger_, "Skipping plan insert: Could not construct lookup query.");
    return false;
  }

  auto exact_matches = coll.queryList(exact_query, /* metadata_only */ true, /* sort_by */ "execution_time_s");

  double best_execution_time = std::numeric_limits<double>::infinity();
  if (!exact_matches.empty())
  {
    best_execution_time = exact_matches.at(0)->lookupDouble("execution_time_s");

    if (delete_worse_trajectories)
    {
      for (auto& match : exact_matches)
      {
        double match_execution_time_s = match->lookupDouble("execution_time_s");
        if (execution_time_s < match_execution_time_s)
        {
          int delete_id = match->lookupInt("id");
          RCLCPP_DEBUG(logger_,
                      "Overwriting plan (id: %d): "
                      "execution_time (%es) > new trajectory's execution_time (%es)",
                      delete_id, match_execution_time_s, execution_time_s);

          Query::Ptr delete_query = coll.createQuery();
          delete_query->append("id", delete_id);
          coll.removeMessages(delete_query);
        }
      }
    }
  }

  // Insert if candidate is best seen.
  if (execution_time_s < best_execution_time)
  {
    Metadata::Ptr insert_metadata = coll.createMetadata();

    bool start_meta_ok =
        this->extract_and_append_trajectory_start_to_metadata(*insert_metadata, move_group, plan_request);
    bool goal_meta_ok =
        this->extract_and_append_trajectory_goal_to_metadata(*insert_metadata, move_group, plan_request);
    insert_metadata->append("execution_time_s", execution_time_s);
    insert_metadata->append("planning_time_s", planning_time_s);

    if (!start_meta_ok || !goal_meta_ok)
    {
      RCLCPP_ERROR(logger_, "Skipping plan insert: Could not construct insert metadata.");
      return false;
    }

    RCLCPP_DEBUG(logger_,
                "Inserting trajectory: New trajectory execution_time (%es) "
                "is better than best trajectory's execution_time (%es)",
                execution_time_s, best_execution_time);

    coll.insert(trajectory, insert_metadata);
    return true;
  }

  RCLCPP_DEBUG(logger_,
              "Skipping plan insert: New trajectory execution_time (%es) "
              "is worse than best trajectory's execution_time (%es)",
              execution_time_s, best_execution_time);
  return false;
}

// MOTION PLAN TRAJECTORY CACHING: QUERY CONSTRUCTION
bool TrajectoryCache::extract_and_append_trajectory_start_to_query(
    Query& query, const moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit_msgs::msg::MotionPlanRequest& plan_request, double match_tolerance)
{
  match_tolerance += exact_match_precision_;

  // Make ignored members explicit
  if (!plan_request.start_state.multi_dof_joint_state.joint_names.empty())
  {
    RCLCPP_WARN(logger_, "Ignoring start_state.multi_dof_joint_states: Not supported.");
  }
  if (!plan_request.start_state.attached_collision_objects.empty())
  {
    RCLCPP_WARN(logger_, "Ignoring start_state.attached_collision_objects: Not supported.");
  }

  query.append("group_name", plan_request.group_name);

  // Workspace params
  // Match anything within our specified workspace limits.
  query.append("workspace_parameters.header.frame_id", plan_request.workspace_parameters.header.frame_id);
  query.appendGTE("workspace_parameters.min_corner.x", plan_request.workspace_parameters.min_corner.x);
  query.appendGTE("workspace_parameters.min_corner.y", plan_request.workspace_parameters.min_corner.y);
  query.appendGTE("workspace_parameters.min_corner.z", plan_request.workspace_parameters.min_corner.z);
  query.appendLTE("workspace_parameters.max_corner.x", plan_request.workspace_parameters.max_corner.x);
  query.appendLTE("workspace_parameters.max_corner.y", plan_request.workspace_parameters.max_corner.y);
  query.appendLTE("workspace_parameters.max_corner.z", plan_request.workspace_parameters.max_corner.z);

  // Joint state
  //   Only accounts for joint_state position. Ignores velocity and effort.
  if (plan_request.start_state.is_diff)
  {
    // If plan request states that the start_state is_diff, then we need to get
    // the current state from the move_group.

    // NOTE: methyldragon -
    //   I think if is_diff is on, the joint states will not be populated in all
    //   of our motion plan requests? If this isn't the case we might need to
    //   apply the joint states as offsets as well.
    //
    // TODO: Since MoveIt also potentially does another getCurrentState() call
    //   when planning, there is a chance that the current state in the cache
    //   differs from the state used in MoveIt's plan.
    //
    //   When upstreaming this class to MoveIt, this issue should go away once
    //   the class is used within the move group's Plan call.
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    if (!current_state)
    {
      RCLCPP_WARN(logger_, "Skipping start query append: Could not get robot state.");
      // NOTE: methyldragon -
      //   Ideally we would restore the original state here and undo our changes, however copy of the query is not supported.
      return false;
    }

    moveit_msgs::msg::RobotState current_state_msg;
    robotStateToRobotStateMsg(*current_state, current_state_msg);

    for (size_t i = 0; i < current_state_msg.joint_state.name.size(); i++)
    {
      query.append("start_state.joint_state.name_" + std::to_string(i), current_state_msg.joint_state.name.at(i));
      query_append_range_inclusive_with_tolerance(query, "start_state.joint_state.position_" + std::to_string(i),
                                                  current_state_msg.joint_state.position.at(i), match_tolerance);
    }
  }
  else
  {
    for (size_t i = 0; i < plan_request.start_state.joint_state.name.size(); i++)
    {
      query.append("start_state.joint_state.name_" + std::to_string(i), plan_request.start_state.joint_state.name.at(i));
      query_append_range_inclusive_with_tolerance(query, "start_state.joint_state.position_" + std::to_string(i),
                                                  plan_request.start_state.joint_state.position.at(i), match_tolerance);
    }
  }
  return true;
}

bool TrajectoryCache::extract_and_append_trajectory_goal_to_query(
    Query& query, const moveit::planning_interface::MoveGroupInterface& /* move_group */,
    const moveit_msgs::msg::MotionPlanRequest& plan_request, double match_tolerance)
{
  match_tolerance += exact_match_precision_;

  // Make ignored members explicit
  bool emit_position_constraint_warning = false;
  for (auto& constraint : plan_request.goal_constraints)
  {
    for (auto& position_constraint : constraint.position_constraints)
    {
      if (!position_constraint.constraint_region.primitives.empty())
      {
        emit_position_constraint_warning = true;
        break;
      }
    }
    if (emit_position_constraint_warning)
    {
      break;
    }
  }
  if (emit_position_constraint_warning)
  {
    RCLCPP_WARN(logger_, "Ignoring goal_constraints.position_constraints.constraint_region: "
                                     "Not supported.");
  }

  query_append_range_inclusive_with_tolerance(query, "max_velocity_scaling_factor",
                                              plan_request.max_velocity_scaling_factor, match_tolerance);
  query_append_range_inclusive_with_tolerance(query, "max_acceleration_scaling_factor",
                                              plan_request.max_acceleration_scaling_factor, match_tolerance);
  query_append_range_inclusive_with_tolerance(query, "max_cartesian_speed", plan_request.max_cartesian_speed,
                                              match_tolerance);

  // Extract constraints (so we don't have cardinality on goal_constraint idx.)
  std::vector<moveit_msgs::msg::JointConstraint> joint_constraints;
  std::vector<moveit_msgs::msg::PositionConstraint> position_constraints;
  std::vector<moveit_msgs::msg::OrientationConstraint> orientation_constraints;
  for (auto& constraint : plan_request.goal_constraints)
  {
    for (auto& joint_constraint : constraint.joint_constraints)
    {
      joint_constraints.push_back(joint_constraint);
    }
    for (auto& position_constraint : constraint.position_constraints)
    {
      position_constraints.push_back(position_constraint);
    }
    for (auto& orientation_constraint : constraint.orientation_constraints)
    {
      orientation_constraints.push_back(orientation_constraint);
    }

    // Also sort for even less cardinality.
    sort_constraints(joint_constraints, position_constraints, orientation_constraints);
  }

  // Joint constraints
  size_t joint_idx = 0;
  for (auto& constraint : joint_constraints)
  {
    std::string meta_name = "goal_constraints.joint_constraints_" + std::to_string(joint_idx++);

    query.append(meta_name + ".joint_name", constraint.joint_name);
    query_append_range_inclusive_with_tolerance(query, meta_name + ".position", constraint.position, match_tolerance);
    query.appendGTE(meta_name + ".tolerance_above", constraint.tolerance_above);
    query.appendLTE(meta_name + ".tolerance_below", constraint.tolerance_below);
  }

  // Position constraints
  // All offsets will be "frozen" and computed wrt. the workspace frame
  // instead.
  if (!position_constraints.empty())
  {
    query.append("goal_constraints.position_constraints.header.frame_id",
                 plan_request.workspace_parameters.header.frame_id);

    size_t pos_idx = 0;
    for (auto& constraint : position_constraints)
    {
      std::string meta_name = "goal_constraints.position_constraints_" + std::to_string(pos_idx++);

      // Compute offsets wrt. to workspace frame.
      double x_offset = 0;
      double y_offset = 0;
      double z_offset = 0;

      if (plan_request.workspace_parameters.header.frame_id != constraint.header.frame_id)
      {
        try
        {
          auto transform = tf_buffer_->lookupTransform(
              constraint.header.frame_id, plan_request.workspace_parameters.header.frame_id, tf2::TimePointZero);

          x_offset = transform.transform.translation.x;
          y_offset = transform.transform.translation.y;
          z_offset = transform.transform.translation.z;
        }
        catch (tf2::TransformException& ex)
        {
          RCLCPP_WARN(logger_,
                      "Skipping goal query append: "
                      "Could not get goal transform for translation %s to %s: %s",
                      plan_request.workspace_parameters.header.frame_id.c_str(), constraint.header.frame_id.c_str(),
                      ex.what());

          // NOTE: methyldragon -
          //   Ideally we would restore the original state here and undo our changes, however copy of the query is not supported.
          return false;
        }
      }

      query.append(meta_name + ".link_name", constraint.link_name);

      query_append_range_inclusive_with_tolerance(query, meta_name + ".target_point_offset.x",
                                                  x_offset + constraint.target_point_offset.x, match_tolerance);
      query_append_range_inclusive_with_tolerance(query, meta_name + ".target_point_offset.y",
                                                  y_offset + constraint.target_point_offset.y, match_tolerance);
      query_append_range_inclusive_with_tolerance(query, meta_name + ".target_point_offset.z",
                                                  z_offset + constraint.target_point_offset.z, match_tolerance);
    }
  }

  // Orientation constraints
  // All offsets will be "frozen" and computed wrt. the workspace frame
  // instead.
  if (!orientation_constraints.empty())
  {
    query.append("goal_constraints.orientation_constraints.header.frame_id",
                 plan_request.workspace_parameters.header.frame_id);

    size_t ori_idx = 0;
    for (auto& constraint : orientation_constraints)
    {
      std::string meta_name = "goal_constraints.orientation_constraints_" + std::to_string(ori_idx++);

      // Compute offsets wrt. to workspace frame.
      geometry_msgs::msg::Quaternion quat_offset;
      quat_offset.x = 0;
      quat_offset.y = 0;
      quat_offset.z = 0;
      quat_offset.w = 1;

      if (plan_request.workspace_parameters.header.frame_id != constraint.header.frame_id)
      {
        try
        {
          auto transform = tf_buffer_->lookupTransform(
              constraint.header.frame_id, plan_request.workspace_parameters.header.frame_id, tf2::TimePointZero);

          quat_offset = transform.transform.rotation;
        }
        catch (tf2::TransformException& ex)
        {
          RCLCPP_WARN(logger_,
                      "Skipping goal query append: "
                      "Could not get goal transform for orientation %s to %s: %s",
                      plan_request.workspace_parameters.header.frame_id.c_str(), constraint.header.frame_id.c_str(),
                      ex.what());

          // NOTE: methyldragon -
          //   Ideally we would restore the original state here and undo our changes, however copy of the query is not supported.
          return false;
        }
      }

      query.append(meta_name + ".link_name", constraint.link_name);

      // Orientation of constraint frame wrt. workspace frame
      tf2::Quaternion tf2_quat_frame_offset(quat_offset.x, quat_offset.y, quat_offset.z, quat_offset.w);

      // Added offset on top of the constraint frame's orientation stated in
      // the constraint.
      tf2::Quaternion tf2_quat_goal_offset(constraint.orientation.x, constraint.orientation.y, constraint.orientation.z,
                                           constraint.orientation.w);

      tf2_quat_frame_offset.normalize();
      tf2_quat_goal_offset.normalize();

      auto final_quat = tf2_quat_goal_offset * tf2_quat_frame_offset;
      final_quat.normalize();

      query_append_range_inclusive_with_tolerance(query, meta_name + ".target_point_offset.x", final_quat.getX(),
                                                  match_tolerance);
      query_append_range_inclusive_with_tolerance(query, meta_name + ".target_point_offset.y", final_quat.getY(),
                                                  match_tolerance);
      query_append_range_inclusive_with_tolerance(query, meta_name + ".target_point_offset.z", final_quat.getZ(),
                                                  match_tolerance);
      query_append_range_inclusive_with_tolerance(query, meta_name + ".target_point_offset.w", final_quat.getW(),
                                                  match_tolerance);
    }
  }

  return true;
}

// MOTION PLAN TRAJECTORY CACHING: METADATA CONSTRUCTION
bool TrajectoryCache::extract_and_append_trajectory_start_to_metadata(
    Metadata& metadata, const moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit_msgs::msg::MotionPlanRequest& plan_request)
{
  // Make ignored members explicit
  if (!plan_request.start_state.multi_dof_joint_state.joint_names.empty())
  {
    RCLCPP_WARN(logger_, "Ignoring start_state.multi_dof_joint_states: Not supported.");
  }
  if (!plan_request.start_state.attached_collision_objects.empty())
  {
    RCLCPP_WARN(logger_, "Ignoring start_state.attached_collision_objects: Not supported.");
  }

  metadata.append("group_name", plan_request.group_name);

  // Workspace params
  metadata.append("workspace_parameters.header.frame_id", plan_request.workspace_parameters.header.frame_id);
  metadata.append("workspace_parameters.min_corner.x", plan_request.workspace_parameters.min_corner.x);
  metadata.append("workspace_parameters.min_corner.y", plan_request.workspace_parameters.min_corner.y);
  metadata.append("workspace_parameters.min_corner.z", plan_request.workspace_parameters.min_corner.z);
  metadata.append("workspace_parameters.max_corner.x", plan_request.workspace_parameters.max_corner.x);
  metadata.append("workspace_parameters.max_corner.y", plan_request.workspace_parameters.max_corner.y);
  metadata.append("workspace_parameters.max_corner.z", plan_request.workspace_parameters.max_corner.z);

  // Joint state
  //   Only accounts for joint_state position. Ignores velocity and effort.
  if (plan_request.start_state.is_diff)
  {
    // If plan request states that the start_state is_diff, then we need to get
    // the current state from the move_group.

    // NOTE: methyldragon -
    //   I think if is_diff is on, the joint states will not be populated in all
    //   of our motion plan requests? If this isn't the case we might need to
    //   apply the joint states as offsets as well.
    //
    // TODO: Since MoveIt also potentially does another getCurrentState() call
    //   when planning, there is a chance that the current state in the cache
    //   differs from the state used in MoveIt's plan.
    //
    //   When upstreaming this class to MoveIt, this issue should go away once
    //   the class is used within the move group's Plan call.
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    if (!current_state)
    {
      RCLCPP_WARN(logger_, "Skipping start metadata append: Could not get robot state.");
      // NOTE: methyldragon -
      //   Ideally we would restore the original state here and undo our changes, however copy of the query is not supported.
      return false;
    }

    moveit_msgs::msg::RobotState current_state_msg;
    robotStateToRobotStateMsg(*current_state, current_state_msg);

    for (size_t i = 0; i < current_state_msg.joint_state.name.size(); i++)
    {
      metadata.append("start_state.joint_state.name_" + std::to_string(i), current_state_msg.joint_state.name.at(i));
      metadata.append("start_state.joint_state.position_" + std::to_string(i),
                      current_state_msg.joint_state.position.at(i));
    }
  }
  else
  {
    for (size_t i = 0; i < plan_request.start_state.joint_state.name.size(); i++)
    {
      metadata.append("start_state.joint_state.name_" + std::to_string(i),
                      plan_request.start_state.joint_state.name.at(i));
      metadata.append("start_state.joint_state.position_" + std::to_string(i),
                      plan_request.start_state.joint_state.position.at(i));
    }
  }

  return true;
}

bool TrajectoryCache::extract_and_append_trajectory_goal_to_metadata(
    Metadata& metadata, const moveit::planning_interface::MoveGroupInterface& /* move_group */,
    const moveit_msgs::msg::MotionPlanRequest& plan_request)
{
  // Make ignored members explicit
  bool emit_position_constraint_warning = false;
  for (auto& constraint : plan_request.goal_constraints)
  {
    for (auto& position_constraint : constraint.position_constraints)
    {
      if (!position_constraint.constraint_region.primitives.empty())
      {
        emit_position_constraint_warning = true;
        break;
      }
    }
    if (emit_position_constraint_warning)
    {
      break;
    }
  }
  if (emit_position_constraint_warning)
  {
    RCLCPP_WARN(logger_, "Ignoring goal_constraints.position_constraints.constraint_region: "
                                     "Not supported.");
  }

  metadata.append("max_velocity_scaling_factor", plan_request.max_velocity_scaling_factor);
  metadata.append("max_acceleration_scaling_factor", plan_request.max_acceleration_scaling_factor);
  metadata.append("max_cartesian_speed", plan_request.max_cartesian_speed);

  // Extract constraints (so we don't have cardinality on goal_constraint idx.)
  std::vector<moveit_msgs::msg::JointConstraint> joint_constraints;
  std::vector<moveit_msgs::msg::PositionConstraint> position_constraints;
  std::vector<moveit_msgs::msg::OrientationConstraint> orientation_constraints;
  for (auto& constraint : plan_request.goal_constraints)
  {
    for (auto& joint_constraint : constraint.joint_constraints)
    {
      joint_constraints.push_back(joint_constraint);
    }
    for (auto& position_constraint : constraint.position_constraints)
    {
      position_constraints.push_back(position_constraint);
    }
    for (auto& orientation_constraint : constraint.orientation_constraints)
    {
      orientation_constraints.push_back(orientation_constraint);
    }

    // Also sort for even less cardinality.
    sort_constraints(joint_constraints, position_constraints, orientation_constraints);
  }

  // Joint constraints
  size_t joint_idx = 0;
  for (auto& constraint : joint_constraints)
  {
    std::string meta_name = "goal_constraints.joint_constraints_" + std::to_string(joint_idx++);

    metadata.append(meta_name + ".joint_name", constraint.joint_name);
    metadata.append(meta_name + ".position", constraint.position);
    metadata.append(meta_name + ".tolerance_above", constraint.tolerance_above);
    metadata.append(meta_name + ".tolerance_below", constraint.tolerance_below);
  }

  // Position constraints
  if (!position_constraints.empty())
  {
    // All offsets will be "frozen" and computed wrt. the workspace frame
    // instead.
    metadata.append("goal_constraints.position_constraints.header.frame_id",
                    plan_request.workspace_parameters.header.frame_id);

    size_t position_idx = 0;
    for (auto& constraint : position_constraints)
    {
      std::string meta_name = "goal_constraints.position_constraints_" + std::to_string(position_idx++);

      // Compute offsets wrt. to workspace frame.
      double x_offset = 0;
      double y_offset = 0;
      double z_offset = 0;

      if (plan_request.workspace_parameters.header.frame_id != constraint.header.frame_id)
      {
        try
        {
          auto transform = tf_buffer_->lookupTransform(
              constraint.header.frame_id, plan_request.workspace_parameters.header.frame_id, tf2::TimePointZero);

          x_offset = transform.transform.translation.x;
          y_offset = transform.transform.translation.y;
          z_offset = transform.transform.translation.z;
        }
        catch (tf2::TransformException& ex)
        {
          RCLCPP_WARN(logger_,
                      "Skipping goal metadata append: "
                      "Could not get goal transform for translation %s to %s: %s",
                      plan_request.workspace_parameters.header.frame_id.c_str(), constraint.header.frame_id.c_str(),
                      ex.what());

          // NOTE: methyldragon -
          //   Ideally we would restore the original state here and undo our changes, however copy of the query is not supported.
          return false;
        }
      }

      metadata.append(meta_name + ".link_name", constraint.link_name);

      metadata.append(meta_name + ".target_point_offset.x", x_offset + constraint.target_point_offset.x);
      metadata.append(meta_name + ".target_point_offset.y", y_offset + constraint.target_point_offset.y);
      metadata.append(meta_name + ".target_point_offset.z", z_offset + constraint.target_point_offset.z);
    }
  }

  // Orientation constraints
  if (!orientation_constraints.empty())
  {
    // All offsets will be "frozen" and computed wrt. the workspace frame
    // instead.
    metadata.append("goal_constraints.orientation_constraints.header.frame_id",
                    plan_request.workspace_parameters.header.frame_id);

    size_t ori_idx = 0;
    for (auto& constraint : orientation_constraints)
    {
      std::string meta_name = "goal_constraints.orientation_constraints_" + std::to_string(ori_idx++);

      // Compute offsets wrt. to workspace frame.
      geometry_msgs::msg::Quaternion quat_offset;
      quat_offset.x = 0;
      quat_offset.y = 0;
      quat_offset.z = 0;
      quat_offset.w = 1;

      if (plan_request.workspace_parameters.header.frame_id != constraint.header.frame_id)
      {
        try
        {
          auto transform = tf_buffer_->lookupTransform(
              constraint.header.frame_id, plan_request.workspace_parameters.header.frame_id, tf2::TimePointZero);

          quat_offset = transform.transform.rotation;
        }
        catch (tf2::TransformException& ex)
        {
          RCLCPP_WARN(logger_,
                      "Skipping goal metadata append: "
                      "Could not get goal transform for orientation %s to %s: %s",
                      plan_request.workspace_parameters.header.frame_id.c_str(), constraint.header.frame_id.c_str(),
                      ex.what());

          // NOTE: methyldragon -
          //   Ideally we would restore the original state here and undo our changes, however copy of the query is not supported.
          return false;
        }
      }

      metadata.append(meta_name + ".link_name", constraint.link_name);

      // Orientation of constraint frame wrt. workspace frame
      tf2::Quaternion tf2_quat_frame_offset(quat_offset.x, quat_offset.y, quat_offset.z, quat_offset.w);

      // Added offset on top of the constraint frame's orientation stated in
      // the constraint.
      tf2::Quaternion tf2_quat_goal_offset(constraint.orientation.x, constraint.orientation.y, constraint.orientation.z,
                                           constraint.orientation.w);

      tf2_quat_frame_offset.normalize();
      tf2_quat_goal_offset.normalize();

      auto final_quat = tf2_quat_goal_offset * tf2_quat_frame_offset;
      final_quat.normalize();

      metadata.append(meta_name + ".target_point_offset.x", final_quat.getX());
      metadata.append(meta_name + ".target_point_offset.y", final_quat.getY());
      metadata.append(meta_name + ".target_point_offset.z", final_quat.getZ());
      metadata.append(meta_name + ".target_point_offset.w", final_quat.getW());
    }
  }

  return true;
}

// =============================================================================
// CARTESIAN TRAJECTORY CACHING
// =============================================================================
// CARTESIAN TRAJECTORY CACHING: TOP LEVEL OPS
moveit_msgs::srv::GetCartesianPath::Request
TrajectoryCache::construct_get_cartesian_path_request(moveit::planning_interface::MoveGroupInterface& move_group,
                                                      const std::vector<geometry_msgs::msg::Pose>& waypoints,
                                                      double step, double jump_threshold, bool avoid_collisions)
{
  moveit_msgs::srv::GetCartesianPath::Request out;

  // Some of these parameters need us to pull PRIVATE values out of the
  // move_group elsewhere... Yes, it is very cursed and I hate it.
  // Fixing it requires fixing it in MoveIt.
  moveit_msgs::msg::MotionPlanRequest tmp;
  move_group.constructMotionPlanRequest(tmp);

  out.start_state = std::move(tmp.start_state);
  out.group_name = std::move(tmp.group_name);
  out.max_velocity_scaling_factor = tmp.max_velocity_scaling_factor;
  out.max_acceleration_scaling_factor = tmp.max_acceleration_scaling_factor;

  out.header.frame_id = move_group.getPoseReferenceFrame();
  out.waypoints = waypoints;
  out.max_step = step;
  out.jump_threshold = jump_threshold;
  out.path_constraints = moveit_msgs::msg::Constraints();
  out.avoid_collisions = avoid_collisions;
  out.link_name = move_group.getEndEffectorLink();
  out.header.stamp = move_group.getNode()->now();

  return out;
}

std::vector<MessageWithMetadata<moveit_msgs::msg::RobotTrajectory>::ConstPtr>
TrajectoryCache::fetch_all_matching_cartesian_trajectories(
    const moveit::planning_interface::MoveGroupInterface& move_group, const std::string& move_group_namespace,
    const moveit_msgs::srv::GetCartesianPath::Request& plan_request, double min_fraction, double start_tolerance,
    double goal_tolerance, bool metadata_only, const std::string& sort_by)
{
  auto coll = db_->openCollection<moveit_msgs::msg::RobotTrajectory>("move_group_cartesian_trajectory_cache",
                                                                     move_group_namespace);

  Query::Ptr query = coll.createQuery();

  bool start_ok =
      this->extract_and_append_cartesian_trajectory_start_to_query(*query, move_group, plan_request, start_tolerance);
  bool goal_ok =
      this->extract_and_append_cartesian_trajectory_goal_to_query(*query, move_group, plan_request, goal_tolerance);

  if (!start_ok || !goal_ok)
  {
    RCLCPP_ERROR(logger_, "Could not construct cartesian trajectory query.");
    return {};
  }

  query->appendGTE("fraction", min_fraction);
  return coll.queryList(query, metadata_only, sort_by, true);
}

MessageWithMetadata<moveit_msgs::msg::RobotTrajectory>::ConstPtr
TrajectoryCache::fetch_best_matching_cartesian_trajectory(
    const moveit::planning_interface::MoveGroupInterface& move_group, const std::string& move_group_namespace,
    const moveit_msgs::srv::GetCartesianPath::Request& plan_request, double min_fraction, double start_tolerance,
    double goal_tolerance, bool metadata_only, const std::string& sort_by)
{
  // First find all matching, but metadata only.
  // Then use the ID metadata of the best plan to pull the actual message.
  auto matching_trajectories = this->fetch_all_matching_cartesian_trajectories(
      move_group, move_group_namespace, plan_request, min_fraction, start_tolerance, goal_tolerance, true, sort_by);

  if (matching_trajectories.empty())
  {
    RCLCPP_DEBUG(logger_, "No matching cartesian trajectories found.");
    return nullptr;
  }

  auto coll = db_->openCollection<moveit_msgs::msg::RobotTrajectory>("move_group_cartesian_trajectory_cache",
                                                                     move_group_namespace);

  // Best plan is at first index, since the lookup query was sorted by
  // execution_time.
  int best_trajectory_id = matching_trajectories.at(0)->lookupInt("id");
  Query::Ptr best_query = coll.createQuery();
  best_query->append("id", best_trajectory_id);

  return coll.findOne(best_query, metadata_only);
}

bool TrajectoryCache::put_cartesian_trajectory(const moveit::planning_interface::MoveGroupInterface& move_group,
                                               const std::string& move_group_namespace,
                                               const moveit_msgs::srv::GetCartesianPath::Request& plan_request,
                                               const moveit_msgs::msg::RobotTrajectory& trajectory,
                                               double execution_time_s, double planning_time_s, double fraction,
                                               bool delete_worse_trajectories)
{
  // Check pre-conditions
  if (!trajectory.multi_dof_joint_trajectory.points.empty())
  {
    RCLCPP_ERROR(logger_, "Skipping cartesian trajectory insert: "
                                      "Multi-DOF trajectory plans are not supported.");
    return false;
  }
  if (plan_request.header.frame_id.empty() || trajectory.joint_trajectory.header.frame_id.empty())
  {
    RCLCPP_ERROR(logger_, "Skipping cartesian trajectory insert: Frame IDs cannot be empty.");
    return false;
  }

  auto coll = db_->openCollection<moveit_msgs::msg::RobotTrajectory>("move_group_cartesian_trajectory_cache",
                                                                     move_group_namespace);

  // Pull out trajectories "exactly" keyed by request in cache.
  Query::Ptr exact_query = coll.createQuery();

  bool start_query_ok =
      this->extract_and_append_cartesian_trajectory_start_to_query(*exact_query, move_group, plan_request, 0);
  bool goal_query_ok =
      this->extract_and_append_cartesian_trajectory_goal_to_query(*exact_query, move_group, plan_request, 0);
  exact_query->append("fraction", fraction);

  if (!start_query_ok || !goal_query_ok)
  {
    RCLCPP_ERROR(logger_, "Skipping cartesian trajectory insert: Could not construct lookup query.");
    return false;
  }

  auto exact_matches = coll.queryList(exact_query, /* metadata_only */ true, /* sort_by */ "execution_time_s");
  double best_execution_time = std::numeric_limits<double>::infinity();
  if (!exact_matches.empty())
  {
    best_execution_time = exact_matches.at(0)->lookupDouble("execution_time_s");

    if (delete_worse_trajectories)
    {
      for (auto& match : exact_matches)
      {
        double match_execution_time_s = match->lookupDouble("execution_time_s");
        if (execution_time_s < match_execution_time_s)
        {
          int delete_id = match->lookupInt("id");
          RCLCPP_DEBUG(logger_,
                      "Overwriting cartesian trajectory (id: %d): "
                      "execution_time (%es) > new trajectory's execution_time (%es)",
                      delete_id, match_execution_time_s, execution_time_s);

          Query::Ptr delete_query = coll.createQuery();
          delete_query->append("id", delete_id);
          coll.removeMessages(delete_query);
        }
      }
    }
  }

  // Insert if candidate is best seen.
  if (execution_time_s < best_execution_time)
  {
    Metadata::Ptr insert_metadata = coll.createMetadata();

    bool start_meta_ok =
        this->extract_and_append_cartesian_trajectory_start_to_metadata(*insert_metadata, move_group, plan_request);
    bool goal_meta_ok =
        this->extract_and_append_cartesian_trajectory_goal_to_metadata(*insert_metadata, move_group, plan_request);
    insert_metadata->append("execution_time_s", execution_time_s);
    insert_metadata->append("planning_time_s", planning_time_s);
    insert_metadata->append("fraction", fraction);

    if (!start_meta_ok || !goal_meta_ok)
    {
      RCLCPP_ERROR(logger_, "Skipping cartesian trajectory insert: "
                                        "Could not construct insert metadata.");
      return false;
    }

    RCLCPP_DEBUG(logger_,
                "Inserting cartesian trajectory: New trajectory execution_time (%es) "
                "is better than best trajectory's execution_time (%es) at fraction (%es)",
                execution_time_s, best_execution_time, fraction);

    coll.insert(trajectory, insert_metadata);
    return true;
  }

  RCLCPP_DEBUG(logger_,
              "Skipping cartesian trajectory insert: New trajectory execution_time (%es) "
              "is worse than best trajectory's execution_time (%es) at fraction (%es)",
              execution_time_s, best_execution_time, fraction);
  return false;
}

// CARTESIAN TRAJECTORY CACHING: QUERY CONSTRUCTION
bool TrajectoryCache::extract_and_append_cartesian_trajectory_start_to_query(
    Query& query, const moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit_msgs::srv::GetCartesianPath::Request& plan_request, double match_tolerance)
{
  match_tolerance += exact_match_precision_;

  // Make ignored members explicit
  if (!plan_request.start_state.multi_dof_joint_state.joint_names.empty())
  {
    RCLCPP_WARN(logger_, "Ignoring start_state.multi_dof_joint_states: Not supported.");
  }
  if (!plan_request.start_state.attached_collision_objects.empty())
  {
    RCLCPP_WARN(logger_, "Ignoring start_state.attached_collision_objects: Not supported.");
  }

  query.append("group_name", plan_request.group_name);

  // Joint state
  //   Only accounts for joint_state position. Ignores velocity and effort.
  if (plan_request.start_state.is_diff)
  {
    // If plan request states that the start_state is_diff, then we need to get
    // the current state from the move_group.

    // NOTE: methyldragon -
    //   I think if is_diff is on, the joint states will not be populated in all
    //   of our motion plan requests? If this isn't the case we might need to
    //   apply the joint states as offsets as well.
    //
    // TODO: Since MoveIt also potentially does another getCurrentState() call
    //   when planning, there is a chance that the current state in the cache
    //   differs from the state used in MoveIt's plan.
    //
    //   When upstreaming this class to MoveIt, this issue should go away once
    //   the class is used within the move group's Plan call.
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    if (!current_state)
    {
      RCLCPP_WARN(logger_, "Skipping start metadata append: Could not get robot state.");
      // NOTE: methyldragon -
      //   Ideally we would restore the original state here and undo our changes, however copy of the query is not supported.
      return false;
    }

    moveit_msgs::msg::RobotState current_state_msg;
    robotStateToRobotStateMsg(*current_state, current_state_msg);

    for (size_t i = 0; i < current_state_msg.joint_state.name.size(); i++)
    {
      query.append("start_state.joint_state.name_" + std::to_string(i), current_state_msg.joint_state.name.at(i));
      query_append_range_inclusive_with_tolerance(query, "start_state.joint_state.position_" + std::to_string(i),
                                                  current_state_msg.joint_state.position.at(i), match_tolerance);
    }
  }
  else
  {
    for (size_t i = 0; i < plan_request.start_state.joint_state.name.size(); i++)
    {
      query.append("start_state.joint_state.name_" + std::to_string(i), plan_request.start_state.joint_state.name.at(i));
      query_append_range_inclusive_with_tolerance(query, "start_state.joint_state.position_" + std::to_string(i),
                                                  plan_request.start_state.joint_state.position.at(i), match_tolerance);
    }
  }

  return true;
}

bool TrajectoryCache::extract_and_append_cartesian_trajectory_goal_to_query(
    Query& query, const moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit_msgs::srv::GetCartesianPath::Request& plan_request, double match_tolerance)
{
  match_tolerance += exact_match_precision_;

  // Make ignored members explicit
  if (!plan_request.path_constraints.joint_constraints.empty() ||
      !plan_request.path_constraints.position_constraints.empty() ||
      !plan_request.path_constraints.orientation_constraints.empty() ||
      !plan_request.path_constraints.visibility_constraints.empty())
  {
    RCLCPP_WARN(logger_, "Ignoring path_constraints: Not supported.");
  }
  if (plan_request.avoid_collisions)
  {
    RCLCPP_WARN(logger_, "Ignoring avoid_collisions: Not supported.");
  }

  query_append_range_inclusive_with_tolerance(query, "max_velocity_scaling_factor",
                                              plan_request.max_velocity_scaling_factor, match_tolerance);
  query_append_range_inclusive_with_tolerance(query, "max_acceleration_scaling_factor",
                                              plan_request.max_acceleration_scaling_factor, match_tolerance);
  query_append_range_inclusive_with_tolerance(query, "max_step", plan_request.max_step, match_tolerance);
  query_append_range_inclusive_with_tolerance(query, "jump_threshold", plan_request.jump_threshold, match_tolerance);

  // Waypoints
  // Restating them in terms of the robot model frame (usually base_link)
  std::string base_frame = move_group.getRobotModel()->getModelFrame();

  // Compute offsets.
  double x_offset = 0;
  double y_offset = 0;
  double z_offset = 0;

  geometry_msgs::msg::Quaternion quat_offset;
  quat_offset.x = 0;
  quat_offset.y = 0;
  quat_offset.z = 0;
  quat_offset.w = 1;

  if (base_frame != plan_request.header.frame_id)
  {
    try
    {
      auto transform = tf_buffer_->lookupTransform(plan_request.header.frame_id, base_frame, tf2::TimePointZero);

      x_offset = transform.transform.translation.x;
      y_offset = transform.transform.translation.y;
      z_offset = transform.transform.translation.z;
      quat_offset = transform.transform.rotation;
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_WARN(logger_,
                  "Skipping goal metadata append: "
                  "Could not get goal transform for %s to %s: %s",
                  base_frame.c_str(), plan_request.header.frame_id.c_str(), ex.what());

      // NOTE: methyldragon -
      //   Ideally we would restore the original state here and undo our changes, however copy of the query is not supported.
      return false;
    }
  }

  tf2::Quaternion tf2_quat_frame_offset(quat_offset.x, quat_offset.y, quat_offset.z, quat_offset.w);
  tf2_quat_frame_offset.normalize();

  size_t waypoint_idx = 0;
  for (auto& waypoint : plan_request.waypoints)
  {
    std::string meta_name = "waypoints_" + std::to_string(waypoint_idx++);

    // Apply offsets
    // Position
    query_append_range_inclusive_with_tolerance(query, meta_name + ".position.x", x_offset + waypoint.position.x,
                                                match_tolerance);
    query_append_range_inclusive_with_tolerance(query, meta_name + ".position.y", y_offset + waypoint.position.y,
                                                match_tolerance);
    query_append_range_inclusive_with_tolerance(query, meta_name + ".position.z", z_offset + waypoint.position.z,
                                                match_tolerance);

    // Orientation
    tf2::Quaternion tf2_quat_goal_offset(waypoint.orientation.x, waypoint.orientation.y, waypoint.orientation.z,
                                         waypoint.orientation.w);
    tf2_quat_goal_offset.normalize();

    auto final_quat = tf2_quat_goal_offset * tf2_quat_frame_offset;
    final_quat.normalize();

    query_append_range_inclusive_with_tolerance(query, meta_name + ".orientation.x", final_quat.getX(), match_tolerance);
    query_append_range_inclusive_with_tolerance(query, meta_name + ".orientation.y", final_quat.getY(), match_tolerance);
    query_append_range_inclusive_with_tolerance(query, meta_name + ".orientation.z", final_quat.getZ(), match_tolerance);
    query_append_range_inclusive_with_tolerance(query, meta_name + ".orientation.w", final_quat.getW(), match_tolerance);
  }

  query.append("link_name", plan_request.link_name);
  query.append("header.frame_id", base_frame);

  return true;
}

// CARTESIAN TRAJECTORY CACHING: METADATA CONSTRUCTION
bool TrajectoryCache::extract_and_append_cartesian_trajectory_start_to_metadata(
    Metadata& metadata, const moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit_msgs::srv::GetCartesianPath::Request& plan_request)
{
  // Make ignored members explicit
  if (!plan_request.start_state.multi_dof_joint_state.joint_names.empty())
  {
    RCLCPP_WARN(logger_, "Ignoring start_state.multi_dof_joint_states: Not supported.");
  }
  if (!plan_request.start_state.attached_collision_objects.empty())
  {
    RCLCPP_WARN(logger_, "Ignoring start_state.attached_collision_objects: Not supported.");
  }

  metadata.append("group_name", plan_request.group_name);

  // Joint state
  //   Only accounts for joint_state position. Ignores velocity and effort.
  if (plan_request.start_state.is_diff)
  {
    // If plan request states that the start_state is_diff, then we need to get
    // the current state from the move_group.

    // NOTE: methyldragon -
    //   I think if is_diff is on, the joint states will not be populated in all
    //   of our motion plan requests? If this isn't the case we might need to
    //   apply the joint states as offsets as well.
    //
    // TODO: Since MoveIt also potentially does another getCurrentState() call
    //   when planning, there is a chance that the current state in the cache
    //   differs from the state used in MoveIt's plan.
    //
    //   When upstreaming this class to MoveIt, this issue should go away once
    //   the class is used within the move group's Plan call.
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    if (!current_state)
    {
      RCLCPP_WARN(logger_, "Skipping start metadata append: Could not get robot state.");
      // NOTE: methyldragon -
      //   Ideally we would restore the original state here and undo our changes, however copy of the query is not supported.
      return false;
    }

    moveit_msgs::msg::RobotState current_state_msg;
    robotStateToRobotStateMsg(*current_state, current_state_msg);

    for (size_t i = 0; i < current_state_msg.joint_state.name.size(); i++)
    {
      metadata.append("start_state.joint_state.name_" + std::to_string(i), current_state_msg.joint_state.name.at(i));
      metadata.append("start_state.joint_state.position_" + std::to_string(i),
                      current_state_msg.joint_state.position.at(i));
    }
  }
  else
  {
    for (size_t i = 0; i < plan_request.start_state.joint_state.name.size(); i++)
    {
      metadata.append("start_state.joint_state.name_" + std::to_string(i),
                      plan_request.start_state.joint_state.name.at(i));
      metadata.append("start_state.joint_state.position_" + std::to_string(i),
                      plan_request.start_state.joint_state.position.at(i));
    }
  }

  return true;
}

bool TrajectoryCache::extract_and_append_cartesian_trajectory_goal_to_metadata(
    Metadata& metadata, const moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit_msgs::srv::GetCartesianPath::Request& plan_request)
{
  // Make ignored members explicit
  if (!plan_request.path_constraints.joint_constraints.empty() ||
      !plan_request.path_constraints.position_constraints.empty() ||
      !plan_request.path_constraints.orientation_constraints.empty() ||
      !plan_request.path_constraints.visibility_constraints.empty())
  {
    RCLCPP_WARN(logger_, "Ignoring path_constraints: Not supported.");
  }
  if (plan_request.avoid_collisions)
  {
    RCLCPP_WARN(logger_, "Ignoring avoid_collisions: Not supported.");
  }

  metadata.append("max_velocity_scaling_factor", plan_request.max_velocity_scaling_factor);
  metadata.append("max_acceleration_scaling_factor", plan_request.max_acceleration_scaling_factor);
  metadata.append("max_step", plan_request.max_step);
  metadata.append("jump_threshold", plan_request.jump_threshold);

  // Waypoints
  // Restating them in terms of the robot model frame (usually base_link)
  std::string base_frame = move_group.getRobotModel()->getModelFrame();

  // Compute offsets.
  double x_offset = 0;
  double y_offset = 0;
  double z_offset = 0;

  geometry_msgs::msg::Quaternion quat_offset;
  quat_offset.x = 0;
  quat_offset.y = 0;
  quat_offset.z = 0;
  quat_offset.w = 1;

  if (base_frame != plan_request.header.frame_id)
  {
    try
    {
      auto transform = tf_buffer_->lookupTransform(plan_request.header.frame_id, base_frame, tf2::TimePointZero);

      x_offset = transform.transform.translation.x;
      y_offset = transform.transform.translation.y;
      z_offset = transform.transform.translation.z;
      quat_offset = transform.transform.rotation;
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_WARN(logger_,
                  "Skipping goal metadata append: "
                  "Could not get goal transform for %s to %s: %s",
                  base_frame.c_str(), plan_request.header.frame_id.c_str(), ex.what());

      // NOTE: methyldragon -
      //   Ideally we would restore the original state here and undo our changes, however copy of the query is not supported.
      return false;
    }
  }

  tf2::Quaternion tf2_quat_frame_offset(quat_offset.x, quat_offset.y, quat_offset.z, quat_offset.w);
  tf2_quat_frame_offset.normalize();

  size_t waypoint_idx = 0;
  for (auto& waypoint : plan_request.waypoints)
  {
    std::string meta_name = "waypoints_" + std::to_string(waypoint_idx++);

    // Apply offsets
    // Position
    metadata.append(meta_name + ".position.x", x_offset + waypoint.position.x);
    metadata.append(meta_name + ".position.y", y_offset + waypoint.position.y);
    metadata.append(meta_name + ".position.z", z_offset + waypoint.position.z);

    // Orientation
    tf2::Quaternion tf2_quat_goal_offset(waypoint.orientation.x, waypoint.orientation.y, waypoint.orientation.z,
                                         waypoint.orientation.w);
    tf2_quat_goal_offset.normalize();

    auto final_quat = tf2_quat_goal_offset * tf2_quat_frame_offset;
    final_quat.normalize();

    metadata.append(meta_name + ".orientation.x", final_quat.getX());
    metadata.append(meta_name + ".orientation.y", final_quat.getY());
    metadata.append(meta_name + ".orientation.z", final_quat.getZ());
    metadata.append(meta_name + ".orientation.w", final_quat.getW());
  }

  metadata.append("link_name", plan_request.link_name);
  metadata.append("header.frame_id", base_frame);

  return true;
}

}  // namespace trajectory_cache
}  // namespace moveit_ros
