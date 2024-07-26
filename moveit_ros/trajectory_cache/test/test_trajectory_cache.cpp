// Copyright 2023 Johnson & Johnson
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

#include <rclcpp/rclcpp.hpp>

#include "moveit/robot_state/conversions.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit/trajectory_cache/trajectory_cache.hpp"

#include <atomic>
#include <thread>

using moveit::planning_interface::MoveGroupInterface;
using moveit_ros::trajectory_cache::TrajectoryCache;

const std::string ROBOT_NAME = "panda";
const std::string ROBOT_FRAME = "world";

// UTILS =======================================================================
// Utility function to emit a pass or fail statement.
void check_and_emit(bool predicate, const std::string& prefix, const std::string& label)
{
  if (predicate)
  {
    std::cout << "[PASS] " << prefix << ": " << label << std::endl;
  }
  else
  {
    std::cout << "[FAIL] " << prefix << ": " << label << std::endl;
  }
}

moveit_msgs::msg::RobotTrajectory get_dummy_panda_traj()
{
  static moveit_msgs::msg::RobotTrajectory out = []() {
    moveit_msgs::msg::RobotTrajectory traj;

    auto trajectory = &traj.joint_trajectory;
    trajectory->header.frame_id = ROBOT_FRAME;

    trajectory->joint_names.push_back(ROBOT_NAME + "_joint1");
    trajectory->joint_names.push_back(ROBOT_NAME + "_joint2");
    trajectory->joint_names.push_back(ROBOT_NAME + "_joint3");
    trajectory->joint_names.push_back(ROBOT_NAME + "_joint4");
    trajectory->joint_names.push_back(ROBOT_NAME + "_joint5");
    trajectory->joint_names.push_back(ROBOT_NAME + "_joint6");
    trajectory->joint_names.push_back(ROBOT_NAME + "_joint7");

    trajectory->points.emplace_back();
    trajectory->points.at(0).positions = { 0, 0, 0, 0, 0, 0 };
    trajectory->points.at(0).velocities = { 0, 0, 0, 0, 0, 0 };
    trajectory->points.at(0).accelerations = { 0, 0, 0, 0, 0, 0 };
    trajectory->points.at(0).time_from_start.sec = 999999;

    return traj;
  }();
  return out;
}

std::vector<geometry_msgs::msg::Pose> get_dummy_waypoints()
{
  static std::vector<geometry_msgs::msg::Pose> out = []() {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    for (size_t i = 0; i < 3; i++)
    {
      waypoints.emplace_back();
      waypoints.at(i).position.x = i;
      waypoints.at(i).position.y = i;
      waypoints.at(i).position.z = i;
      waypoints.at(i).orientation.w = i;
      waypoints.at(i).orientation.x = i + 0.1;
      waypoints.at(i).orientation.y = i + 0.1;
      waypoints.at(i).orientation.z = i + 0.1;
    }
    return waypoints;
  }();
  return out;
}

void test_motion_trajectories(std::shared_ptr<MoveGroupInterface> move_group, std::shared_ptr<TrajectoryCache> cache)
{
  // Setup =====================================================================
  // All variants are modified copies of `plan_req`.

  /// MotionPlanRequest

  // Plain start
  moveit_msgs::msg::MotionPlanRequest plan_req;
  move_group->constructMotionPlanRequest(plan_req);
  plan_req.workspace_parameters.header.frame_id = ROBOT_FRAME;
  plan_req.workspace_parameters.max_corner.x = 10;
  plan_req.workspace_parameters.max_corner.y = 10;
  plan_req.workspace_parameters.max_corner.z = 10;
  plan_req.workspace_parameters.min_corner.x = -10;
  plan_req.workspace_parameters.min_corner.y = -10;
  plan_req.workspace_parameters.min_corner.z = -10;
  plan_req.start_state.multi_dof_joint_state.joint_names.clear();
  plan_req.start_state.multi_dof_joint_state.transforms.clear();
  plan_req.start_state.multi_dof_joint_state.twist.clear();
  plan_req.start_state.multi_dof_joint_state.wrench.clear();

  // Empty frame start
  moveit_msgs::msg::MotionPlanRequest empty_frame_plan_req = plan_req;
  empty_frame_plan_req.workspace_parameters.header.frame_id = "";

  // is_diff = true
  auto is_diff_plan_req = plan_req;
  is_diff_plan_req.start_state.is_diff = true;
  is_diff_plan_req.start_state.joint_state.header.frame_id = "";
  is_diff_plan_req.start_state.joint_state.name.clear();
  is_diff_plan_req.start_state.joint_state.position.clear();
  is_diff_plan_req.start_state.joint_state.velocity.clear();
  is_diff_plan_req.start_state.joint_state.effort.clear();

  // Something close enough (mod 0.1 away)
  auto close_matching_plan_req = plan_req;
  close_matching_plan_req.workspace_parameters.max_corner.x += 0.05;
  close_matching_plan_req.workspace_parameters.min_corner.x -= 0.05;
  close_matching_plan_req.start_state.joint_state.position.at(0) -= 0.05;
  close_matching_plan_req.start_state.joint_state.position.at(1) += 0.05;
  close_matching_plan_req.start_state.joint_state.position.at(2) -= 0.05;
  close_matching_plan_req.goal_constraints.at(0).joint_constraints.at(0).position -= 0.05;
  close_matching_plan_req.goal_constraints.at(0).joint_constraints.at(1).position += 0.05;
  close_matching_plan_req.goal_constraints.at(0).joint_constraints.at(2).position -= 0.05;

  // Close with swapped constraints (mod 0.1 away)
  auto swapped_close_matching_plan_req = close_matching_plan_req;
  std::swap(swapped_close_matching_plan_req.goal_constraints.at(0).joint_constraints.at(0),
            swapped_close_matching_plan_req.goal_constraints.at(0).joint_constraints.at(1));

  // Smaller workspace start
  auto smaller_workspace_plan_req = plan_req;
  smaller_workspace_plan_req.workspace_parameters.max_corner.x = 1;
  smaller_workspace_plan_req.workspace_parameters.max_corner.y = 1;
  smaller_workspace_plan_req.workspace_parameters.max_corner.z = 1;
  smaller_workspace_plan_req.workspace_parameters.min_corner.x = -1;
  smaller_workspace_plan_req.workspace_parameters.min_corner.y = -1;
  smaller_workspace_plan_req.workspace_parameters.min_corner.z = -1;

  // Larger workspace start
  auto larger_workspace_plan_req = plan_req;
  larger_workspace_plan_req.workspace_parameters.max_corner.x = 50;
  larger_workspace_plan_req.workspace_parameters.max_corner.y = 50;
  larger_workspace_plan_req.workspace_parameters.max_corner.z = 50;
  larger_workspace_plan_req.workspace_parameters.min_corner.x = -50;
  larger_workspace_plan_req.workspace_parameters.min_corner.y = -50;
  larger_workspace_plan_req.workspace_parameters.min_corner.z = -50;

  // Different
  auto different_plan_req = plan_req;
  different_plan_req.workspace_parameters.max_corner.x += 1.05;
  different_plan_req.workspace_parameters.min_corner.x -= 2.05;
  different_plan_req.start_state.joint_state.position.at(0) -= 3.05;
  different_plan_req.start_state.joint_state.position.at(1) += 4.05;
  different_plan_req.start_state.joint_state.position.at(2) -= 5.05;
  different_plan_req.goal_constraints.at(0).joint_constraints.at(0).position -= 6.05;
  different_plan_req.goal_constraints.at(0).joint_constraints.at(1).position += 7.05;
  different_plan_req.goal_constraints.at(0).joint_constraints.at(2).position -= 8.05;

  /// RobotTrajectory

  // Trajectory
  auto traj = get_dummy_panda_traj();

  // Trajectory with no frame_id in its trajectory header
  auto empty_frame_traj = traj;
  empty_frame_traj.joint_trajectory.header.frame_id = "";

  auto different_traj = traj;
  different_traj.joint_trajectory.points.at(0).positions.at(0) = 999;
  different_traj.joint_trajectory.points.at(0).positions.at(1) = 999;
  different_traj.joint_trajectory.points.at(0).positions.at(2) = 999;

  // Test Utils

  std::string prefix;

  // Checks ====================================================================

  // Initially empty
  prefix = "test_motion_trajectories.empty";

  check_and_emit(cache->countTrajectories(ROBOT_NAME) == 0, prefix, "Trajectory cache initially empty");

  check_and_emit(cache->fetchAllMatchingTrajectories(*move_group, ROBOT_NAME, plan_req, 999, 999).empty(), prefix,
                 "Fetch all trajectories on empty cache returns empty");

  check_and_emit(cache->fetchBestMatchingTrajectory(*move_group, ROBOT_NAME, plan_req, 999, 999) == nullptr, prefix,
                 "Fetch best trajectory on empty cache returns nullptr");

  // Put trajectory empty frame
  //
  // Trajectory must have frame in joint trajectory, expect put fail
  prefix = "test_motion_trajectories.putTrajectory_empty_frame";
  double execution_time = 999;
  double planning_time = 999;

  check_and_emit(!cache->putTrajectory(*move_group, ROBOT_NAME, plan_req, empty_frame_traj, execution_time,
                                       planning_time, false),
                 prefix, "Put empty frame trajectory, no delete_worse_trajectories, not ok");

  check_and_emit(cache->countTrajectories(ROBOT_NAME) == 0, prefix, "No trajectories in cache");

  // Put trajectory req empty frame
  //
  // Trajectory request having no frame in workspace should default to robot frame
  prefix = "test_motion_trajectories.putTrajectory_req_empty_frame";
  execution_time = 1000;
  planning_time = 1000;

  check_and_emit(cache->putTrajectory(*move_group, ROBOT_NAME, empty_frame_plan_req, traj, execution_time,
                                      planning_time, false),
                 prefix, "Put empty frame req trajectory, no delete_worse_trajectories, ok");

  check_and_emit(cache->countTrajectories(ROBOT_NAME) == 1, prefix, "One trajectory in cache");

  // Put second, no delete_worse_trajectories
  prefix = "test_motion_trajectories.put_second";
  execution_time = 999;
  planning_time = 999;

  check_and_emit(cache->putTrajectory(*move_group, ROBOT_NAME, plan_req, traj, execution_time, planning_time, false),
                 prefix, "Put second valid trajectory, no delete_worse_trajectories, ok");

  check_and_emit(cache->countTrajectories(ROBOT_NAME) == 2, prefix, "Two trajectories in cache");

  // Fetch matching, no tolerance
  //
  // Exact key match should have cache hit
  prefix = "test_motion_trajectories.put_second.fetch_matching_no_tolerance";

  auto fetched_trajectories = cache->fetchAllMatchingTrajectories(*move_group, ROBOT_NAME, plan_req, 0, 0);

  auto fetched_traj = cache->fetchBestMatchingTrajectory(*move_group, ROBOT_NAME, plan_req, 0, 0);

  check_and_emit(fetched_trajectories.size() == 2, prefix, "Fetch all returns two");
  check_and_emit(fetched_traj != nullptr, prefix, "Fetch best trajectory is not nullptr");

  check_and_emit(*fetched_trajectories.at(0) == *fetched_traj, prefix, "Fetched trajectory on both fetches match");

  check_and_emit(*fetched_traj == traj, prefix, "Fetched trajectory matches original");

  check_and_emit(fetched_traj->lookupDouble("execution_time_s") == execution_time, prefix,
                 "Fetched trajectory has correct execution time");

  check_and_emit(fetched_traj->lookupDouble("planning_time_s") == planning_time, prefix,
                 "Fetched trajectory has correct planning time");

  // Fetch with is_diff
  //
  // is_diff key should be equivalent to exact match if robot state did not
  // change, hence should have cache hit
  prefix = "test_motion_trajectories.put_second.fetch_is_diff_no_tolerance";

  fetched_trajectories = cache->fetchAllMatchingTrajectories(*move_group, ROBOT_NAME, is_diff_plan_req, 0, 0);

  fetched_traj = cache->fetchBestMatchingTrajectory(*move_group, ROBOT_NAME, is_diff_plan_req, 0, 0);

  check_and_emit(fetched_trajectories.size() == 2, prefix, "Fetch all returns two");
  check_and_emit(fetched_traj != nullptr, prefix, "Fetch best trajectory is not nullptr");

  check_and_emit(*fetched_trajectories.at(0) == *fetched_traj, prefix, "Fetched trajectory on both fetches match");

  check_and_emit(*fetched_traj == traj, prefix, "Fetched trajectory matches original");

  check_and_emit(fetched_traj->lookupDouble("execution_time_s") == execution_time, prefix,
                 "Fetched trajectory has correct execution time");

  check_and_emit(fetched_traj->lookupDouble("planning_time_s") == planning_time, prefix,
                 "Fetched trajectory has correct planning time");

  // Fetch non-matching, out of tolerance
  //
  // Non-matching key should not have cache hit
  prefix = "test_motion_trajectories.put_second.fetch_non_matching_out_of_tolerance";

  fetched_trajectories = cache->fetchAllMatchingTrajectories(*move_group, ROBOT_NAME, close_matching_plan_req, 0, 0);

  fetched_traj = cache->fetchBestMatchingTrajectory(*move_group, ROBOT_NAME, close_matching_plan_req, 0, 0);

  check_and_emit(fetched_trajectories.size() == 0, prefix, "Fetch all returns empty");
  check_and_emit(fetched_traj == nullptr, prefix, "Fetch best trajectory is nullptr");

  // Fetch non-matching, only start in tolerance (but not goal)
  //
  // Non-matching key should not have cache hit
  prefix = "test_motion_trajectories.put_second.fetch_non_matching_only_start_in_tolerance";

  fetched_trajectories = cache->fetchAllMatchingTrajectories(*move_group, ROBOT_NAME, close_matching_plan_req, 999, 0);

  fetched_traj = cache->fetchBestMatchingTrajectory(*move_group, ROBOT_NAME, close_matching_plan_req, 999, 0);

  check_and_emit(fetched_trajectories.size() == 0, prefix, "Fetch all returns empty");
  check_and_emit(fetched_traj == nullptr, prefix, "Fetch best trajectory is nullptr");

  // Fetch non-matching, only goal in tolerance (but not start)
  //
  // Non-matching key should not have cache hit
  prefix = "test_motion_trajectories.put_second.fetch_non_matching_only_goal_in_tolerance";

  fetched_trajectories = cache->fetchAllMatchingTrajectories(*move_group, ROBOT_NAME, close_matching_plan_req, 0, 999);

  fetched_traj = cache->fetchBestMatchingTrajectory(*move_group, ROBOT_NAME, close_matching_plan_req, 0, 999);

  check_and_emit(fetched_trajectories.size() == 0, prefix, "Fetch all returns empty");
  check_and_emit(fetched_traj == nullptr, prefix, "Fetch best trajectory is nullptr");

  // Fetch non-matching, in tolerance
  //
  // Close key within tolerance limit should have cache hit
  prefix = "test_motion_trajectories.put_second.fetch_non_matching_in_tolerance";

  fetched_trajectories =
      cache->fetchAllMatchingTrajectories(*move_group, ROBOT_NAME, close_matching_plan_req, 0.1, 0.1);

  fetched_traj = cache->fetchBestMatchingTrajectory(*move_group, ROBOT_NAME, close_matching_plan_req, 0.1, 0.1);

  check_and_emit(fetched_trajectories.size() == 2, prefix, "Fetch all returns two");
  check_and_emit(fetched_traj != nullptr, prefix, "Fetch best trajectory is not nullptr");

  check_and_emit(*fetched_trajectories.at(0) == *fetched_traj, prefix, "Fetched trajectory on both fetches match");

  check_and_emit(*fetched_traj == traj, prefix, "Fetched trajectory matches original");

  check_and_emit(fetched_traj->lookupDouble("execution_time_s") == execution_time, prefix,
                 "Fetched trajectory has correct execution time");

  check_and_emit(fetched_traj->lookupDouble("planning_time_s") == planning_time, prefix,
                 "Fetched trajectory has correct planning time");

  // Fetch swapped
  //
  // Matches should be mostly invariant to constraint ordering
  prefix = "test_motion_trajectories.put_second.fetch_swapped";

  fetched_trajectories =
      cache->fetchAllMatchingTrajectories(*move_group, ROBOT_NAME, swapped_close_matching_plan_req, 0.1, 0.1);

  fetched_traj = cache->fetchBestMatchingTrajectory(*move_group, ROBOT_NAME, swapped_close_matching_plan_req, 0.1, 0.1);

  check_and_emit(fetched_trajectories.size() == 2, prefix, "Fetch all returns two");
  check_and_emit(fetched_traj != nullptr, prefix, "Fetch best trajectory is not nullptr");

  check_and_emit(*fetched_trajectories.at(0) == *fetched_traj, prefix, "Fetched trajectory on both fetches match");

  check_and_emit(*fetched_traj == traj, prefix, "Fetched trajectory matches original");

  check_and_emit(fetched_traj->lookupDouble("execution_time_s") == execution_time, prefix,
                 "Fetched trajectory has correct execution time");

  check_and_emit(fetched_traj->lookupDouble("planning_time_s") == planning_time, prefix,
                 "Fetched trajectory has correct planning time");

  // Fetch with smaller workspace
  //
  // Matching trajectories with more restrictive workspace requirements should not
  // pull up trajectories cached for less restrictive workspace requirements
  prefix = "test_motion_trajectories.put_second.fetch_smaller_workspace";

  fetched_trajectories =
      cache->fetchAllMatchingTrajectories(*move_group, ROBOT_NAME, smaller_workspace_plan_req, 999, 999);

  fetched_traj = cache->fetchBestMatchingTrajectory(*move_group, ROBOT_NAME, smaller_workspace_plan_req, 999, 999);

  check_and_emit(fetched_trajectories.size() == 0, prefix, "Fetch all returns empty");
  check_and_emit(fetched_traj == nullptr, prefix, "Fetch best trajectory is nullptr");

  // Fetch with larger workspace
  //
  // Matching trajectories with less restrictive workspace requirements should pull up
  // trajectories cached for more restrictive workspace requirements
  prefix = "test_motion_trajectories.put_second.fetch_larger_workspace";

  fetched_trajectories =
      cache->fetchAllMatchingTrajectories(*move_group, ROBOT_NAME, larger_workspace_plan_req, 999, 999);

  fetched_traj = cache->fetchBestMatchingTrajectory(*move_group, ROBOT_NAME, larger_workspace_plan_req, 999, 999);

  check_and_emit(fetched_trajectories.size() == 2, prefix, "Fetch all returns two");
  check_and_emit(fetched_traj != nullptr, prefix, "Fetch best trajectory is not nullptr");

  check_and_emit(*fetched_trajectories.at(0) == *fetched_traj, prefix, "Fetched trajectory on both fetches match");

  check_and_emit(*fetched_traj == traj, prefix, "Fetched trajectory matches original");

  check_and_emit(fetched_traj->lookupDouble("execution_time_s") == execution_time, prefix,
                 "Fetched trajectory has correct execution time");

  check_and_emit(fetched_traj->lookupDouble("planning_time_s") == planning_time, prefix,
                 "Fetched trajectory has correct planning time");

  check_and_emit(fetched_traj->lookupDouble("workspace_parameters.max_corner.x") <=
                     larger_workspace_plan_req.workspace_parameters.max_corner.x,
                 prefix, "Fetched trajectory has more restrictive workspace max_corner");

  check_and_emit(fetched_traj->lookupDouble("workspace_parameters.max_corner.x") >=
                     larger_workspace_plan_req.workspace_parameters.min_corner.x,
                 prefix, "Fetched trajectory has more restrictive workspace min_corner");

  // Put worse, no delete_worse_trajectories
  //
  // Worse trajectories should not be inserted
  prefix = "test_motion_trajectories.put_worse";
  double worse_execution_time = execution_time + 100;

  check_and_emit(!cache->putTrajectory(*move_group, ROBOT_NAME, plan_req, traj, worse_execution_time, planning_time,
                                       false),
                 prefix, "Put worse trajectory, no delete_worse_trajectories, not ok");

  check_and_emit(cache->countTrajectories(ROBOT_NAME) == 2, prefix, "Two trajectories in cache");

  // Put better, no delete_worse_trajectories
  //
  // Better trajectories should be inserted
  prefix = "test_motion_trajectories.put_better";
  double better_execution_time = execution_time - 100;

  check_and_emit(cache->putTrajectory(*move_group, ROBOT_NAME, plan_req, traj, better_execution_time, planning_time,
                                      false),
                 prefix, "Put better trajectory, no delete_worse_trajectories, ok");

  check_and_emit(cache->countTrajectories(ROBOT_NAME) == 3, prefix, "Three trajectories in cache");

  // Fetch sorted
  //
  // With multiple trajectories in cache, fetches should be sorted accordingly
  prefix = "test_motion_trajectories.put_better.fetch_sorted";

  fetched_trajectories = cache->fetchAllMatchingTrajectories(*move_group, ROBOT_NAME, plan_req, 0, 0);

  fetched_traj = cache->fetchBestMatchingTrajectory(*move_group, ROBOT_NAME, plan_req, 0, 0);

  check_and_emit(fetched_trajectories.size() == 3, prefix, "Fetch all returns three");
  check_and_emit(fetched_traj != nullptr, prefix, "Fetch best trajectory is not nullptr");

  check_and_emit(*fetched_trajectories.at(0) == *fetched_traj, prefix, "Fetched trajectory on both fetches match");

  check_and_emit(*fetched_traj == traj, prefix, "Fetched trajectory matches original");

  check_and_emit(fetched_traj->lookupDouble("execution_time_s") == better_execution_time, prefix,
                 "Fetched trajectory has correct execution time");

  check_and_emit(fetched_traj->lookupDouble("planning_time_s") == planning_time, prefix,
                 "Fetched trajectory has correct planning time");

  check_and_emit(fetched_trajectories.at(0)->lookupDouble("execution_time_s") == better_execution_time &&
                     fetched_trajectories.at(1)->lookupDouble("execution_time_s") == execution_time,
                 prefix, "Fetched trajectories are sorted correctly");

  // Put better, delete_worse_trajectories
  //
  // Better, different, trajectories should be inserted
  prefix = "test_motion_trajectories.put_better_delete_worse_trajectories";
  double even_better_execution_time = better_execution_time - 100;

  check_and_emit(cache->putTrajectory(*move_group, ROBOT_NAME, plan_req, different_traj, even_better_execution_time,
                                      planning_time, true),
                 prefix, "Put better trajectory, delete_worse_trajectories, ok");

  check_and_emit(cache->countTrajectories(ROBOT_NAME) == 1, prefix, "One trajectory in cache");

  // Fetch better plan
  prefix = "test_motion_trajectories.put_better_delete_worse_trajectories.fetch";

  fetched_trajectories = cache->fetchAllMatchingTrajectories(*move_group, ROBOT_NAME, plan_req, 0, 0);

  fetched_traj = cache->fetchBestMatchingTrajectory(*move_group, ROBOT_NAME, plan_req, 0, 0);

  check_and_emit(fetched_trajectories.size() == 1, prefix, "Fetch all returns one");
  check_and_emit(fetched_traj != nullptr, prefix, "Fetch best trajectory is not nullptr");

  check_and_emit(*fetched_trajectories.at(0) == *fetched_traj, prefix, "Fetched trajectory on both fetches match");

  check_and_emit(*fetched_traj == different_traj, prefix, "Fetched trajectory matches original");

  check_and_emit(fetched_traj->lookupDouble("execution_time_s") == even_better_execution_time, prefix,
                 "Fetched trajectory has correct execution time");

  check_and_emit(fetched_traj->lookupDouble("planning_time_s") == planning_time, prefix,
                 "Fetched trajectory has correct planning time");

  // Put different req, delete_worse_trajectories
  //
  // Unrelated trajectory requests should live alongside pre-existing plans
  prefix = "test_motion_trajectories.put_different_req";

  check_and_emit(cache->putTrajectory(*move_group, ROBOT_NAME, different_plan_req, different_traj,
                                      better_execution_time, planning_time, true),
                 prefix, "Put different trajectory req, delete_worse_trajectories, ok");

  check_and_emit(cache->countTrajectories(ROBOT_NAME) == 2, prefix, "Two trajectories in cache");

  // Fetch with different trajectory req
  //
  // With multiple trajectories in cache, fetches should be sorted accordingly
  prefix = "test_motion_trajectories.put_different_req.fetch";

  fetched_trajectories = cache->fetchAllMatchingTrajectories(*move_group, ROBOT_NAME, different_plan_req, 0, 0);

  fetched_traj = cache->fetchBestMatchingTrajectory(*move_group, ROBOT_NAME, different_plan_req, 0, 0);

  check_and_emit(fetched_trajectories.size() == 1, prefix, "Fetch all returns one");
  check_and_emit(fetched_traj != nullptr, prefix, "Fetch best trajectory is not nullptr");

  check_and_emit(*fetched_trajectories.at(0) == *fetched_traj, prefix, "Fetched trajectory on both fetches match");

  check_and_emit(*fetched_traj == different_traj, prefix, "Fetched trajectory matches original");

  check_and_emit(fetched_traj->lookupDouble("execution_time_s") == better_execution_time, prefix,
                 "Fetched trajectory has correct execution time");

  check_and_emit(fetched_traj->lookupDouble("planning_time_s") == planning_time, prefix,
                 "Fetched trajectory has correct planning time");

  // Fetch different robot
  //
  // Since we didn't populate anything, we should expect empty
  prefix = "test_motion_trajectories.different_robot.empty";
  std::string different_robot_name = "different_robot";

  check_and_emit(cache->countCartesianTrajectories(different_robot_name) == 0, prefix, "No trajectories in cache");

  // Put first for different robot, delete_worse_trajectories
  //
  // A different robot's cache should not interact with the original cache
  prefix = "test_motion_trajectories.different_robot.put_first";
  check_and_emit(cache->putTrajectory(*move_group, different_robot_name, different_plan_req, different_traj,
                                      better_execution_time, planning_time, true),
                 prefix, "Put different trajectory req, delete_worse_trajectories, ok");

  check_and_emit(cache->countTrajectories(different_robot_name) == 1, prefix, "One trajectory in cache");

  check_and_emit(cache->countTrajectories(ROBOT_NAME) == 2, prefix, "Two trajectories in original robot's cache");

  fetched_trajectories = cache->fetchAllMatchingTrajectories(*move_group, ROBOT_NAME, different_plan_req, 0, 0);

  check_and_emit(fetched_trajectories.size() == 1, prefix, "Fetch all on original still returns one");
}

void test_cartesian_trajectories(std::shared_ptr<MoveGroupInterface> move_group, std::shared_ptr<TrajectoryCache> cache)
{
  std::string prefix;

  /// First, test if construction even works...

  // Construct get cartesian trajectory request
  prefix = "test_cartesian_trajectories.constructGetCartesianPathRequest";

  int test_step = 1;
  int test_jump = 2;
  auto test_waypoints = get_dummy_waypoints();
  auto cartesian_plan_req_under_test =
      cache->constructGetCartesianPathRequest(*move_group, test_waypoints, test_step, test_jump, false);

  check_and_emit(cartesian_plan_req_under_test.waypoints == test_waypoints &&
                     static_cast<int>(cartesian_plan_req_under_test.max_step) == test_step &&
                     static_cast<int>(cartesian_plan_req_under_test.jump_threshold) == test_jump &&
                     !cartesian_plan_req_under_test.avoid_collisions,
                 prefix, "Ok");

  // Setup =====================================================================
  // All variants are modified copies of `cartesian_plan_req`.

  /// GetCartesianPath::Request

  // Plain start
  auto waypoints = get_dummy_waypoints();
  auto cartesian_plan_req = cache->constructGetCartesianPathRequest(*move_group, waypoints, 1, 1, false);
  cartesian_plan_req.start_state.multi_dof_joint_state.joint_names.clear();
  cartesian_plan_req.start_state.multi_dof_joint_state.transforms.clear();
  cartesian_plan_req.start_state.multi_dof_joint_state.twist.clear();
  cartesian_plan_req.start_state.multi_dof_joint_state.wrench.clear();
  cartesian_plan_req.path_constraints.joint_constraints.clear();
  cartesian_plan_req.path_constraints.position_constraints.clear();
  cartesian_plan_req.path_constraints.orientation_constraints.clear();
  cartesian_plan_req.path_constraints.visibility_constraints.clear();

  // Empty frame start
  auto empty_frame_cartesian_plan_req = cartesian_plan_req;
  empty_frame_cartesian_plan_req.header.frame_id = "";

  // is_diff = true
  auto is_diff_cartesian_plan_req = cartesian_plan_req;
  is_diff_cartesian_plan_req.start_state.is_diff = true;
  is_diff_cartesian_plan_req.start_state.joint_state.header.frame_id = "";
  is_diff_cartesian_plan_req.start_state.joint_state.name.clear();
  is_diff_cartesian_plan_req.start_state.joint_state.position.clear();
  is_diff_cartesian_plan_req.start_state.joint_state.velocity.clear();
  is_diff_cartesian_plan_req.start_state.joint_state.effort.clear();

  // Something close enough (mod 0.1 away)
  auto close_matching_cartesian_plan_req = cartesian_plan_req;
  close_matching_cartesian_plan_req.start_state.joint_state.position.at(0) -= 0.05;
  close_matching_cartesian_plan_req.start_state.joint_state.position.at(1) += 0.05;
  close_matching_cartesian_plan_req.start_state.joint_state.position.at(2) -= 0.05;
  close_matching_cartesian_plan_req.waypoints.at(0).position.x -= 0.05;
  close_matching_cartesian_plan_req.waypoints.at(1).position.x += 0.05;
  close_matching_cartesian_plan_req.waypoints.at(2).position.x -= 0.05;

  // Different
  auto different_cartesian_plan_req = cartesian_plan_req;
  different_cartesian_plan_req.start_state.joint_state.position.at(0) -= 1.05;
  different_cartesian_plan_req.start_state.joint_state.position.at(1) += 2.05;
  different_cartesian_plan_req.start_state.joint_state.position.at(2) -= 3.05;
  different_cartesian_plan_req.waypoints.at(0).position.x -= 1.05;
  different_cartesian_plan_req.waypoints.at(1).position.x += 2.05;
  different_cartesian_plan_req.waypoints.at(2).position.x -= 3.05;

  /// RobotTrajectory

  // Trajectory
  auto traj = get_dummy_panda_traj();

  // Trajectory with no frame_id in its trajectory header
  auto empty_frame_traj = traj;
  empty_frame_traj.joint_trajectory.header.frame_id = "";

  auto different_traj = traj;
  different_traj.joint_trajectory.points.at(0).positions.at(0) = 999;
  different_traj.joint_trajectory.points.at(0).positions.at(1) = 999;
  different_traj.joint_trajectory.points.at(0).positions.at(2) = 999;

  // Checks ====================================================================

  // Initially empty
  prefix = "test_cartesian_trajectories.empty";

  check_and_emit(cache->countCartesianTrajectories(ROBOT_NAME) == 0, prefix, "Trajectory cache initially empty");

  check_and_emit(
      cache->fetchAllMatchingCartesianTrajectories(*move_group, ROBOT_NAME, cartesian_plan_req, 0, 999, 999).empty(),
      prefix, "Fetch all trajectories on empty cache returns empty");

  check_and_emit(cache->fetchBestMatchingCartesianTrajectory(*move_group, ROBOT_NAME, cartesian_plan_req, 0, 999,
                                                             999) == nullptr,
                 prefix, "Fetch best trajectory on empty cache returns nullptr");

  // Put trajectory empty frame
  //
  // Trajectory must have frame in joint trajectory, expect put fail
  prefix = "test_cartesian_trajectories.putTrajectory_empty_frame";
  double execution_time = 999;
  double planning_time = 999;
  double fraction = 0.5;

  check_and_emit(!cache->putCartesianTrajectory(*move_group, ROBOT_NAME, cartesian_plan_req, empty_frame_traj,
                                                execution_time, planning_time, fraction, false),
                 prefix, "Put empty frame trajectory, no delete_worse_trajectories, not ok");

  check_and_emit(cache->countCartesianTrajectories(ROBOT_NAME) == 0, prefix, "No trajectories in cache");

  // Put trajectory req empty frame
  //
  // Trajectory request having no frame in workspace should default to robot frame
  prefix = "test_cartesian_trajectories.putTrajectory_req_empty_frame";
  execution_time = 1000;
  planning_time = 1000;

  check_and_emit(cache->putCartesianTrajectory(*move_group, ROBOT_NAME, empty_frame_cartesian_plan_req, traj,
                                               execution_time, planning_time, fraction, false),
                 prefix, "Put empty frame req trajectory, no delete_worse_trajectories, ok");

  check_and_emit(cache->countCartesianTrajectories(ROBOT_NAME) == 1, prefix, "One trajectory in cache");

  // Put second, no delete_worse_trajectories
  prefix = "test_cartesian_trajectories.put_second";
  execution_time = 999;
  planning_time = 999;

  check_and_emit(cache->putCartesianTrajectory(*move_group, ROBOT_NAME, cartesian_plan_req, traj, execution_time,
                                               planning_time, fraction, false),
                 prefix, "Put second valid trajectory, no delete_worse_trajectories, ok");

  check_and_emit(cache->countCartesianTrajectories(ROBOT_NAME) == 2, prefix, "Two trajectories in cache");

  // Fetch matching, no tolerance
  //
  // Exact key match should have cache hit
  prefix = "test_cartesian_trajectories.put_second.fetch_matching_no_tolerance";

  auto fetched_trajectories =
      cache->fetchAllMatchingCartesianTrajectories(*move_group, ROBOT_NAME, cartesian_plan_req, fraction, 0, 0);

  auto fetched_traj =
      cache->fetchBestMatchingCartesianTrajectory(*move_group, ROBOT_NAME, cartesian_plan_req, fraction, 0, 0);

  check_and_emit(fetched_trajectories.size() == 2, prefix, "Fetch all returns two");
  check_and_emit(fetched_traj != nullptr, prefix, "Fetch best trajectory is not nullptr");

  check_and_emit(*fetched_trajectories.at(0) == *fetched_traj, prefix, "Fetched trajectory on both fetches match");

  check_and_emit(*fetched_traj == traj, prefix, "Fetched trajectory matches original");

  check_and_emit(fetched_traj->lookupDouble("execution_time_s") == execution_time, prefix,
                 "Fetched trajectory has correct execution time");

  check_and_emit(fetched_traj->lookupDouble("planning_time_s") == planning_time, prefix,
                 "Fetched trajectory has correct planning time");

  check_and_emit(fetched_traj->lookupDouble("fraction") == fraction, prefix, "Fetched trajectory has correct fraction");

  // Fetch with is_diff
  //
  // is_diff key should be equivalent to exact match if robot state did not
  // change, hence should have cache hit
  prefix = "test_cartesian_trajectories.put_second.fetch_is_diff_no_tolerance";

  fetched_trajectories =
      cache->fetchAllMatchingCartesianTrajectories(*move_group, ROBOT_NAME, is_diff_cartesian_plan_req, fraction, 0, 0);

  fetched_traj =
      cache->fetchBestMatchingCartesianTrajectory(*move_group, ROBOT_NAME, is_diff_cartesian_plan_req, fraction, 0, 0);

  check_and_emit(fetched_trajectories.size() == 2, prefix, "Fetch all returns two");
  check_and_emit(fetched_traj != nullptr, prefix, "Fetch best trajectory is not nullptr");

  check_and_emit(*fetched_trajectories.at(0) == *fetched_traj, prefix, "Fetched trajectory on both fetches match");

  check_and_emit(*fetched_traj == traj, prefix, "Fetched trajectory matches original");

  check_and_emit(fetched_traj->lookupDouble("execution_time_s") == execution_time, prefix,
                 "Fetched trajectory has correct execution time");

  check_and_emit(fetched_traj->lookupDouble("planning_time_s") == planning_time, prefix,
                 "Fetched trajectory has correct planning time");

  check_and_emit(fetched_traj->lookupDouble("fraction") == fraction, prefix, "Fetched trajectory has correct fraction");

  // Fetch non-matching, out of tolerance
  //
  // Non-matching key should not have cache hit
  prefix = "test_cartesian_trajectories.put_second.fetch_non_matching_out_of_tolerance";

  fetched_trajectories = cache->fetchAllMatchingCartesianTrajectories(
      *move_group, ROBOT_NAME, close_matching_cartesian_plan_req, fraction, 0, 0);

  fetched_traj = cache->fetchBestMatchingCartesianTrajectory(*move_group, ROBOT_NAME, close_matching_cartesian_plan_req,
                                                             fraction, 0, 0);

  check_and_emit(fetched_trajectories.size() == 0, prefix, "Fetch all returns empty");
  check_and_emit(fetched_traj == nullptr, prefix, "Fetch best trajectory is nullptr");

  // Fetch non-matching, only start in tolerance (but not goal)
  //
  // Non-matching key should not have cache hit
  prefix = "test_motion_trajectories.put_second.fetch_non_matching_only_start_in_tolerance";

  fetched_trajectories = cache->fetchAllMatchingCartesianTrajectories(
      *move_group, ROBOT_NAME, close_matching_cartesian_plan_req, fraction, 999, 0);

  fetched_traj = cache->fetchBestMatchingCartesianTrajectory(*move_group, ROBOT_NAME, close_matching_cartesian_plan_req,
                                                             fraction, 999, 0);

  check_and_emit(fetched_trajectories.size() == 0, prefix, "Fetch all returns empty");
  check_and_emit(fetched_traj == nullptr, prefix, "Fetch best trajectory is nullptr");

  // Fetch non-matching, only goal in tolerance (but not start)
  //
  // Non-matching key should not have cache hit
  prefix = "test_motion_trajectories.put_second.fetch_non_matching_only_goal_in_tolerance";

  fetched_trajectories = cache->fetchAllMatchingCartesianTrajectories(
      *move_group, ROBOT_NAME, close_matching_cartesian_plan_req, fraction, 0, 999);

  fetched_traj = cache->fetchBestMatchingCartesianTrajectory(*move_group, ROBOT_NAME, close_matching_cartesian_plan_req,
                                                             fraction, 0, 999);

  check_and_emit(fetched_trajectories.size() == 0, prefix, "Fetch all returns empty");
  check_and_emit(fetched_traj == nullptr, prefix, "Fetch best trajectory is nullptr");

  // Fetch non-matching, in tolerance
  //
  // Close key within tolerance limit should have cache hit
  prefix = "test_cartesian_trajectories.put_second.fetch_non_matching_in_tolerance";

  fetched_trajectories = cache->fetchAllMatchingCartesianTrajectories(
      *move_group, ROBOT_NAME, close_matching_cartesian_plan_req, fraction, 0.1, 0.1);

  fetched_traj = cache->fetchBestMatchingCartesianTrajectory(*move_group, ROBOT_NAME, close_matching_cartesian_plan_req,
                                                             fraction, 0.1, 0.1);

  check_and_emit(fetched_trajectories.size() == 2, prefix, "Fetch all returns two");
  check_and_emit(fetched_traj != nullptr, prefix, "Fetch best trajectory is not nullptr");

  check_and_emit(*fetched_trajectories.at(0) == *fetched_traj, prefix, "Fetched trajectory on both fetches match");

  check_and_emit(*fetched_traj == traj, prefix, "Fetched trajectory matches original");

  check_and_emit(fetched_traj->lookupDouble("execution_time_s") == execution_time, prefix,
                 "Fetched trajectory has correct execution time");

  check_and_emit(fetched_traj->lookupDouble("planning_time_s") == planning_time, prefix,
                 "Fetched trajectory has correct planning time");

  check_and_emit(fetched_traj->lookupDouble("fraction") == fraction, prefix, "Fetched trajectory has correct fraction");

  // Fetch with higher fraction
  //
  // Matching trajectories with more restrictive fraction requirements should not
  // pull up trajectories cached for less restrictive fraction requirements
  prefix = "test_cartesian_trajectories.put_second.fetch_higher_fraction";

  fetched_trajectories =
      cache->fetchAllMatchingCartesianTrajectories(*move_group, ROBOT_NAME, cartesian_plan_req, 1, 999, 999);

  fetched_traj = cache->fetchBestMatchingCartesianTrajectory(*move_group, ROBOT_NAME, cartesian_plan_req, 1, 999, 999);

  check_and_emit(fetched_trajectories.size() == 0, prefix, "Fetch all returns empty");
  check_and_emit(fetched_traj == nullptr, prefix, "Fetch best trajectory is nullptr");

  // Fetch with lower fraction
  //
  // Matching trajectories with less restrictive fraction requirements should pull up
  // trajectories cached for more restrictive fraction requirements
  prefix = "test_cartesian_trajectories.put_second.fetch_lower_fraction";

  fetched_trajectories =
      cache->fetchAllMatchingCartesianTrajectories(*move_group, ROBOT_NAME, cartesian_plan_req, 0, 999, 999);

  fetched_traj = cache->fetchBestMatchingCartesianTrajectory(*move_group, ROBOT_NAME, cartesian_plan_req, 0, 999, 999);

  check_and_emit(fetched_trajectories.size() == 2, prefix, "Fetch all returns two");
  check_and_emit(fetched_traj != nullptr, prefix, "Fetch best trajectory is not nullptr");

  check_and_emit(*fetched_trajectories.at(0) == *fetched_traj, prefix, "Fetched trajectory on both fetches match");

  check_and_emit(*fetched_traj == traj, prefix, "Fetched trajectory matches original");

  check_and_emit(fetched_traj->lookupDouble("execution_time_s") == execution_time, prefix,
                 "Fetched trajectory has correct execution time");

  check_and_emit(fetched_traj->lookupDouble("planning_time_s") == planning_time, prefix,
                 "Fetched trajectory has correct planning time");

  check_and_emit(fetched_traj->lookupDouble("fraction") == fraction, prefix, "Fetched trajectory has correct fraction");

  // Put worse, no delete_worse_trajectories
  //
  // Worse trajectories should not be inserted
  prefix = "test_cartesian_trajectories.put_worse";
  double worse_execution_time = execution_time + 100;

  check_and_emit(!cache->putCartesianTrajectory(*move_group, ROBOT_NAME, cartesian_plan_req, traj, worse_execution_time,
                                                planning_time, fraction, false),
                 prefix, "Put worse trajectory, no delete_worse_trajectories, not ok");

  check_and_emit(cache->countCartesianTrajectories(ROBOT_NAME) == 2, prefix, "Two trajectories in cache");

  // Put better, no delete_worse_trajectories
  //
  // Better trajectories should be inserted
  prefix = "test_cartesian_trajectories.put_better";
  double better_execution_time = execution_time - 100;

  check_and_emit(cache->putCartesianTrajectory(*move_group, ROBOT_NAME, cartesian_plan_req, traj, better_execution_time,
                                               planning_time, fraction, false),
                 prefix, "Put better trajectory, no delete_worse_trajectories, ok");

  check_and_emit(cache->countCartesianTrajectories(ROBOT_NAME) == 3, prefix, "Three trajectories in cache");

  // Fetch sorted
  //
  // With multiple trajectories in cache, fetches should be sorted accordingly
  prefix = "test_cartesian_trajectories.put_better.fetch_sorted";

  fetched_trajectories =
      cache->fetchAllMatchingCartesianTrajectories(*move_group, ROBOT_NAME, cartesian_plan_req, fraction, 0, 0);

  fetched_traj =
      cache->fetchBestMatchingCartesianTrajectory(*move_group, ROBOT_NAME, cartesian_plan_req, fraction, 0, 0);

  check_and_emit(fetched_trajectories.size() == 3, prefix, "Fetch all returns three");
  check_and_emit(fetched_traj != nullptr, prefix, "Fetch best trajectory is not nullptr");

  check_and_emit(*fetched_trajectories.at(0) == *fetched_traj, prefix, "Fetched trajectory on both fetches match");

  check_and_emit(*fetched_traj == traj, prefix, "Fetched trajectory matches original");

  check_and_emit(fetched_traj->lookupDouble("execution_time_s") == better_execution_time, prefix,
                 "Fetched trajectory has correct execution time");

  check_and_emit(fetched_traj->lookupDouble("planning_time_s") == planning_time, prefix,
                 "Fetched trajectory has correct planning time");

  check_and_emit(fetched_traj->lookupDouble("fraction") == fraction, prefix, "Fetched trajectory has correct fraction");

  check_and_emit(fetched_trajectories.at(0)->lookupDouble("execution_time_s") == better_execution_time &&
                     fetched_trajectories.at(1)->lookupDouble("execution_time_s") == execution_time,
                 prefix, "Fetched trajectories are sorted correctly");

  // Put better, delete_worse_trajectories
  //
  // Better, different, trajectories should be inserted
  prefix = "test_cartesian_trajectories.put_better_delete_worse_trajectories";
  double even_better_execution_time = better_execution_time - 100;

  check_and_emit(cache->putCartesianTrajectory(*move_group, ROBOT_NAME, cartesian_plan_req, different_traj,
                                               even_better_execution_time, planning_time, fraction, true),
                 prefix, "Put better trajectory, delete_worse_trajectories, ok");

  check_and_emit(cache->countCartesianTrajectories(ROBOT_NAME) == 1, prefix, "One trajectory in cache");

  // Fetch better plan
  prefix = "test_cartesian_trajectories.put_better_delete_worse_trajectories.fetch";

  fetched_trajectories =
      cache->fetchAllMatchingCartesianTrajectories(*move_group, ROBOT_NAME, cartesian_plan_req, fraction, 0, 0);

  fetched_traj =
      cache->fetchBestMatchingCartesianTrajectory(*move_group, ROBOT_NAME, cartesian_plan_req, fraction, 0, 0);

  check_and_emit(fetched_trajectories.size() == 1, prefix, "Fetch all returns one");
  check_and_emit(fetched_traj != nullptr, prefix, "Fetch best trajectory is not nullptr");

  check_and_emit(*fetched_trajectories.at(0) == *fetched_traj, prefix, "Fetched trajectory on both fetches match");

  check_and_emit(*fetched_traj == different_traj, prefix, "Fetched trajectory matches original");

  check_and_emit(fetched_traj->lookupDouble("execution_time_s") == even_better_execution_time, prefix,
                 "Fetched trajectory has correct execution time");

  check_and_emit(fetched_traj->lookupDouble("planning_time_s") == planning_time, prefix,
                 "Fetched trajectory has correct planning time");

  check_and_emit(fetched_traj->lookupDouble("fraction") == fraction, prefix, "Fetched trajectory has correct fraction");

  // Put different req, delete_worse_trajectories
  //
  // Unrelated trajectory requests should live alongside pre-existing plans
  prefix = "test_cartesian_trajectories.put_different_req";

  check_and_emit(cache->putCartesianTrajectory(*move_group, ROBOT_NAME, different_cartesian_plan_req, different_traj,
                                               better_execution_time, planning_time, fraction, true),
                 prefix, "Put different trajectory req, delete_worse_trajectories, ok");

  check_and_emit(cache->countCartesianTrajectories(ROBOT_NAME) == 2, prefix, "Two trajectories in cache");

  // Fetch with different trajectory req
  //
  // With multiple trajectories in cache, fetches should be sorted accordingly
  prefix = "test_cartesian_trajectories.put_different_req.fetch";

  fetched_trajectories = cache->fetchAllMatchingCartesianTrajectories(*move_group, ROBOT_NAME,
                                                                      different_cartesian_plan_req, fraction, 0, 0);

  fetched_traj = cache->fetchBestMatchingCartesianTrajectory(*move_group, ROBOT_NAME, different_cartesian_plan_req,
                                                             fraction, 0, 0);

  check_and_emit(fetched_trajectories.size() == 1, prefix, "Fetch all returns one");
  check_and_emit(fetched_traj != nullptr, prefix, "Fetch best trajectory is not nullptr");

  check_and_emit(*fetched_trajectories.at(0) == *fetched_traj, prefix, "Fetched trajectory on both fetches match");

  check_and_emit(*fetched_traj == different_traj, prefix, "Fetched trajectory matches original");

  check_and_emit(fetched_traj->lookupDouble("execution_time_s") == better_execution_time, prefix,
                 "Fetched trajectory has correct execution time");

  check_and_emit(fetched_traj->lookupDouble("planning_time_s") == planning_time, prefix,
                 "Fetched trajectory has correct planning time");

  check_and_emit(fetched_traj->lookupDouble("fraction") == fraction, prefix, "Fetched trajectory has correct fraction");

  // Fetch different robot
  //
  // Since we didn't populate anything, we should expect empty
  prefix = "test_cartesian_trajectories.different_robot.empty";
  std::string different_robot_name = "different_robot";

  check_and_emit(cache->countCartesianTrajectories(different_robot_name) == 0, prefix, "No trajectories in cache");

  // Put first for different robot, delete_worse_trajectories
  //
  // A different robot's cache should not interact with the original cache
  prefix = "test_cartesian_trajectories.different_robot.put_first";
  check_and_emit(cache->putCartesianTrajectory(*move_group, different_robot_name, different_cartesian_plan_req,
                                               different_traj, better_execution_time, planning_time, fraction, true),
                 prefix, "Put different trajectory req, delete_worse_trajectories, ok");

  check_and_emit(cache->countCartesianTrajectories(different_robot_name) == 1, prefix, "One trajectory in cache");

  check_and_emit(cache->countCartesianTrajectories(ROBOT_NAME) == 2, prefix,
                 "Two trajectories in original robot's cache");

  fetched_trajectories = cache->fetchAllMatchingCartesianTrajectories(*move_group, ROBOT_NAME,
                                                                      different_cartesian_plan_req, fraction, 0, 0);

  check_and_emit(fetched_trajectories.size() == 1, prefix, "Fetch all on original still returns one");
}

int main(int argc, char** argv)
{
  // Setup
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions test_node_options;
  test_node_options.automatically_declare_parameters_from_overrides(true);
  test_node_options.arguments({ "--ros-args", "-r", "__node:=test_node" });

  rclcpp::NodeOptions move_group_node_options;
  move_group_node_options.automatically_declare_parameters_from_overrides(true);
  move_group_node_options.arguments({ "--ros-args", "-r", "__node:=move_group_node" });

  auto test_node = rclcpp::Node::make_shared("test_node", test_node_options);
  auto move_group_node = rclcpp::Node::make_shared("move_group_node", move_group_node_options);

  std::atomic<bool> running = true;

  std::thread spin_thread([&]() {
    while (rclcpp::ok() && running)
    {
      rclcpp::spin_some(test_node);
      rclcpp::spin_some(move_group_node);
    }
  });

  // This is necessary
  test_node->declare_parameter<std::string>("warehouse_plugin", "warehouse_ros_sqlite::DatabaseConnection");

  // Test proper
  {
    auto cache = std::make_shared<TrajectoryCache>(test_node);
    check_and_emit(cache->init(":memory:", 0, 1e-4), "init", "Cache init");

    auto move_group = std::make_shared<MoveGroupInterface>(move_group_node, "panda_arm");

    auto curr_state = move_group->getCurrentState(60);
    move_group->setStartState(*curr_state);

    test_motion_trajectories(move_group, cache);
    test_cartesian_trajectories(move_group, cache);
  }

  running = false;
  spin_thread.join();

  test_node.reset();
  move_group_node.reset();

  rclcpp::shutdown();
  return 0;
}
