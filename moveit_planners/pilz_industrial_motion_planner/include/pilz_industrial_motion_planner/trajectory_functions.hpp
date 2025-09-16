/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
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
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
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

#pragma once

#include <Eigen/Geometry>
#include <kdl/trajectory.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
// TODO: Remove conditional include when released to all active distros.
#if __has_include(<tf2/transform_datatypes.hpp>)
#include <tf2/transform_datatypes.hpp>
#else
#include <tf2/transform_datatypes.h>
#endif
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <moveit/planning_scene/planning_scene.hpp>

#include <pilz_industrial_motion_planner/cartesian_trajectory.hpp>
#include <pilz_industrial_motion_planner/limits_container.hpp>
#include <pilz_industrial_motion_planner/trajectory_generation_exceptions.hpp>

namespace pilz_industrial_motion_planner
{
/**
 * @brief compute the inverse kinematics of a given pose, also check robot self
 * collision
 * @param scene: planning scene
 * @param group_name: name of planning group
 * @param link_name: name of target link
 * @param pose: target pose in IK solver Frame
 * @param frame_id: reference frame of the target pose
 * @param seed: seed state of IK solver
 * @param solution: solution of IK
 * @param check_self_collision: true to enable self collision checking after IK
 * computation
 * @param timeout: timeout for IK, if not set the default solver timeout is used
 * @return true if succeed
 */
bool computePoseIK(const planning_scene::PlanningSceneConstPtr& scene, const std::string& group_name,
                   const std::string& link_name, const Eigen::Isometry3d& pose, const std::string& frame_id,
                   const std::map<std::string, double>& seed, std::map<std::string, double>& solution,
                   bool check_self_collision = true, const double timeout = 0.0);

bool computePoseIK(const planning_scene::PlanningSceneConstPtr& scene, const std::string& group_name,
                   const std::string& link_name, const geometry_msgs::msg::Pose& pose, const std::string& frame_id,
                   const std::map<std::string, double>& seed, std::map<std::string, double>& solution,
                   bool check_self_collision = true, const double timeout = 0.0);

/**
 * @brief compute the pose of a link at a given robot state
 * @param robot_state: an arbitrary robot state (with collision objects attached)
 * @param link_name: target link name
 * @param joint_state: joint positions of this group
 * @param pose: pose of the link in base frame of robot model
 * @return true if succeed
 */
bool computeLinkFK(moveit::core::RobotState& robot_state, const std::string& link_name,
                   const std::map<std::string, double>& joint_state, Eigen::Isometry3d& pose);

bool computeLinkFK(moveit::core::RobotState& robot_state, const std::string& link_name,
                   const std::vector<std::string>& joint_names, const std::vector<double>& joint_positions,
                   Eigen::Isometry3d& pose);

/**
 * @brief verify the velocity/acceleration limits of current sample (based on
 * backward difference computation)
 * v(k) = [x(k) - x(k-1)]/[t(k) - t(k-1)]
 * a(k) = [v(k) - v(k-1)]/[t(k) - t(k-2)]*2
 * @param position_last: position of last sample
 * @param velocity_last: velocity of last sample
 * @param position_current: position of current sample
 * @param duration_last: duration of last sample
 * @param duration_current: duration of current sample
 * @param joint_limits: joint limits
 * @return
 */
bool verifySampleJointLimits(const std::map<std::string, double>& position_last,
                             const std::map<std::string, double>& velocity_last,
                             const std::map<std::string, double>& position_current, double duration_last,
                             double duration_current, const JointLimitsContainer& joint_limits);

/**
 * @brief Interpolates between two poses.
 *
 * This function computes an interpolated pose between a start pose and an end pose based on the given interpolation factor.
 *
 * @param start_pose The starting pose as an Eigen::Isometry3d.
 * @param end_pose The ending pose as an Eigen::Isometry3d.
 * @param interpolation_factor A double value between 0 and 1 that determines the interpolation factor.
 *                             A value of 0 returns the start_pose, and a value of 1 returns the end_pose.
 * @param interpolated_pose The resulting interpolated pose as an Eigen::Isometry3d.
 */
void interpolate(const Eigen::Isometry3d& start_pose, const Eigen::Isometry3d& end_pose,
                 const double& interpolation_factor, Eigen::Isometry3d& interpolated_pose);

/**
 * @brief Generate joint trajectory from a KDL Cartesian trajectory
 * @param scene: planning scene
 * @param joint_limits: joint limits
 * @param trajectory: KDL Cartesian trajectory
 * @param group_name: name of the planning group
 * @param link_name: name of the target robot link
 * @param initial_joint_position: initial joint positions, needed for selecting
 * the ik solution
 * @param sampling_time: sampling time of the generated trajectory
 * @param joint_trajectory: output as robot joint trajectory, first and last
 * point will have zero velocity
 * and acceleration
 * @param error_code: detailed error information
 * @param check_self_collision: check for self collision during creation
 * @return true if succeed
 */
bool generateJointTrajectory(const planning_scene::PlanningSceneConstPtr& scene,
                             const JointLimitsContainer& joint_limits, const KDL::Trajectory& trajectory,
                             const std::string& group_name, const std::string& link_name,
                             const std::map<std::string, double>& initial_joint_position, double sampling_time,
                             trajectory_msgs::msg::JointTrajectory& joint_trajectory,
                             moveit_msgs::msg::MoveItErrorCodes& error_code, bool check_self_collision = false);

/**
 * @brief Generate joint trajectory from a MultiDOFJointTrajectory
 * @param scene: planning scene
 * @param trajectory: Cartesian trajectory
 * @param info: motion plan information
 * @param sampling_time
 * @param joint_trajectory
 * @param error_code
 * @return true if succeed
 */
bool generateJointTrajectory(const planning_scene::PlanningSceneConstPtr& scene,
                             const JointLimitsContainer& joint_limits,
                             const pilz_industrial_motion_planner::CartesianTrajectory& trajectory,
                             const std::string& group_name, const std::string& link_name,
                             const std::map<std::string, double>& initial_joint_position,
                             const std::map<std::string, double>& initial_joint_velocity,
                             trajectory_msgs::msg::JointTrajectory& joint_trajectory,
                             moveit_msgs::msg::MoveItErrorCodes& error_code, bool check_self_collision = false);

/**
 * @brief Determines the sampling time and checks that both trajectroies use the
 * same sampling time.
 * @return TRUE if the sampling time is equal between all given points (except
 * the last two points
 * of each trajectory), otherwise FALSE.
 */
bool determineAndCheckSamplingTime(const robot_trajectory::RobotTrajectoryPtr& first_trajectory,
                                   const robot_trajectory::RobotTrajectoryPtr& second_trajectory, double EPSILON,
                                   double& sampling_time);

/**
 * @brief Check if the two robot states have the same joint
 * position/velocity/acceleration.
 *
 * @param joint_group_name The name of the joint group.
 * @param epsilon Constants defining how close the joint
 * position/velocity/acceleration have to be to be
 * recognized as equal.
 *
 * @return True if joint positions, joint velocities and joint accelerations are
 * equal, otherwise false.
 */
bool isRobotStateEqual(const moveit::core::RobotState& state1, const moveit::core::RobotState& state2,
                       const std::string& joint_group_name, double epsilon);

/**
 * @brief check if the robot state have zero velocity/acceleration
 * @param state
 * @param group
 * @param EPSILON
 * @return
 */
bool isRobotStateStationary(const moveit::core::RobotState& state, const std::string& group, double EPSILON);

/**
 * @brief Performs a linear search for the intersection point of the trajectory
 * with the blending radius.
 * @param center_position Center of blending sphere.
 * @param r Radius of blending sphere.
 * @param traj The trajectory.
 * @param inverseOrder TRUE: Farthest element from blending sphere center is
 * located at the
 * smallest index of trajectroy.
 * @param index The intersection index which has to be determined.
 */
bool linearSearchIntersectionPoint(const std::string& link_name, const Eigen::Vector3d& center_position, const double r,
                                   const robot_trajectory::RobotTrajectoryPtr& traj, bool inverseOrder,
                                   std::size_t& index);

bool intersectionFound(const Eigen::Vector3d& p_center, const Eigen::Vector3d& p_current, const Eigen::Vector3d& p_next,
                       double r);

/**
 * @brief Checks if current robot state is in self collision.
 * @param scene: planning scene.
 * @param test_for_self_collision Flag to deactivate this check during IK.
 * @param robot_model: robot kinematics model.
 * @param state Robot state instance used for .
 * @param group
 * @param ik_solution
 * @return
 */
bool isStateColliding(const planning_scene::PlanningSceneConstPtr& scene, moveit::core::RobotState* state,
                      const moveit::core::JointModelGroup* const group, const double* const ik_solution);
}  // namespace pilz_industrial_motion_planner

void normalizeQuaternion(geometry_msgs::msg::Quaternion& quat);

/**
 * @brief Adapt goal pose, defined by position+orientation, to consider offset
 * @param constraint to apply offset to
 * @param offset to apply to the constraint
 * @param orientation to apply to the offset
 * @return final goal pose
 */
Eigen::Isometry3d getConstraintPose(const geometry_msgs::msg::Point& position,
                                    const geometry_msgs::msg::Quaternion& orientation,
                                    const geometry_msgs::msg::Vector3& offset);

/**
 * @brief Conviencency method, passing args from a goal constraint
 * @param goal goal constraint
 * @return final goal pose
 */
Eigen::Isometry3d getConstraintPose(const moveit_msgs::msg::Constraints& goal);
