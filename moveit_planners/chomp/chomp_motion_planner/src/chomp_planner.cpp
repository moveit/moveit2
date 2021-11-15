/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: E. Gil Jones */

#include <chomp_motion_planner/chomp_optimizer.h>
#include <chomp_motion_planner/chomp_planner.h>
#include <chomp_motion_planner/chomp_trajectory.h>
#include <moveit/robot_state/conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <chrono>

#include "rclcpp/rclcpp.hpp"

namespace chomp
{
rclcpp::Logger LOGGER = rclcpp::get_logger("chomp_planner");

bool ChompPlanner::solve(
  const planning_scene::PlanningSceneConstPtr & planning_scene,
  const planning_interface::MotionPlanRequest & req, const ChompParameters & params,
  planning_interface::MotionPlanDetailedResponse & res) const
{
  std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();
  if (!planning_scene) {
    RCLCPP_ERROR(LOGGER, "No planning scene initialized.");
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return false;
  }

  // get the specified start state
  moveit::core::RobotState start_state = planning_scene->getCurrentState();
  moveit::core::robotStateMsgToRobotState(
    planning_scene->getTransforms(), req.start_state, start_state);

  if (!start_state.satisfiesBounds()) {
    RCLCPP_ERROR(LOGGER, "Start state violates joint limits");
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
    return false;
  }

  ChompTrajectory trajectory(planning_scene->getRobotModel(), 3.0, .03, req.group_name);
  robotStateToArray(start_state, req.group_name, trajectory.getTrajectoryPoint(0));

  if (req.goal_constraints.size() != 1) {
    RCLCPP_ERROR(
      LOGGER, "Expecting exactly one goal constraint, got: %zd", req.goal_constraints.size());
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  // Convert to joint space goal.
  planning_interface::MotionPlanRequest req_mod;
  if (
    req.goal_constraints[0].joint_constraints.empty() ||
    !req.goal_constraints[0].position_constraints.empty() ||
    !req.goal_constraints[0].orientation_constraints.empty()) {
    RCLCPP_INFO(LOGGER, "Converting work-space to joint-space goal ...");

    geometry_msgs::msg::Point p = req.goal_constraints.at(0)
                                    .position_constraints.at(0)
                                    .constraint_region.primitive_poses.at(0)
                                    .position;
    p.x -= req.goal_constraints.at(0).position_constraints.at(0).target_point_offset.x;
    p.y -= req.goal_constraints.at(0).position_constraints.at(0).target_point_offset.y;
    p.z -= req.goal_constraints.at(0).position_constraints.at(0).target_point_offset.z;

    geometry_msgs::msg::Pose pose;
    pose.position = p;
    pose.orientation = req.goal_constraints.at(0).orientation_constraints.at(0).orientation;
    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);
    pose.orientation = tf2::toMsg(q.normalize());
    Eigen::Isometry3d pose_eigen;
    tf2::fromMsg(pose, pose_eigen);
    moveit::core::RobotModelConstPtr robot_model = planning_scene->getRobotModel();

    std::map<std::string, double> start_joint_position;
    for (std::size_t i = 0; i < req.start_state.joint_state.name.size(); ++i) {
      start_joint_position[req.start_state.joint_state.name[i]] =
        req.start_state.joint_state.position[i];
    }

    std::map<std::string, double> goal_joint_position;
    if (!computePoseIK(
          robot_model, req.group_name,
          req.goal_constraints.at(0).position_constraints.at(0).link_name, pose_eigen,
          robot_model->getModelFrame(), start_joint_position, goal_joint_position)) {
      throw std::runtime_error("No IK solution for goal pose");
    }

    req_mod.group_name = req.group_name;
    req_mod.goal_constraints.resize(1);
    req_mod.goal_constraints[0].joint_constraints.resize(goal_joint_position.size());
    const std::vector<std::string> joint_names =
      robot_model->getJointModelGroup(req.group_name)->getActiveJointModelNames();
    RCLCPP_INFO(LOGGER, "The joint space goal is:");
    for (std::size_t i = 0; i < goal_joint_position.size(); ++i) {
      req_mod.goal_constraints[0].joint_constraints[i].joint_name = joint_names[i];
      req_mod.goal_constraints[0].joint_constraints[i].position =
        goal_joint_position[joint_names[i]];
      RCLCPP_INFO(
        LOGGER, "(%d) %s: %.3f", i,
        req_mod.goal_constraints[0].joint_constraints[i].joint_name.c_str(),
        req_mod.goal_constraints[0].joint_constraints[i].position);
    }
  } else {
    req_mod = req;
  }

  // if (
  //   req.goal_constraints[0].joint_constraints.empty() ||
  //   !req.goal_constraints[0].position_constraints.empty() ||
  //   !req.goal_constraints[0].orientation_constraints.empty()) {
  //   RCLCPP_ERROR(LOGGER, "Only joint-space goals are supported");
  //   res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
  //   return false;
  // }

  const size_t goal_index = trajectory.getNumPoints() - 1;
  moveit::core::RobotState goal_state(start_state);
  for (const moveit_msgs::msg::JointConstraint & joint_constraint :
       req_mod.goal_constraints[0].joint_constraints)
    goal_state.setVariablePosition(joint_constraint.joint_name, joint_constraint.position);
  if (!goal_state.satisfiesBounds()) {
    RCLCPP_ERROR(LOGGER, "Goal state violates joint limits");
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
    return false;
  }
  robotStateToArray(goal_state, req_mod.group_name, trajectory.getTrajectoryPoint(goal_index));

  const moveit::core::JointModelGroup * model_group =
    planning_scene->getRobotModel()->getJointModelGroup(req_mod.group_name);
  // fix the goal to move the shortest angular distance for wrap-around joints:
  for (size_t i = 0; i < model_group->getActiveJointModels().size(); i++) {
    const moveit::core::JointModel * model = model_group->getActiveJointModels()[i];
    const moveit::core::RevoluteJointModel * revolute_joint =
      dynamic_cast<const moveit::core::RevoluteJointModel *>(model);

    if (revolute_joint != nullptr) {
      if (revolute_joint->isContinuous()) {
        double start = (trajectory)(0, i);
        double end = (trajectory)(goal_index, i);
        RCLCPP_INFO(
          LOGGER, "Start is %s end %s short %f", start, end, shortestAngularDistance(start, end));
        (trajectory)(goal_index, i) = start + shortestAngularDistance(start, end);
      }
    }
  }

  // fill in an initial trajectory based on user choice from the chomp_config.yaml file
  if (params.trajectory_initialization_method_.compare("quintic-spline") == 0)
    trajectory.fillInMinJerk();
  else if (params.trajectory_initialization_method_.compare("linear") == 0)
    trajectory.fillInLinearInterpolation();
  else if (params.trajectory_initialization_method_.compare("cubic") == 0)
    trajectory.fillInCubicInterpolation();
  else if (params.trajectory_initialization_method_.compare("fillTrajectory") == 0) {
    if (!(trajectory.fillInFromTrajectory(*res.trajectory_[0]))) {
      RCLCPP_ERROR(
        LOGGER,
        "Input trajectory has less than 2 points, "
        "trajectory must contain at least start and goal state");
      return false;
    }
  } else {
    RCLCPP_ERROR(LOGGER, "invalid interpolation method specified in the chomp_planner file");
    return false;
  }

  RCLCPP_INFO(
    LOGGER, "CHOMP trajectory initialized using method: %s ",
    (params.trajectory_initialization_method_).c_str());

  // optimize!
  std::chrono::time_point<std::chrono::system_clock> create_time = std::chrono::system_clock::now();

  int replan_count = 0;
  bool replan_flag = false;
  double org_learning_rate = 0.04, org_ridge_factor = 0.0, org_planning_time_limit = 10;
  int org_max_iterations = 200;

  // storing the initial chomp parameters values
  org_learning_rate = params.learning_rate_;
  org_ridge_factor = params.ridge_factor_;
  org_planning_time_limit = params.planning_time_limit_;
  org_max_iterations = params.max_iterations_;

  std::unique_ptr<ChompOptimizer> optimizer;

  // create a non_const_params variable which stores the non constant version of the const params variable
  ChompParameters params_nonconst = params;

  // while loop for replanning (recovery behaviour) if collision free optimized solution not found
  while (true) {
    if (replan_flag) {
      // increase learning rate in hope to find a successful path; increase ridge factor to avoid obstacles; add 5
      // additional secs in hope to find a solution; increase maximum iterations
      params_nonconst.setRecoveryParams(
        params_nonconst.learning_rate_ + 0.02, params_nonconst.ridge_factor_ + 0.002,
        params_nonconst.planning_time_limit_ + 5, params_nonconst.max_iterations_ + 50);
    }

    // initialize a ChompOptimizer object to load up the optimizer with default parameters or with updated parameters in
    // case of a recovery behaviour
    optimizer.reset(new ChompOptimizer(
      &trajectory, planning_scene, req.group_name, &params_nonconst, start_state));
    if (!optimizer->isInitialized()) {
      RCLCPP_ERROR(LOGGER, "Could not initialize optimizer");
      res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
      return false;
    }

    RCLCPP_DEBUG(
      LOGGER, "Optimization took %f sec to create",
      (std::chrono::system_clock::now() - create_time).count());

    bool optimization_result = optimizer->optimize();

    // replan with updated parameters if no solution is found
    if (params_nonconst.enable_failure_recovery_) {
      RCLCPP_INFO(
        LOGGER,
        "Planned with Chomp Parameters (learning_rate, ridge_factor, "
        "planning_time_limit, max_iterations), attempt: # %d ",
        (replan_count + 1));
      RCLCPP_INFO(
        LOGGER, "Learning rate: %f ridge factor: %f planning time limit: %f max_iterations %d ",
        params_nonconst.learning_rate_, params_nonconst.ridge_factor_,
        params_nonconst.planning_time_limit_, params_nonconst.max_iterations_);

      if (!optimization_result && replan_count < params_nonconst.max_recovery_attempts_) {
        replan_count++;
        replan_flag = true;
      } else {
        break;
      }
    } else
      break;
  }  // end of while loop

  // resetting the CHOMP Parameters to the original values after a successful plan
  params_nonconst.setRecoveryParams(
    org_learning_rate, org_ridge_factor, org_planning_time_limit, org_max_iterations);

  RCLCPP_DEBUG(
    LOGGER, "Optimization actually took %f sec to run",
    (std::chrono::system_clock::now() - create_time).count());
  create_time = std::chrono::system_clock::now();
  // assume that the trajectory is now optimized, fill in the output structure:

  RCLCPP_DEBUG(LOGGER, "Output trajectory has %zd joints", trajectory.getNumJoints());

  auto result = std::make_shared<robot_trajectory::RobotTrajectory>(
    planning_scene->getRobotModel(), req.group_name);
  // fill in the entire trajectory
  for (size_t i = 0; i < trajectory.getNumPoints(); i++) {
    const Eigen::MatrixXd::RowXpr source = trajectory.getTrajectoryPoint(i);
    auto state = std::make_shared<moveit::core::RobotState>(start_state);
    size_t joint_index = 0;
    for (const moveit::core::JointModel * jm : result->getGroup()->getActiveJointModels()) {
      assert(jm->getVariableCount() == 1);
      state->setVariablePosition(jm->getFirstVariableIndex(), source[joint_index++]);
    }
    result->addSuffixWayPoint(state, 0.0);
  }

  res.trajectory_.resize(1);
  res.trajectory_[0] = result;

  RCLCPP_DEBUG(
    LOGGER, "Bottom took %f sec to create",
    (std::chrono::system_clock::now() - create_time).count());
  RCLCPP_DEBUG(
    LOGGER, "Serviced planning request in %f wall-seconds",
    (std::chrono::system_clock::now() - start_time).count());

  res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  int count = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now() - start_time)
                .count();
  res.processing_time_.resize(1);
  res.processing_time_[0] = (double)count / 1000.0;

  // report planning failure if path has collisions
  if (not optimizer->isCollisionFree()) {
    RCLCPP_ERROR(LOGGER, "Motion plan is invalid.");
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  // check that final state is within goal tolerances
  kinematic_constraints::JointConstraint jc(planning_scene->getRobotModel());
  const moveit::core::RobotState & last_state = result->getLastWayPoint();
  for (const moveit_msgs::msg::JointConstraint & constraint :
       req.goal_constraints[0].joint_constraints) {
    if (!jc.configure(constraint) || !jc.decide(last_state).satisfied) {
      RCLCPP_ERROR(LOGGER, "Goal constraints are violated: %s", constraint.joint_name);
      res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED;
      return false;
    }
  }

  count = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now() - start_time)
            .count();
  res.processing_time_.resize(1);
  res.processing_time_[0] = (double)count / 1000.0;

  return true;
}

bool ChompPlanner::computePoseIK(
  const moveit::core::RobotModelConstPtr & robot_model, const std::string & group_name,
  const std::string & link_name, const Eigen::Isometry3d & pose, const std::string & frame_id,
  const std::map<std::string, double> & seed, std::map<std::string, double> & solution,
  bool check_self_collision, const double timeout) const
{
  if (!robot_model->hasJointModelGroup(group_name)) {
    RCLCPP_ERROR(LOGGER, "Robot model has no planning group named as %s", group_name);
    return false;
  }

  if (!robot_model->getJointModelGroup(group_name)->canSetStateFromIK(link_name)) {
    RCLCPP_ERROR(
      LOGGER, "No valid IK solver exists for %s in planning group %s", link_name, group_name);
    return false;
  }

  if (frame_id != robot_model->getModelFrame()) {
    RCLCPP_ERROR(
      LOGGER, "Given frame (%s) is unequal to model frame(%s)", frame_id,
      robot_model->getModelFrame());
    return false;
  }

  moveit::core::RobotState rstate(robot_model);
  // By setting the robot state to default values, we basically allow
  // the user of this function to supply an incomplete or even empty seed.
  rstate.setToDefaultValues();
  rstate.setVariablePositions(seed);

  moveit::core::GroupStateValidityCallbackFn ik_constraint_function;

  ik_constraint_function = std::bind(
    &ChompPlanner::isStateColliding, this, check_self_collision, robot_model, std::placeholders::_1,
    std::placeholders::_2, std::placeholders::_3);

  // Call Inverse Kinematics solver.
  if (rstate.setFromIK(
        robot_model->getJointModelGroup(group_name), pose, link_name, timeout,
        ik_constraint_function)) {
    // copy the solution
    for (const std::string & joint_name :
         robot_model->getJointModelGroup(group_name)->getActiveJointModelNames()) {
      solution[joint_name] = rstate.getVariablePosition(joint_name);
      RCLCPP_ERROR(LOGGER, "%s: %.4f", joint_name.c_str(), solution[joint_name]);
    }
    return true;
  } else {
    RCLCPP_ERROR(LOGGER, "Inverse kinematics found no solution.");
    return false;
  }
}

bool ChompPlanner::isStateColliding(
  const bool test_for_self_collision, const moveit::core::RobotModelConstPtr & robot_model,
  moveit::core::RobotState * rstate, const moveit::core::JointModelGroup * const group,
  const double * const ik_solution) const
{
  if (!test_for_self_collision) {
    return true;
  }

  rstate->setJointGroupPositions(group, ik_solution);
  rstate->update();
  collision_detection::CollisionRequest collision_req;
  collision_req.group_name = group->getName();
  collision_detection::CollisionResult collision_res;
  planning_scene::PlanningScene(robot_model)
    .checkSelfCollision(collision_req, collision_res, *rstate);

  return !collision_res.collision;
}

}  // namespace chomp
