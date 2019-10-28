/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, CRI group, NTU, Singapore
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
 *   * Neither the name of CRI group nor the names of its
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

/* Author: Francisco Suarez-Ruiz */

#include <moveit/lma_kinematics_plugin/lma_kinematics_plugin.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

#include <tf2_kdl/tf2_kdl.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>

// register as a KinematicsBase implementation
#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(lma_kinematics_plugin::LMAKinematicsPlugin, kinematics::KinematicsBase)

namespace lma_kinematics_plugin
{
LMAKinematicsPlugin::LMAKinematicsPlugin() : initialized_(false)
{
}

void LMAKinematicsPlugin::getRandomConfiguration(Eigen::VectorXd& jnt_array) const
{
  state_->setToRandomPositions(joint_model_group_);
  state_->copyJointGroupPositions(joint_model_group_, &jnt_array[0]);
}

void LMAKinematicsPlugin::getRandomConfiguration(const Eigen::VectorXd& seed_state,
                                                 const std::vector<double>& consistency_limits,
                                                 Eigen::VectorXd& jnt_array) const
{
  joint_model_group_->getVariableRandomPositionsNearBy(state_->getRandomNumberGenerator(), &jnt_array[0],
                                                       &seed_state[0], consistency_limits);
}

bool LMAKinematicsPlugin::checkConsistency(const Eigen::VectorXd& seed_state,
                                           const std::vector<double>& consistency_limits,
                                           const Eigen::VectorXd& solution) const
{
  for (std::size_t i = 0; i < dimension_; ++i)
    if (fabs(seed_state(i) - solution(i)) > consistency_limits[i])
      return false;
  return true;
}

bool LMAKinematicsPlugin::initialize(const moveit::core::RobotModel& robot_model, const std::string& group_name,
                                     const std::string& base_frame, const std::vector<std::string>& tip_frames,
                                     double search_discretization)
{
  storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);
  joint_model_group_ = robot_model_->getJointModelGroup(group_name);
  if (!joint_model_group_)
    return false;

  if (!joint_model_group_->isChain())
  {
    RCLCPP_ERROR(node_->get_logger(), "Group '%s' is not a chain", group_name.c_str());
    return false;
  }
  if (!joint_model_group_->isSingleDOFJoints())
  {
    RCLCPP_ERROR(node_->get_logger(), "Group '%s' includes joints that have more than 1 DOF", group_name.c_str());
    return false;
  }

  KDL::Tree kdl_tree;

  if (!kdl_parser::treeFromUrdfModel(*robot_model.getURDF(), kdl_tree))
  {
    RCLCPP_ERROR(node_->get_logger(), "Could not initialize tree object");
    return false;
  }
  if (!kdl_tree.getChain(base_frame_, getTipFrame(), kdl_chain_))
  {
    RCLCPP_ERROR(node_->get_logger(), "Could not initialize chain object");
    return false;
  }

  for (const robot_model::JointModel* jm : joint_model_group_->getJointModels())
  {
    if (jm->getType() == moveit::core::JointModel::REVOLUTE || jm->getType() == moveit::core::JointModel::PRISMATIC)
    {
      joints_.push_back(jm);
      joint_names_.push_back(jm->getName());
    }
  }
  dimension_ = joints_.size();

  // Get Solver Parameters
  lookupParam(node_,"max_solver_iterations", max_solver_iterations_, 500);
  lookupParam(node_,"epsilon", epsilon_, 1e-5);
  lookupParam(node_,"orientation_vs_position", orientation_vs_position_weight_, 0.01);

  bool position_ik;
  lookupParam(node_,"position_only_ik", position_ik, false);
  if (position_ik)  // position_only_ik overrules orientation_vs_position
    orientation_vs_position_weight_ = 0.0;
  if (orientation_vs_position_weight_ == 0.0)
    RCLCPP_INFO(node_->get_logger(), "Using position only ik");

  // Setup the joint state groups that we need
  state_.reset(new robot_state::RobotState(robot_model_));

  fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

  initialized_ = true;
  RCLCPP_DEBUG(node_->get_logger(), "LMA solver initialized");
  return true;
}

bool LMAKinematicsPlugin::timedOut(const std::chrono::system_clock::time_point& start_time, double duration) const
{
  return (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start_time).count() /1000.0 >= duration);
}

bool LMAKinematicsPlugin::getPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                        std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                                        const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;

  // limit search to a single attempt by setting a timeout of zero
  return searchPositionIK(ik_pose, ik_seed_state, 0.0, solution, IKCallbackFn(), error_code, consistency_limits,
                          options);
}

bool LMAKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                           double timeout, std::vector<double>& solution,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, IKCallbackFn(), error_code, consistency_limits,
                          options);
}

bool LMAKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                           double timeout, const std::vector<double>& consistency_limits,
                                           std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, IKCallbackFn(), error_code, consistency_limits,
                          options);
}

bool LMAKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                           double timeout, std::vector<double>& solution,
                                           const IKCallbackFn& solution_callback,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool LMAKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                           double timeout, const std::vector<double>& consistency_limits,
                                           std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

void LMAKinematicsPlugin::harmonize(Eigen::VectorXd& values) const
{
  size_t i = 0;
  for (auto* jm : joints_)
    jm->harmonizePosition(&values[i++]);
}

bool LMAKinematicsPlugin::obeysLimits(const Eigen::VectorXd& values) const
{
  size_t i = 0;
  for (const auto& jm : joints_)
    if (!jm->satisfiesPositionBounds(&values[i++]))
      return false;
  return true;
}

bool LMAKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                           double timeout, std::vector<double>& solution,
                                           const IKCallbackFn& solution_callback,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const std::vector<double>& consistency_limits,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  auto start_time = std::chrono::system_clock::now();
  if (!initialized_)
  {
    RCLCPP_ERROR(node_->get_logger(), "kinematics solver not initialized");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  if (ik_seed_state.size() != dimension_)
  {
    RCLCPP_ERROR(node_->get_logger(), "Seed state must have size %d instead of size %d",dimension_, ik_seed_state.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  if (!consistency_limits.empty() && consistency_limits.size() != dimension_)
  {
    RCLCPP_ERROR(node_->get_logger(), "Consistency limits be empty or must have size %d instead of size %d", dimension_, consistency_limits.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  Eigen::Matrix<double, 6, 1> cartesian_weights;
  cartesian_weights(0) = 1;
  cartesian_weights(1) = 1;
  cartesian_weights(2) = 1;
  cartesian_weights(3) = orientation_vs_position_weight_;
  cartesian_weights(4) = orientation_vs_position_weight_;
  cartesian_weights(5) = orientation_vs_position_weight_;

  KDL::JntArray jnt_seed_state(dimension_);
  KDL::JntArray jnt_pos_in(dimension_);
  KDL::JntArray jnt_pos_out(dimension_);
  jnt_seed_state.data = Eigen::Map<const Eigen::VectorXd>(ik_seed_state.data(), ik_seed_state.size());
  jnt_pos_in = jnt_seed_state;

  KDL::ChainIkSolverPos_LMA ik_solver_pos(kdl_chain_, cartesian_weights, epsilon_, max_solver_iterations_);
  solution.resize(dimension_);

  KDL::Frame pose_desired;
  tf2::fromMsg(ik_pose, pose_desired);

  RCLCPP_DEBUG(node_->get_logger(), "searchPositionIK2: Position request pose is %d %d %d %d %d %d %d",
                                    ik_pose.position.x, ik_pose.position.y, ik_pose.position.z, ik_pose.orientation.x,
                                    ik_pose.orientation.y, ik_pose.orientation.z, ik_pose.orientation.w);
  unsigned int attempt = 0;
  do
  {
    ++attempt;
    if (attempt > 1)  // randomly re-seed after first attempt
    {
      if (!consistency_limits.empty())
        getRandomConfiguration(jnt_seed_state.data, consistency_limits, jnt_pos_in.data);
      else
        getRandomConfiguration(jnt_pos_in.data);
      RCLCPP_DEBUG(node_->get_logger(), "New random configuration (%d): %d",attempt, jnt_pos_in.data);
    }

    int ik_valid = ik_solver_pos.CartToJnt(jnt_pos_in, pose_desired, jnt_pos_out);
    if (ik_valid == 0 || options.return_approximate_solution)  // found acceptable solution
    {
      harmonize(jnt_pos_out.data);
      if (!consistency_limits.empty() && !checkConsistency(jnt_seed_state.data, consistency_limits, jnt_pos_out.data))
        continue;
      if (!obeysLimits(jnt_pos_out.data))
        continue;

      Eigen::Map<Eigen::VectorXd>(solution.data(), solution.size()) = jnt_pos_out.data;
      if (!solution_callback.empty())
      {
        solution_callback(ik_pose, solution, error_code);
        if (error_code.val != error_code.SUCCESS)
          continue;
      }

      // solution passed consistency check and solution callback
      error_code.val = error_code.SUCCESS;
      RCLCPP_DEBUG(node_->get_logger(), "Solved after %ld < %fs and %d attempts",(std::chrono::system_clock::now() - start_time).count(),timeout,attempt);
      return true;
    }
  } while (!timedOut(start_time, timeout));

  RCLCPP_DEBUG(node_->get_logger(), "IK timed out after %ld > %fs and %d attempts",(std::chrono::system_clock::now() - start_time).count(),timeout,attempt);
  error_code.val = error_code.TIMED_OUT;
  return false;
}

bool LMAKinematicsPlugin::getPositionFK(const std::vector<std::string>& link_names,
                                        const std::vector<double>& joint_angles,
                                        std::vector<geometry_msgs::msg::Pose>& poses) const
{
  if (!initialized_)
  {
    RCLCPP_ERROR(node_->get_logger(), "kinematics solver not initialized");
    return false;
  }
  poses.resize(link_names.size());
  if (joint_angles.size() != dimension_)
  {
    RCLCPP_ERROR(node_->get_logger(), "Joint angles vector must have size: %d", dimension_);
    return false;
  }

  KDL::Frame p_out;
  KDL::JntArray jnt_pos_in(dimension_);
  jnt_pos_in.data = Eigen::Map<const Eigen::VectorXd>(joint_angles.data(), joint_angles.size());

  bool valid = true;
  for (unsigned int i = 0; i < poses.size(); i++)
  {
    if (fk_solver_->JntToCart(jnt_pos_in, p_out) >= 0)
    {
      //TODO (anasarrak): Add a toMsg transformation for KDL::Frame
      poses[i].position.x = p_out.p[0];
      poses[i].position.y = p_out.p[1];
      poses[i].position.z = p_out.p[2];
      p_out.M.GetQuaternion(poses[i].orientation.x, poses[i].orientation.y,
                            poses[i].orientation.z, poses[i].orientation.w);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Could not compute FK for %s", link_names[i].c_str());
      valid = false;
    }
  }
  return valid;
}

const std::vector<std::string>& LMAKinematicsPlugin::getJointNames() const
{
  return joint_names_;
}

const std::vector<std::string>& LMAKinematicsPlugin::getLinkNames() const
{
  return getTipFrames();
}

}  // namespace lma_kinematics_plugin
