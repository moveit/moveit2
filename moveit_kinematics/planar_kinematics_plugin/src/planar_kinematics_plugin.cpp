/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik Robotics
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
 *   * Neither the name of PickNik Robotics nor the names of its
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
#include <moveit/planar_kinematics_plugin/planar_kinematics_plugin.hpp>

#include <tf2/transform_datatypes.h>

#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

namespace planar_kinematics_plugin
{
static rclcpp::Logger LOGGER = rclcpp::get_logger("planar_kinematics_plugin");

bool PlanarKinematicsPlugin::initialize(const rclcpp::Node::SharedPtr& node,
                                        const moveit::core::RobotModel& robot_model, const std::string& group_name,
                                        const std::string& base_frame, const std::vector<std::string>& tip_frames,
                                        double search_discretization)
{
  node_ = node;
  storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);
  joint_model_group_ = robot_model_->getJointModelGroup(group_name);
  if (!joint_model_group_)
  {
    return false;
  }

  std::vector<const moveit::core::JointModelGroup*> sub_groups;
  joint_model_group_->getSubgroups(sub_groups);
  // Expect two sub-groups
  // 1- A group with only one planar joint for the mobile base
  // 2- A group for the other 1-DOF joints
  if (sub_groups.size() != 2)
  {
    RCLCPP_ERROR(LOGGER, "Group '%s' need to have two sub-groups", group_name.c_str());
    return false;
  }
  // Find the sub-group for the arm joint model group
  if (const auto group_it = std::find_if(sub_groups.cbegin(), sub_groups.cend(),
                                         [](const moveit::core::JointModelGroup* const group) {
                                           return group->isChain() && group->isSingleDOFJoints();
                                         });
      group_it != sub_groups.cend())
  {
    arm_jmg_ = *group_it;
    RCLCPP_DEBUG_STREAM(LOGGER, "Found sub-group for the arm: '" << arm_jmg_->getName() << "'");
  }
  else
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to find a sub-group for the arm in " << joint_model_group_->getName());
    return false;
  }
  // Find the sub-group for the mobile base
  if (auto group_it = std::find_if(sub_groups.cbegin(), sub_groups.cend(),
                                   [](const moveit::core::JointModelGroup* const group) {
                                     return group->getJointModels().size() == 1 &&
                                            group->getJointModels()[0]->getType() == moveit::core::JointModel::PLANAR;
                                   });
      group_it != sub_groups.cend())
  {
    RCLCPP_DEBUG_STREAM(LOGGER, "Found sub-group for the mobile base: " << (*group_it)->getName());
    mobile_base_jmg_ = *group_it;
    mobile_base_joint_ = mobile_base_jmg_->getJointModels()[0];
  }
  else
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to find a sub-group for the mobile base in " << joint_model_group_->getName());
    return false;
  }

  mobile_base_index_ = arm_jmg_->getActiveJointModels().size() + arm_jmg_->getMimicJointModels().size();
  dimension_ = mobile_base_index_ + mobile_base_joint_->getVariableCount();

  for (size_t i = 0; i < arm_jmg_->getJointModels().size(); ++i)
  {
    if (arm_jmg_->getJointModels()[i]->getType() == moveit::core::JointModel::REVOLUTE ||
        arm_jmg_->getJointModels()[i]->getType() == moveit::core::JointModel::PRISMATIC)
    {
      solver_info_.joint_names.push_back(arm_jmg_->getJointModelNames()[i]);
      const auto& jvec = arm_jmg_->getJointModels()[i]->getVariableBoundsMsg();
      solver_info_.limits.insert(solver_info_.limits.end(), jvec.begin(), jvec.end());
    }
  }

  solver_info_.joint_names.push_back(mobile_base_joint_->getName());
  const auto& jvec = mobile_base_joint_->getVariableBoundsMsg();
  solver_info_.limits.insert(solver_info_.limits.end(), jvec.begin(), jvec.end());

  if (!joint_model_group_->hasLinkModel(getTipFrame()))
  {
    RCLCPP_ERROR(LOGGER, "Could not find tip name '%s' in joint group '%s'", getTipFrame().c_str(), group_name.c_str());
    return false;
  }
  solver_info_.link_names.push_back(getTipFrame());

  joint_min_.resize(solver_info_.limits.size());
  joint_max_.resize(solver_info_.limits.size());

  for (unsigned int i = 0; i < solver_info_.limits.size(); i++)
  {
    joint_min_(i) = solver_info_.limits[i].min_position;
    joint_max_(i) = solver_info_.limits[i].max_position;
  }

  // Setup the joint state groups that we need
  state_ = std::make_shared<moveit::core::RobotState>(robot_model_);

  initialized_ = true;
  RCLCPP_DEBUG(LOGGER, "KDL solver initialized");
  return true;
}

bool PlanarKinematicsPlugin::timedOut(const rclcpp::Time& start_time, double duration) const
{
  return ((node_->now() - start_time).seconds() >= duration);
}

bool PlanarKinematicsPlugin::getPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                           const std::vector<double>& ik_seed_state, std::vector<double>& solution,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;

  // limit search to a single attempt by setting a timeout of zero
  return searchPositionIK(ik_pose, ik_seed_state, 0.0, consistency_limits, solution, IKCallbackFn(), error_code,
                          options);
}

bool PlanarKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                              const std::vector<double>& ik_seed_state, double timeout,
                                              std::vector<double>& solution,
                                              moveit_msgs::msg::MoveItErrorCodes& error_code,
                                              const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, IKCallbackFn(), error_code,
                          options);
}

bool PlanarKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                              const std::vector<double>& ik_seed_state, double timeout,
                                              const std::vector<double>& consistency_limits,
                                              std::vector<double>& solution,
                                              moveit_msgs::msg::MoveItErrorCodes& error_code,
                                              const kinematics::KinematicsQueryOptions& options) const
{
  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, IKCallbackFn(), error_code,
                          options);
}

bool PlanarKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                              const std::vector<double>& ik_seed_state, double timeout,
                                              std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                              moveit_msgs::msg::MoveItErrorCodes& error_code,
                                              const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code,
                          options);
}

bool PlanarKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                              const std::vector<double>& ik_seed_state, double timeout,
                                              const std::vector<double>& consistency_limits,
                                              std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                              moveit_msgs::msg::MoveItErrorCodes& error_code,
                                              const kinematics::KinematicsQueryOptions& options) const
{
  rclcpp::Time start_time = node_->now();
  if (!initialized_)
  {
    RCLCPP_ERROR(LOGGER, "kinematics solver not initialized");
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
  }

  if (ik_seed_state.size() != dimension_)
  {
    RCLCPP_ERROR(LOGGER, "Seed state must have size %d instead of size %zu\n", dimension_, ik_seed_state.size());
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
  }

  if (!consistency_limits.empty())
  {
    if (consistency_limits.size() != dimension_)
    {
      RCLCPP_ERROR(LOGGER,
                   "Consistency limits must be empty or have size %d instead "
                   "of size %zu\n",
                   dimension_, consistency_limits.size());
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
      return false;
    }
  }

  solution.resize(dimension_);

  RCLCPP_DEBUG_STREAM(LOGGER, "searchPositionIK: Position request pose is "
                                  << ik_pose.position.x << " " << ik_pose.position.y << " " << ik_pose.position.z << " "
                                  << ik_pose.orientation.x << " " << ik_pose.orientation.y << " "
                                  << ik_pose.orientation.z << " " << ik_pose.orientation.w);

  // auto ik_seed_state_local = ik_seed_state;
  std::vector<double> arm_jmg_solution(solution.begin(), std::next(solution.begin(), mobile_base_index_));
  const auto arm_jmg_consistency_limits =
      consistency_limits.empty() ?
          std::vector<double>{} :
          std::vector<double>(consistency_limits.cbegin(), std::next(consistency_limits.cbegin(), mobile_base_index_));

  const std::vector<double> arm_jmg_ik_seed_state(ik_seed_state.cbegin(),
                                                  std::next(ik_seed_state.cbegin(), mobile_base_index_));

  const auto arm_ik_solver = arm_jmg_->getGroupKinematics().first.solver_instance_;
  Eigen::Isometry3d eigen_ik_pose;
  tf2::fromMsg(ik_pose, eigen_ik_pose);
  solution = ik_seed_state;
  // Notation:
  // B_T_A: The homogenous transformation of A frame w.r.t. B frame
  // arm_b: Base frame of the kinematics solver for the arm jmg
  unsigned int attempt = 0;
  do
  {
    ++attempt;
    // First we solve for the arm only by fixing the mobile base
    state_->setJointGroupPositions(mobile_base_jmg_, &solution[mobile_base_index_]);
    state_->updateLinkTransforms();
    // Convert the desired pose from the arm + mobile base solver base frame to the arm group base frame
    const auto arm_b_t_ik_pose = tf2::toMsg(state_->getFrameTransform(arm_ik_solver->getBaseFrame()).inverse() *
                                            state_->getFrameTransform(getBaseFrame()) * eigen_ik_pose);

    // We can't use the solution_callback directly with the IK solver for the arm since we'll only get the joint values
    // for the arm and solution_callback meant to be used with the joint model group that contains both the arm & mobile
    // base to fix this we create a callback that will append the solution for the mobile base to the arm's solution and
    // call solution_callback
    IKCallbackFn arm_solution_callback;
    if (!solution_callback.empty())
    {
      arm_solution_callback = [&solution_callback, &solution = std::as_const(solution),
                               mobile_base_index = this->mobile_base_index_](
                                  const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& arm_solution,
                                  moveit_msgs::msg::MoveItErrorCodes& error_code) {
        std::vector<double> jmg_solution(arm_solution.cbegin(), arm_solution.cend());
        // Combine arm's solution with mobile base's solution to call solution_callback
        std::copy(std::next(solution.cbegin(), mobile_base_index), solution.cend(), std::back_inserter(jmg_solution));
        solution_callback(ik_pose, jmg_solution, error_code);
      };
    }
    const bool ik_valid = arm_ik_solver->searchPositionIK(arm_b_t_ik_pose, arm_jmg_ik_seed_state, timeout * 0.5,
                                                          arm_jmg_consistency_limits, arm_jmg_solution,
                                                          arm_solution_callback, error_code, options);

    if (ik_valid || options.return_approximate_solution)  // found acceptable solution
    {
      std::copy(arm_jmg_solution.begin(), arm_jmg_solution.end(), solution.begin());
      if (error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        continue;
      RCLCPP_DEBUG_STREAM(LOGGER, "Solved after " << (node_->now() - start_time).seconds() << " < " << timeout
                                                  << "s and " << attempt << " attempts");
      return true;
    }

    // If we failed to find a solution for the arm JMG we solve only for the mobile base joint by fixing the arm JMG
    if (attempt == 1)
      state_->setJointGroupPositions(arm_jmg_, &arm_jmg_ik_seed_state[0]);
    else
      state_->setJointGroupPositions(arm_jmg_, &arm_jmg_solution[0]);
    state_->updateLinkTransforms();
    // The transformation for the tip frame w.r.t. the mobile base link given the arm's joint values
    const auto mobile_base_link_t_current =
        state_->getFrameTransform(mobile_base_joint_->getChildLinkModel()->getName()).inverse() *
        state_->getFrameTransform(getTipFrame());
    // The error for the mobile base link
    const auto t_mobile_base_link = eigen_ik_pose * mobile_base_link_t_current.inverse();

    Eigen::AngleAxisd rotation(t_mobile_base_link.rotation());
    Eigen::Map<Eigen::Vector2d>(&solution.at(mobile_base_index_)) = t_mobile_base_link.translation().block<2, 1>(0, 0);
    // Check if the rotation is a pure yaw
    const double angle = rotation.axis()(2) * rotation.angle();
    if (std::abs(rotation.angle()) - std::abs(angle) < 1e-2)
      solution.back() = angle;
  } while (!timedOut(start_time, timeout));
  error_code.val = moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT;
  return false;
}

bool PlanarKinematicsPlugin::getPositionFK(const std::vector<std::string>& link_names,
                                           const std::vector<double>& joint_angles,
                                           std::vector<geometry_msgs::msg::Pose>& poses) const
{
  if (!initialized_)
  {
    RCLCPP_ERROR(LOGGER, "kinematics solver not initialized");
    return false;
  }
  poses.resize(link_names.size());
  if (joint_angles.size() != dimension_)
  {
    RCLCPP_ERROR(LOGGER, "Joint angles vector must have size: %d", dimension_);
    return false;
  }
  state_->setJointGroupPositions(joint_model_group_, joint_angles);
  state_->updateLinkTransforms();

  for (unsigned int i = 0; i < poses.size(); i++)
  {
    poses[i] = tf2::toMsg(state_->getFrameTransform(link_names[i]));
  }
  return true;
}

const std::vector<std::string>& PlanarKinematicsPlugin::getJointNames() const
{
  return solver_info_.joint_names;
}

const std::vector<std::string>& PlanarKinematicsPlugin::getLinkNames() const
{
  return solver_info_.link_names;
}

}  // namespace planar_kinematics_plugin

// register PlanarKinematicsPlugin as a KinematicsBase implementation
#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(planar_kinematics_plugin::PlanarKinematicsPlugin, kinematics::KinematicsBase)
