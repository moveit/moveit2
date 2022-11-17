/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK, The University of Tokyo.
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
 *   * Neither the name of JSK, The University of Tokyo nor the names of its
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

/* Author: Dave Coleman, Masaki Murooka */

#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit/srv_kinematics_plugin/srv_kinematics_plugin.h>
#include <class_loader/class_loader.hpp>
#include <moveit/robot_state/conversions.h>
#include <iterator>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// register SRVKinematics as a KinematicsBase implementation
CLASS_LOADER_REGISTER_CLASS(srv_kinematics_plugin::SrvKinematicsPlugin, kinematics::KinematicsBase)

namespace srv_kinematics_plugin
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_srv_kinematics_plugin.srv_kinematics_plugin");

SrvKinematicsPlugin::SrvKinematicsPlugin() : active_(false)
{
}

bool SrvKinematicsPlugin::initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
                                     const std::string& group_name, const std::string& base_frame,
                                     const std::vector<std::string>& tip_frames, double search_discretization)
{
  node_ = node;
  bool debug = false;

  RCLCPP_INFO(LOGGER, "SrvKinematicsPlugin initializing");

  // Get Solver Parameters
  std::string kinematics_param_prefix = "robot_description_kinematics." + group_name;
  param_listener_ = std::make_shared<srv_kinematics::ParamListener>(node, kinematics_param_prefix);
  params_ = param_listener_->get_params();

  storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);
  joint_model_group_ = robot_model_->getJointModelGroup(group_name);
  if (!joint_model_group_)
    return false;

  if (debug)
  {
    std::cout << "Joint Model Variable Names: ------------------------------------------- \n ";
    const std::vector<std::string> jm_names = joint_model_group_->getVariableNames();
    std::copy(jm_names.begin(), jm_names.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
    std::cout << '\n';
  }

  // Get the dimension of the planning group
  dimension_ = joint_model_group_->getVariableCount();
  RCLCPP_INFO_STREAM(LOGGER, "Dimension planning group '"
                                 << group_name << "': " << dimension_
                                 << ". Active Joints Models: " << joint_model_group_->getActiveJointModels().size()
                                 << ". Mimic Joint Models: " << joint_model_group_->getMimicJointModels().size());

  // Copy joint names
  for (std::size_t i = 0; i < joint_model_group_->getJointModels().size(); ++i)
  {
    ik_group_info_.joint_names.push_back(joint_model_group_->getJointModelNames()[i]);
  }

  if (debug)
  {
    RCLCPP_ERROR(LOGGER, "tip links available:");
    std::copy(tip_frames_.begin(), tip_frames_.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
  }

  // Make sure all the tip links are in the link_names vector
  for (const std::string& tip_frame : tip_frames_)
  {
    if (!joint_model_group_->hasLinkModel(tip_frame))
    {
      RCLCPP_ERROR(LOGGER, "Could not find tip name '%s' in joint group '%s'", tip_frame.c_str(), group_name.c_str());
      return false;
    }
    ik_group_info_.link_names.push_back(tip_frame);
  }

  // Setup the joint state groups that we need
  robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
  robot_state_->setToDefaultValues();

  // Create the ROS2 service client
  RCLCPP_DEBUG(LOGGER, "IK Service client topic : %s", params_.kinematics_solver_service_name.c_str());
  ik_service_client_ = node_->create_client<moveit_msgs::srv::GetPositionIK>(params_.kinematics_solver_service_name);

  if (!ik_service_client_->wait_for_service(std::chrono::seconds(1)))  // wait 0.1 seconds, blocking
    RCLCPP_WARN_STREAM(LOGGER,
                       "Unable to connect to ROS service client with name: " << ik_service_client_->get_service_name());
  else
    RCLCPP_INFO_STREAM(LOGGER,
                       "Service client started with ROS service name: " << ik_service_client_->get_service_name());

  active_ = true;
  RCLCPP_DEBUG(LOGGER, "ROS service-based kinematics solver initialized");
  return true;
}

bool SrvKinematicsPlugin::setRedundantJoints(const std::vector<unsigned int>& redundant_joints)
{
  if (num_possible_redundant_joints_ < 0)
  {
    RCLCPP_ERROR(LOGGER, "This group cannot have redundant joints");
    return false;
  }
  if (int(redundant_joints.size()) > num_possible_redundant_joints_)
  {
    RCLCPP_ERROR(LOGGER, "This group can only have %d redundant joints", num_possible_redundant_joints_);
    return false;
  }

  return true;
}

bool SrvKinematicsPlugin::isRedundantJoint(unsigned int index) const
{
  for (const unsigned int& redundant_joint_indice : redundant_joint_indices_)
    if (redundant_joint_indice == index)
      return true;
  return false;
}

int SrvKinematicsPlugin::getJointIndex(const std::string& name) const
{
  for (unsigned int i = 0; i < ik_group_info_.joint_names.size(); ++i)
  {
    if (ik_group_info_.joint_names[i] == name)
      return i;
  }
  return -1;
}

bool SrvKinematicsPlugin::timedOut(const rclcpp::Time& start_time, double duration) const
{
  return ((node_->now() - start_time).seconds() >= duration);
}

bool SrvKinematicsPlugin::getPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                        const std::vector<double>& ik_seed_state, std::vector<double>& solution,
                                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                                        const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, default_timeout_, consistency_limits, solution, IKCallbackFn(),
                          error_code, options);
}

bool SrvKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                           const std::vector<double>& ik_seed_state, double timeout,
                                           std::vector<double>& solution,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, IKCallbackFn(), error_code,
                          options);
}

bool SrvKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                           const std::vector<double>& ik_seed_state, double timeout,
                                           const std::vector<double>& consistency_limits, std::vector<double>& solution,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, IKCallbackFn(), error_code,
                          options);
}

bool SrvKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                           const std::vector<double>& ik_seed_state, double timeout,
                                           std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code,
                          options);
}

bool SrvKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                           const std::vector<double>& ik_seed_state, double timeout,
                                           const std::vector<double>& consistency_limits, std::vector<double>& solution,
                                           const IKCallbackFn& solution_callback,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  // Convert single pose into a vector of one pose
  std::vector<geometry_msgs::msg::Pose> ik_poses;
  ik_poses.push_back(ik_pose);

  return searchPositionIK(ik_poses, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code,
                          options);
}

bool SrvKinematicsPlugin::searchPositionIK(const std::vector<geometry_msgs::msg::Pose>& ik_poses,
                                           const std::vector<double>& ik_seed_state, double /*timeout*/,
                                           const std::vector<double>& /*consistency_limits*/,
                                           std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& /*options*/,
                                           const moveit::core::RobotState* /*context_state*/) const
{
  // Check if active
  if (!active_)
  {
    RCLCPP_ERROR(LOGGER, "kinematics not active");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  // Check if seed state correct
  if (ik_seed_state.size() != dimension_)
  {
    RCLCPP_ERROR_STREAM(LOGGER,
                        "Seed state must have size " << dimension_ << " instead of size " << ik_seed_state.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  // Check that we have the same number of poses as tips
  if (tip_frames_.size() != ik_poses.size())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Mismatched number of pose requests (" << ik_poses.size() << ") to tip frames ("
                                                                       << tip_frames_.size()
                                                                       << ") in searchPositionIK");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  // Create the service message
  auto ik_srv = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
  ik_srv->ik_request.avoid_collisions = true;
  ik_srv->ik_request.group_name = getGroupName();

  // Copy seed state into virtual robot state and convert into moveit_msg
  robot_state_->setJointGroupPositions(joint_model_group_, ik_seed_state);
  moveit::core::robotStateToRobotStateMsg(*robot_state_, ik_srv->ik_request.robot_state);

  // Load the poses into the request in difference places depending if there is more than one or not
  geometry_msgs::msg::PoseStamped ik_pose_st;
  ik_pose_st.header.frame_id = base_frame_;
  if (tip_frames_.size() > 1)
  {
    // Load into vector of poses
    for (std::size_t i = 0; i < tip_frames_.size(); ++i)
    {
      ik_pose_st.pose = ik_poses[i];
      ik_srv->ik_request.pose_stamped_vector.push_back(ik_pose_st);
      ik_srv->ik_request.ik_link_names.push_back(tip_frames_[i]);
    }
  }
  else
  {
    ik_pose_st.pose = ik_poses[0];

    // Load into single pose value
    ik_srv->ik_request.pose_stamped = ik_pose_st;
    ik_srv->ik_request.ik_link_name = getTipFrames()[0];
  }

  RCLCPP_DEBUG(LOGGER, "Calling service: %s", ik_service_client_->get_service_name());
  auto result_future = ik_service_client_->async_send_request(ik_srv);
  const auto& response = result_future.get();
  if (rclcpp::spin_until_future_complete(node_, result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    // Check error code
    error_code.val = response->error_code.val;
    if (error_code.val != error_code.SUCCESS)
    {
      // TODO (JafarAbdi) Print the entire message for ROS2?
      // RCLCPP_DEBUG("srv", "An IK that satisifes the constraints and is collision free could not be found."
      //                                   << "\nRequest was: \n"
      //                                   << ik_srv.request.ik_request << "\nResponse was: \n"
      //                                   << ik_srv.response.solution);
      switch (error_code.val)
      {
        case moveit_msgs::msg::MoveItErrorCodes::FAILURE:
          RCLCPP_ERROR(LOGGER, "Service failed with with error code: FAILURE");
          break;
        case moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION:
          RCLCPP_DEBUG(LOGGER, "Service failed with with error code: NO IK SOLUTION");
          break;
        default:
          RCLCPP_DEBUG_STREAM(LOGGER, "Service failed with with error code: " << error_code.val);
      }
      return false;
    }
  }
  else
  {
    RCLCPP_DEBUG_STREAM(LOGGER, "Service call failed to connect to service: " << ik_service_client_->get_service_name());
    error_code.val = error_code.FAILURE;
    return false;
  }
  // Convert the robot state message to our robot_state representation
  if (!moveit::core::robotStateMsgToRobotState(response->solution, *robot_state_))
  {
    RCLCPP_ERROR(LOGGER, "An error occurred converting received robot state message into internal robot state.");
    error_code.val = error_code.FAILURE;
    return false;
  }

  // Get just the joints we are concerned about in our planning group
  robot_state_->copyJointGroupPositions(joint_model_group_, solution);

  // Run the solution callback (i.e. collision checker) if available
  if (solution_callback)
  {
    RCLCPP_DEBUG(LOGGER, "Calling solution callback on IK solution");

    // hack: should use all poses, not just the 0th
    solution_callback(ik_poses[0], solution, error_code);

    if (error_code.val != error_code.SUCCESS)
    {
      switch (error_code.val)
      {
        case moveit_msgs::msg::MoveItErrorCodes::FAILURE:
          RCLCPP_ERROR(LOGGER, "IK solution callback failed with with error code: FAILURE");
          break;
        case moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION:
          RCLCPP_ERROR(LOGGER, "IK solution callback failed with with error code: "
                               "NO IK SOLUTION");
          break;
        default:
          RCLCPP_ERROR_STREAM(LOGGER, "IK solution callback failed with with error code: " << error_code.val);
      }
      return false;
    }
  }

  RCLCPP_INFO(LOGGER, "IK Solver Succeeded!");
  return true;
}

bool SrvKinematicsPlugin::getPositionFK(const std::vector<std::string>& link_names,
                                        const std::vector<double>& joint_angles,
                                        std::vector<geometry_msgs::msg::Pose>& poses) const
{
  if (!active_)
  {
    RCLCPP_ERROR(LOGGER, "kinematics not active");
    return false;
  }
  poses.resize(link_names.size());
  if (joint_angles.size() != dimension_)
  {
    RCLCPP_ERROR(LOGGER, "Joint angles vector must have size: %d", dimension_);
    return false;
  }

  RCLCPP_ERROR(LOGGER, "Forward kinematics not implemented");

  return false;
}

const std::vector<std::string>& SrvKinematicsPlugin::getJointNames() const
{
  return ik_group_info_.joint_names;
}

const std::vector<std::string>& SrvKinematicsPlugin::getLinkNames() const
{
  return ik_group_info_.link_names;
}

const std::vector<std::string>& SrvKinematicsPlugin::getVariableNames() const
{
  return joint_model_group_->getVariableNames();
}

}  // namespace srv_kinematics_plugin
