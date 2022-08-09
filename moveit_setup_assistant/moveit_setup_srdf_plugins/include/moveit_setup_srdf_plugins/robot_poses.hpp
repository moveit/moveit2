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

/* Author: David V. Lu!! */
#pragma once

#include <moveit_setup_srdf_plugins/srdf_step.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>

namespace moveit_setup
{
namespace srdf_setup
{
// Note: Does not derive from SuperSRDFStep because we always need to check the name AND the group name for each pose
class RobotPoses : public SRDFStep
{
public:
  std::string getName() const override
  {
    return "Robot Poses";
  }

  void onInit() override;

  bool isReady() const override
  {
    return hasGroups();
  }

  std::vector<std::string> getGroupNames() const
  {
    return srdf_config_->getGroupNames();
  }

  /**
   * Find the associated data by name
   *
   * @param name - name of data to find in datastructure
   * @return pointer to data in datastructure
   */
  srdf::Model::GroupState* findPoseByName(const std::string& name, const std::string& group);

  std::vector<srdf::Model::GroupState>& getGroupStates()
  {
    return srdf_config_->getGroupStates();
  }

  moveit::core::RobotState& getState()
  {
    return srdf_config_->getPlanningScene()->getCurrentStateNonConst();
  }

  /**
   * @brief Publish the given state on the moveit_robot_state topic
   */
  void publishState(const moveit::core::RobotState& robot_state);

  /**
   * @brief Check if the given robot state is in collision
   */
  bool checkSelfCollision(const moveit::core::RobotState& robot_state);

  /**
   * @brief Returns a vector of joint models for the given group name
   * @throws runtime_error if the group does not exist
   *
   * Note: "Simple" means we exclude Passive/Mimic/MultiDOF joints
   */
  std::vector<const moveit::core::JointModel*> getSimpleJointModels(const std::string& group_name) const;

  void removePoseByName(const std::string& pose_name, const std::string& group_name)
  {
    srdf_config_->removePoseByName(pose_name, group_name);
  }

  void setToCurrentValues(srdf::Model::GroupState& group_state);

  /// Load the allowed collision matrix from the SRDF's list of link pairs
  void loadAllowedCollisionMatrix();

protected:
  /// Remember the publisher for quick publishing later
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr pub_robot_state_;

  // ******************************************************************************************
  // Collision Variables
  // ******************************************************************************************
  collision_detection::CollisionRequest request_;

  /// Allowed collision matrix for robot poses
  collision_detection::AllowedCollisionMatrix allowed_collision_matrix_;
};
}  // namespace srdf_setup
}  // namespace moveit_setup
