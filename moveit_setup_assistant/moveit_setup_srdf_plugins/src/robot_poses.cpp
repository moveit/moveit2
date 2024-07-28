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

#include <moveit_setup_srdf_plugins/robot_poses.hpp>
#include <moveit/robot_state/conversions.h>

namespace moveit_setup
{
namespace srdf_setup
{
void RobotPoses::onInit()
{
  SRDFStep::onInit();

  // Create scene publisher for later use
  pub_robot_state_ = parent_node_->create_publisher<moveit_msgs::msg::DisplayRobotState>("moveit_robot_state", 1);

  // Set the planning scene
  // srdf_config_->getPlanningScene()->setName("MoveIt Planning Scene");

  // Collision Detection initialization -------------------------------

  // Setup the request
  request_.contacts = true;
  request_.max_contacts = 1;
  request_.max_contacts_per_pair = 1;
  request_.verbose = false;
}

// ******************************************************************************************
// Find the associated data by name
// ******************************************************************************************
srdf::Model::GroupState* RobotPoses::findPoseByName(const std::string& name, const std::string& group)
{
  // Find the group state we are editing based on the pose name
  srdf::Model::GroupState* searched_state = nullptr;  // used for holding our search results

  for (srdf::Model::GroupState& state : srdf_config_->getGroupStates())
  {
    if (state.name_ == name && state.group_ == group)  // match
    {
      searched_state = &state;
      break;
    }
  }

  return searched_state;
}

// ******************************************************************************************
// Load the allowed collision matrix from the SRDF's list of link pairs
// ******************************************************************************************
void RobotPoses::loadAllowedCollisionMatrix()
{
  // Clear the allowed collision matrix
  allowed_collision_matrix_.clear();

  // Update the allowed collision matrix, in case there has been a change
  for (const auto& disabled_collision : srdf_config_->getDisabledCollisions())
  {
    allowed_collision_matrix_.setEntry(disabled_collision.link1_, disabled_collision.link2_, true);
  }
}

// ******************************************************************************************
// Publish the current RobotState to Rviz
// ******************************************************************************************
void RobotPoses::publishState(const moveit::core::RobotState& robot_state)
{
  // Create a planning scene message
  moveit_msgs::msg::DisplayRobotState msg;
  moveit::core::robotStateToRobotStateMsg(robot_state, msg.state);

  // Publish!
  pub_robot_state_->publish(msg);
}

bool RobotPoses::checkSelfCollision(const moveit::core::RobotState& robot_state)
{
  // Decide if current state is in collision
  collision_detection::CollisionResult result;
  srdf_config_->getPlanningScene()->checkSelfCollision(request_, result, robot_state, allowed_collision_matrix_);
  return !result.contacts.empty();
}

std::vector<const moveit::core::JointModel*> RobotPoses::getSimpleJointModels(const std::string& group_name) const
{
  moveit::core::RobotModelPtr robot_model = srdf_config_->getRobotModel();
  if (!robot_model->hasJointModelGroup(group_name))
  {
    throw std::runtime_error(std::string("Unable to find joint model group for group: ") + group_name +
                             ". Are you sure this group has associated joints/links?");
  }

  const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(group_name);

  std::vector<const moveit::core::JointModel*> joint_models;
  for (const moveit::core::JointModel* joint_model : joint_model_group->getJointModels())
  {
    if (joint_model->getVariableCount() != 1 ||  // only consider 1-variable joints
        joint_model->isPassive() ||              // ignore passive
        joint_model->getMimic())                 // and mimic joints
      continue;

    joint_models.push_back(joint_model);
  }
  return joint_models;
}

void RobotPoses::setToCurrentValues(srdf::Model::GroupState& group_state)
{
  // Clear the old values (if any)
  group_state.joint_values_.clear();

  const auto& robot_state = srdf_config_->getPlanningScene()->getCurrentState();
  for (const moveit::core::JointModel* joint_model : getSimpleJointModels(group_state.group_))
  {
    // Create vector for new joint values
    std::vector<double> joint_values(joint_model->getVariableCount());
    const double* const first_variable = robot_state.getVariablePositions() + joint_model->getFirstVariableIndex();
    std::copy(first_variable, first_variable + joint_values.size(), joint_values.begin());

    // Add joint vector to SRDF
    group_state.joint_values_[joint_model->getName()] = std::move(joint_values);
  }
  srdf_config_->updateRobotModel(POSES);
}

}  // namespace srdf_setup
}  // namespace moveit_setup
