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

/* Author: Ioan Sucan, Sebastian Jahr
 * Desc: This adapter checks if the start state is in collision.
 */

#include <moveit/planning_interface/planning_request_adapter.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/trajectory_processing/trajectory_tools.hpp>
#include <class_loader/class_loader.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter_value.hpp>
#include <moveit/utils/logger.hpp>

#include <moveit_ros_planning/default_request_adapter_parameters.hpp>

namespace default_planning_request_adapters
{

/** @brief This adapter checks if the start state is in collision.*/
class CheckStartStateCollision : public planning_interface::PlanningRequestAdapter
{
public:
  CheckStartStateCollision() : logger_(moveit::getLogger("moveit.ros.validate_start_state"))
  {
  }

  ~CheckStartStateCollision() override = default;

  [[nodiscard]] std::string getDescription() const override
  {
    return std::string("CheckStartStateCollision");
  }

  [[nodiscard]] moveit::core::MoveItErrorCode adapt(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                    planning_interface::MotionPlanRequest& req) const override
  {
    RCLCPP_DEBUG(logger_, "Running '%s'", getDescription().c_str());

    // Get the start state TODO(sjahr): Should we check if req.start state == planning scene start state?
    moveit::core::RobotState start_state = planning_scene->getCurrentState();
    moveit::core::robotStateMsgToRobotState(planning_scene->getTransforms(), req.start_state, start_state);

    collision_detection::CollisionRequest creq;
    creq.group_name = req.group_name;
    collision_detection::CollisionResult cres;
    // TODO(sjahr): Would verbose make sense?
    planning_scene->checkCollision(creq, cres, start_state);

    auto status = moveit::core::MoveItErrorCode();
    if (!cres.collision)
    {
      status.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    }
    else
    {
      collision_detection::CollisionResult::ContactMap contacts;
      planning_scene->getCollidingPairs(contacts);

      std::string contact_information = std::to_string(contacts.size()) + " contact(s) detected : ";

      for (const auto& [contact_pair, contact_info] : contacts)
      {
        contact_information.append(contact_pair.first + " - " + contact_pair.second + ", ");
      }

      status.val = moveit_msgs::msg::MoveItErrorCodes::START_STATE_IN_COLLISION;
      status.message = std::string(contact_information);
    }
    status.source = getDescription();
    return status;
  }

private:
  rclcpp::Logger logger_;
};
}  // namespace default_planning_request_adapters

CLASS_LOADER_REGISTER_CLASS(default_planning_request_adapters::CheckStartStateCollision,
                            planning_interface::PlanningRequestAdapter)
