/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, PickNik Inc.
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
 *   * Neither the name of PickNik Inc. nor the names of its
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
   Desc: Response adapter that checks a path for validity (collision avoidance, feasibility and constraint satisfaction)
*/

#include <moveit/planning_interface/planning_response_adapter.h>
#include <class_loader/class_loader.hpp>
#include <moveit/utils/logger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <moveit/collision_detection/collision_tools.h>

#include <default_response_adapter_parameters.hpp>
namespace default_planning_response_adapters
{
/**
 * @brief Adapter to check the request path validity (collision avoidance, feasibility and constraint satisfaction)
 *
 */
class ValidateSolution : public planning_interface::PlanningResponseAdapter
{
public:
  ValidateSolution() : logger_(moveit::getLogger("moveit.ros.validate_solution"))
  {
  }

  void initialize(const rclcpp::Node::SharedPtr& node, const std::string& parameter_namespace) override
  {
    auto param_listener =
        std::make_unique<default_response_adapter_parameters::ParamListener>(node, parameter_namespace);
    // Read parameters
    const auto params = param_listener->get_params();

    if (!params.display_contacts_topic.empty())
    {
      contacts_publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(params.display_contacts_topic,
                                                                                         rclcpp::SystemDefaultsQoS());
    }
  }

  [[nodiscard]] std::string getDescription() const override
  {
    return std::string("ValidateSolution");
  }

  void adapt(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const planning_interface::MotionPlanRequest& req,
             planning_interface::MotionPlanResponse& res) const override
  {
    RCLCPP_DEBUG(logger_, " Running '%s'", getDescription().c_str());
    if (!res.trajectory)
    {
      RCLCPP_ERROR(logger_, "No motion path to display in MotionPlanResponse.");
      res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
      return;
    }

    std::size_t state_count = res.trajectory->getWayPointCount();
    RCLCPP_DEBUG(logger_, "Motion planner reported a solution path with %ld states", state_count);
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker m;
    m.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(m);

    std::vector<std::size_t> indices;
    if (!planning_scene->isPathValid(*res.trajectory, req.path_constraints, req.group_name, false, &indices))
    {
      // check to see if there is any problem with the states that are found to be invalid
      res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;

      // If a contact publisher exists, publish contacts
      if (contacts_publisher_)
      {
        // display error messages
        std::stringstream ss;
        for (std::size_t it : indices)
        {
          ss << it << ' ';
        }

        RCLCPP_ERROR_STREAM(logger_, "Computed path is not valid. Invalid states at index locations: [ "
                                         << ss.str() << "] out of " << state_count
                                         << ". Explanations follow in command line. Contacts are published on "
                                         << contacts_publisher_->get_topic_name());

        // call validity checks in verbose mode for the problematic states
        for (std::size_t it : indices)
        {
          // check validity with verbose on
          const moveit::core::RobotState& robot_state = res.trajectory->getWayPoint(it);
          planning_scene->isStateValid(robot_state, req.path_constraints, req.group_name, true);

          // compute the contacts if any
          collision_detection::CollisionRequest c_req;
          collision_detection::CollisionResult c_res;
          c_req.contacts = true;
          c_req.max_contacts = 10;
          c_req.max_contacts_per_pair = 3;
          c_req.verbose = false;
          planning_scene->checkCollision(c_req, c_res, robot_state);
          if (c_res.contact_count > 0)
          {
            visualization_msgs::msg::MarkerArray arr_i;
            collision_detection::getCollisionMarkersFromContacts(arr_i, planning_scene->getPlanningFrame(),
                                                                 c_res.contacts);
            arr.markers.insert(arr.markers.end(), arr_i.markers.begin(), arr_i.markers.end());
          }
        }
        RCLCPP_ERROR(logger_, "Completed listing of explanations for invalid states.");
        contacts_publisher_->publish(arr);
      }
    }
  }

private:
  rclcpp::Logger logger_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr contacts_publisher_;
};
}  // namespace default_planning_response_adapters

CLASS_LOADER_REGISTER_CLASS(default_planning_response_adapters::ValidateSolution,
                            planning_interface::PlanningResponseAdapter)
