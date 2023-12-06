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
   Desc: Response adapter to display the motion path in RVIZ by publishing as EE pose marker array via ROS topic.
*/

#include <moveit/planning_interface/planning_response_adapter.h>
#include <moveit/robot_state/conversions.h>
#include <class_loader/class_loader.hpp>
#include <moveit/utils/logger.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <default_response_adapter_parameters.hpp>

namespace default_planning_response_adapters
{
/**
 * @brief Adapter to publish the EE path as marker array via ROS topic if a path exist. Otherwise, a warning is printed
 * but this adapter cannot fail.
 *
 */
class DisplayMotionPath : public planning_interface::PlanningResponseAdapter
{
public:
  DisplayMotionPath() : logger_(moveit::getLogger("display_motion_path"))
  {
  }

  void initialize(const rclcpp::Node::SharedPtr& node, const std::string& parameter_namespace) override
  {
    auto param_listener =
        std::make_unique<default_response_adapter_parameters::ParamListener>(node, parameter_namespace);
    // Read parameters
    const auto params = param_listener->get_params();
    display_path_publisher_ = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(params.display_path_topic,
                                                                                          rclcpp::SystemDefaultsQoS());
  }

  [[nodiscard]] std::string getDescription() const override
  {
    return std::string("DisplayMotionPath");
  }

  void adapt(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const planning_interface::MotionPlanRequest& /* req */,
             planning_interface::MotionPlanResponse& res) const override
  {
    RCLCPP_DEBUG(logger_, " Running '%s'", getDescription().c_str());
    if (res.trajectory)
    {
      moveit_msgs::msg::DisplayTrajectory disp;
      disp.model_id = planning_scene->getRobotModel()->getName();
      disp.trajectory.resize(1);
      res.trajectory->getRobotTrajectoryMsg(disp.trajectory.at(0));
      moveit::core::robotStateToRobotStateMsg(res.trajectory->getFirstWayPoint(), disp.trajectory_start);
      display_path_publisher_->publish(disp);
    }
    else
    {
      RCLCPP_WARN(logger_, "No motion path to display in MotionPlanResponse.");
    }
  }

private:
  rclcpp::Logger logger_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_path_publisher_;
};
}  // namespace default_planning_response_adapters

CLASS_LOADER_REGISTER_CLASS(default_planning_response_adapters::DisplayMotionPath,
                            planning_interface::PlanningResponseAdapter)
