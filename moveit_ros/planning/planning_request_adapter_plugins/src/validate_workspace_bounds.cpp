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

/* Author: Ioan Sucan
 * Desc: If not specified by the planning request, this request adapter will specify a default workspace for planning.
 * The default workspace is a cube whose edge length is defined with a ROS 2 parameter.
 */

#include <moveit/planning_interface/planning_request_adapter.h>
#include <class_loader/class_loader.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <moveit/utils/logger.hpp>

#include <default_request_adapter_parameters.hpp>

namespace default_planning_request_adapters
{

/** @brief If not specified by the planning request, this request adapter will specify a default workspace for planning. */
class ValidateWorkspaceBounds : public planning_interface::PlanningRequestAdapter
{
public:
  ValidateWorkspaceBounds() : logger_(moveit::getLogger("validate_workspace_bounds"))
  {
  }

  void initialize(const rclcpp::Node::SharedPtr& node, const std::string& parameter_namespace) override
  {
    param_listener_ = std::make_unique<default_request_adapter_parameters::ParamListener>(node, parameter_namespace);
  }

  [[nodiscard]] std::string getDescription() const override
  {
    return std::string("ValidateWorkspaceBounds");
  }

  [[nodiscard]] moveit::core::MoveItErrorCode adapt(const planning_scene::PlanningSceneConstPtr& /*planning_scene*/,
                                                    planning_interface::MotionPlanRequest& req) const override
  {
    RCLCPP_DEBUG(logger_, "Running '%s'", getDescription().c_str());
    const moveit_msgs::msg::WorkspaceParameters& wparams = req.workspace_parameters;
    if (std::abs(wparams.min_corner.x) < std::numeric_limits<double>::epsilon() &&
        std::abs(wparams.min_corner.y) < std::numeric_limits<double>::epsilon() &&
        std::abs(wparams.min_corner.z) < std::numeric_limits<double>::epsilon() &&
        std::abs(wparams.max_corner.x) < std::numeric_limits<double>::epsilon() &&
        std::abs(wparams.max_corner.y) < std::numeric_limits<double>::epsilon() &&
        std::abs(wparams.max_corner.z) < std::numeric_limits<double>::epsilon())
    {
      RCLCPP_WARN(logger_, "It looks like the planning volume was not specified. Using default values.");
      moveit_msgs::msg::WorkspaceParameters& default_wp = req.workspace_parameters;
      const auto params = param_listener_->get_params();

      default_wp.min_corner.x = default_wp.min_corner.y = default_wp.min_corner.z =
          -params.default_workspace_bounds / 2.0;
      default_wp.max_corner.x = default_wp.max_corner.y = default_wp.max_corner.z =
          params.default_workspace_bounds / 2.0;
    }
    return moveit::core::MoveItErrorCode(moveit_msgs::msg::MoveItErrorCodes::SUCCESS, std::string(""), getDescription());
  }

private:
  std::unique_ptr<default_request_adapter_parameters::ParamListener> param_listener_;
  rclcpp::Logger logger_;
};
}  // namespace default_planning_request_adapters

CLASS_LOADER_REGISTER_CLASS(default_planning_request_adapters::ValidateWorkspaceBounds,
                            planning_interface::PlanningRequestAdapter)
