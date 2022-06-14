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

/* Author: Ioan Sucan */

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <class_loader/class_loader.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter_value.hpp>

namespace default_planner_request_adapters
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_ros.fix_workspace_bounds");

class FixWorkspaceBounds : public planning_request_adapter::PlanningRequestAdapter
{
public:
  static const std::string WBOUNDS_PARAM_NAME;

  void initialize(const rclcpp::Node::SharedPtr& node, const std::string& parameter_namespace) override
  {
    node_ = node;
    workspace_extent_ = getParam(node_, LOGGER, parameter_namespace, WBOUNDS_PARAM_NAME, 10.0);
    workspace_extent_ /= 2.0;
  }

  std::string getDescription() const override
  {
    return "Fix Workspace Bounds";
  }

  bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
                    std::vector<std::size_t>& /*added_path_index*/) const override
  {
    RCLCPP_DEBUG(LOGGER, "Running '%s'", getDescription().c_str());
    const moveit_msgs::msg::WorkspaceParameters& wparams = req.workspace_parameters;
    if (wparams.min_corner.x == wparams.max_corner.x && wparams.min_corner.x == 0.0 &&
        wparams.min_corner.y == wparams.max_corner.y && wparams.min_corner.y == 0.0 &&
        wparams.min_corner.z == wparams.max_corner.z && wparams.min_corner.z == 0.0)
    {
      RCLCPP_DEBUG(LOGGER, "It looks like the planning volume was not specified. Using default values.");
      planning_interface::MotionPlanRequest req2 = req;
      moveit_msgs::msg::WorkspaceParameters& default_wp = req2.workspace_parameters;
      default_wp.min_corner.x = default_wp.min_corner.y = default_wp.min_corner.z = -workspace_extent_;
      default_wp.max_corner.x = default_wp.max_corner.y = default_wp.max_corner.z = workspace_extent_;
      return planner(planning_scene, req2, res);
    }
    else
      return planner(planning_scene, req, res);
  }

private:
  rclcpp::Node::SharedPtr node_;
  double workspace_extent_;
};

const std::string FixWorkspaceBounds::WBOUNDS_PARAM_NAME = "default_workspace_bounds";
}  // namespace default_planner_request_adapters

CLASS_LOADER_REGISTER_CLASS(default_planner_request_adapters::FixWorkspaceBounds,
                            planning_request_adapter::PlanningRequestAdapter)
