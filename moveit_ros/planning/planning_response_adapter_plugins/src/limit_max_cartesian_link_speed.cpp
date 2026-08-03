/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  Copyright (c) 2020, Benjamin Scholz
 *  Copyright (c) 2021, Thies Oelerich
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
 *   * Neither the name of the authors nor the names of other
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

/* Authors: Benjamin Scholz, Thies Oelerich, based off add_time_parameterization.cpp by Ioan Sucan */

#include <moveit/planning_interface/planning_response_adapter.hpp>
#include <moveit/trajectory_processing/limit_cartesian_speed.hpp>
#include <class_loader/class_loader.hpp>
#include <moveit/utils/logger.hpp>

#include <moveit_ros_planning/default_response_adapter_parameters.hpp>

namespace default_planning_response_adapters
{
using namespace trajectory_processing;

/** @brief This adapter uses the limit max Cartesian link speed method */
class LimitMaxCartesianLinkSpeed : public planning_interface::PlanningResponseAdapter
{
public:
  LimitMaxCartesianLinkSpeed() : logger_(moveit::getLogger("moveit.ros.limit_max_cartesian_link_speed"))
  {
  }

  ~LimitMaxCartesianLinkSpeed() override = default;

  void initialize(const rclcpp::Node::SharedPtr& /*node*/, const std::string& /*parameter_namespace*/) override
  {
  }

  [[nodiscard]] std::string getDescription() const override
  {
    return std::string("Limiting Max Cartesian Speed");
  }

  void adapt(const planning_scene::PlanningSceneConstPtr& /*planning_scene*/,
             const planning_interface::MotionPlanRequest& req,
             planning_interface::MotionPlanResponse& res) const override
  {
    RCLCPP_DEBUG(logger_, " Running '%s'", getDescription().c_str());
    if (!res.trajectory)
    {
      RCLCPP_ERROR(logger_, "Cannot apply response adapter '%s' because MotionPlanResponse does not contain a path.",
                   getDescription().c_str());
      res.error_code = moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN;
      return;
    }
    if (req.max_cartesian_speed <= 0.0)
      return;
    RCLCPP_DEBUG(logger_, "'%s' below '%f' [m/s] for link '%s'", getDescription().c_str(), req.max_cartesian_speed,
                 req.cartesian_speed_limited_link.c_str());
    if (!trajectory_processing::limitMaxCartesianLinkSpeed(*res.trajectory, req.max_cartesian_speed,
                                                           req.cartesian_speed_limited_link))
    {
      RCLCPP_ERROR(logger_, "Limiting Cartesian speed for the solution path failed.");
      res.error_code = moveit::core::MoveItErrorCode::FAILURE;
    }
    else
    {
      res.error_code = moveit::core::MoveItErrorCode::SUCCESS;
    }
  }

protected:
  // std::unique_ptr<default_response_adapter_parameters::ParamListener> param_listener_;
  rclcpp::Logger logger_;
};
}  // namespace default_planning_response_adapters

CLASS_LOADER_REGISTER_CLASS(default_planning_response_adapters::LimitMaxCartesianLinkSpeed,
                            planning_interface::PlanningResponseAdapter);