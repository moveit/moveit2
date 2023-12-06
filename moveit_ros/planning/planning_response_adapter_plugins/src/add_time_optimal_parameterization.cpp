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

/* Author: Ioan Sucan, Michael Ferguson */

#include <moveit/planning_interface/planning_response_adapter.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <class_loader/class_loader.hpp>
#include <moveit/utils/logger.hpp>

#include <default_response_adapter_parameters.hpp>

namespace default_planning_response_adapters
{
using namespace trajectory_processing;

/** @brief This adapter uses the time-optimal trajectory generation method */
class AddTimeOptimalParameterization : public planning_interface::PlanningResponseAdapter
{
public:
  AddTimeOptimalParameterization() : logger_(moveit::getLogger("add_time_optimal_parameterization"))
  {
  }

  void initialize(const rclcpp::Node::SharedPtr& node, const std::string& parameter_namespace) override
  {
    param_listener_ = std::make_unique<default_response_adapter_parameters::ParamListener>(node, parameter_namespace);
  }

  [[nodiscard]] std::string getDescription() const override
  {
    return std::string("AddTimeOptimalParameterization");
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

    const auto params = param_listener_->get_params().totg;
    TimeOptimalTrajectoryGeneration totg(params.path_tolerance, params.resample_dt, params.min_angle_change);
    if (totg.computeTimeStamps(*res.trajectory, req.max_velocity_scaling_factor, req.max_acceleration_scaling_factor))
    {
      res.error_code = moveit::core::MoveItErrorCode::SUCCESS;
    }
    else
    {
      RCLCPP_ERROR(logger_, "Response adapter '%s' failed to generate a trajectory.", getDescription().c_str());
      res.error_code = moveit::core::MoveItErrorCode::FAILURE;
    }
  }

protected:
  std::unique_ptr<default_response_adapter_parameters::ParamListener> param_listener_;
  rclcpp::Logger logger_;
};

}  // namespace default_planning_response_adapters

CLASS_LOADER_REGISTER_CLASS(default_planning_response_adapters::AddTimeOptimalParameterization,
                            planning_interface::PlanningResponseAdapter)
