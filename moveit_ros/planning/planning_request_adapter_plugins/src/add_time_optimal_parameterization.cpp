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

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <class_loader/class_loader.hpp>

namespace default_planner_request_adapters
{
using namespace trajectory_processing;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_ros.add_time_optimal_parameterization");

/** @brief This adapter uses the time-optimal trajectory generation method */
class AddTimeOptimalParameterization : public planning_request_adapter::PlanningRequestAdapter
{
public:
  AddTimeOptimalParameterization() : planning_request_adapter::PlanningRequestAdapter()
  {
  }

  void initialize(const rclcpp::Node::SharedPtr& node, const std::string& parameter_namespace) override
  {
    path_tolerance_ = getParam(node, LOGGER, parameter_namespace, "path_tolerance", 0.1);
    resample_dt_ = getParam(node, LOGGER, parameter_namespace, "resample_dt", 0.1);
    min_angle_change_ = getParam(node, LOGGER, parameter_namespace, "min_angle_change", 0.001);
  }

  std::string getDescription() const override
  {
    return "Add Time Optimal Parameterization";
  }

  bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
                    std::vector<std::size_t>& /*added_path_index*/) const override
  {
    bool result = planner(planning_scene, req, res);
    if (result && res.trajectory)
    {
      RCLCPP_DEBUG(LOGGER, " Running '%s'", getDescription().c_str());
      TimeOptimalTrajectoryGeneration totg(path_tolerance_, resample_dt_, min_angle_change_);
      if (!totg.computeTimeStamps(*res.trajectory, req.max_velocity_scaling_factor, req.max_acceleration_scaling_factor))
      {
        RCLCPP_WARN(LOGGER, " Time parametrization for the solution path failed.");
        result = false;
      }
    }

    return result;
  }

protected:
  double path_tolerance_;
  double resample_dt_;
  double min_angle_change_;
};

}  // namespace default_planner_request_adapters

CLASS_LOADER_REGISTER_CLASS(default_planner_request_adapters::AddTimeOptimalParameterization,
                            planning_request_adapter::PlanningRequestAdapter)
