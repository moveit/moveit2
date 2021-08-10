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

#include <class_loader/class_loader.hpp>

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

namespace default_planner_request_adapters
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_ros.add_time_parameterization");

class AddTimeParameterization : public planning_request_adapter::PlanningRequestAdapter
{
public:
  AddTimeParameterization() : planning_request_adapter::PlanningRequestAdapter()
  {
  }

  void initialize(const rclcpp::Node::SharedPtr& /* node */, const std::string& /* parameter_namespace */) override
  {
  }

  std::string getDescription() const override
  {
    return "Add Time Parameterization";
  }

  bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
                    std::vector<std::size_t>& /*added_path_index*/) const override
  {
    bool result = planner(planning_scene, req, res);
    if (result && res.trajectory_)
    {
      RCLCPP_DEBUG(LOGGER, "Running '%s'", getDescription().c_str());
      if (!time_param_.computeTimeStamps(*res.trajectory_, req.max_velocity_scaling_factor,
                                         req.max_acceleration_scaling_factor))
      {
        RCLCPP_WARN(LOGGER, "Time parametrization for the solution path failed.");
        result = false;
      }
    }

    return result;
  }

private:
  trajectory_processing::IterativeParabolicTimeParameterization time_param_;
};
}  // namespace default_planner_request_adapters

CLASS_LOADER_REGISTER_CLASS(default_planner_request_adapters::AddTimeParameterization,
                            planning_request_adapter::PlanningRequestAdapter)
