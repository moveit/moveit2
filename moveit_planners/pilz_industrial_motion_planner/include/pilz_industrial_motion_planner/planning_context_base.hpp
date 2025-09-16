/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
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
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
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

#pragma once

#include <pilz_industrial_motion_planner/joint_limits_container.hpp>
#include <pilz_industrial_motion_planner/trajectory_generator.hpp>

#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_interface/planning_response.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/conversions.hpp>

#include <atomic>
#include <thread>

namespace pilz_industrial_motion_planner
{
/**
 * @brief PlanningContext for obtaining trajectories
 */
template <typename GeneratorT>
class PlanningContextBase : public planning_interface::PlanningContext
{
public:
  PlanningContextBase<GeneratorT>(const std::string& name, const std::string& group,
                                  const moveit::core::RobotModelConstPtr& model,
                                  const pilz_industrial_motion_planner::LimitsContainer& limits,
                                  const std::shared_ptr<interpolation::ParamListener>& interpolation_param_listener)
    : planning_interface::PlanningContext(name, group)
    , terminated_(false)
    , model_(model)
    , limits_(limits)
    , interpolation_param_listener_(interpolation_param_listener)
    , generator_(model, limits_, group)
  {
  }

  ~PlanningContextBase() override
  {
  }

  /**
   * @brief Calculates a trajectory for the request this context is currently
   * set for
   * @param res The result containing the respective trajectory, or error_code
   * on failure
   * @return true on success, false otherwise
   */
  void solve(planning_interface::MotionPlanResponse& res) override;

  /**
   * @brief Will return the same trajectory as
   * solve(planning_interface::MotionPlanResponse& res)
   * This function just delegates to the common response however here the same
   * trajectory is stored with the
   * descriptions "plan", "simplify", "interpolate"
   * @param res The detailed response
   * @return true on success, false otherwise
   */
  void solve(planning_interface::MotionPlanDetailedResponse& res) override;

  /**
   * @brief Will terminate solve()
   * @return
   * @note Currently will not stop a running solve but not start future solves.
   */
  bool terminate() override;

  /**
   * @copydoc planning_interface::PlanningContext::clear()
   */
  void clear() override;

  /// Flag if terminated
  std::atomic_bool terminated_;

  /// The robot model
  moveit::core::RobotModelConstPtr model_;

  /// Joint limits to be used during planning
  pilz_industrial_motion_planner::LimitsContainer limits_;

  /// Listener for interpolation parameters
  std::shared_ptr<interpolation::ParamListener> interpolation_param_listener_;

protected:
  GeneratorT generator_;
};

template <typename GeneratorT>
void pilz_industrial_motion_planner::PlanningContextBase<GeneratorT>::solve(planning_interface::MotionPlanResponse& res)
{
  if (terminated_)
  {
    RCLCPP_ERROR(rclcpp::get_logger("moveit.pilz_industrial_motion_planner.planning_context_base"),
                 "Using solve on a terminated planning context!");
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return;
  }
  generator_.generate(getPlanningScene(), request_, res, interpolation_param_listener_->get_params());
}

template <typename GeneratorT>
void pilz_industrial_motion_planner::PlanningContextBase<GeneratorT>::solve(
    planning_interface::MotionPlanDetailedResponse& res)
{
  // delegate to regular response
  planning_interface::MotionPlanResponse undetailed_response;
  solve(undetailed_response);

  res.description.push_back("plan");
  res.trajectory.push_back(undetailed_response.trajectory);
  res.processing_time.push_back(undetailed_response.planning_time);

  res.description.push_back("simplify");
  res.trajectory.push_back(undetailed_response.trajectory);
  res.processing_time.push_back(0);

  res.description.push_back("interpolate");
  res.trajectory.push_back(undetailed_response.trajectory);
  res.processing_time.push_back(0);

  res.error_code = undetailed_response.error_code;
}

template <typename GeneratorT>
bool pilz_industrial_motion_planner::PlanningContextBase<GeneratorT>::terminate()
{
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("moveit.pilz_industrial_motion_planner.planning_context_base"),
                      "Terminate called");
  terminated_ = true;
  return true;
}

template <typename GeneratorT>
void pilz_industrial_motion_planner::PlanningContextBase<GeneratorT>::clear()
{
  // No structures that need cleaning
  return;
}

}  // namespace pilz_industrial_motion_planner
