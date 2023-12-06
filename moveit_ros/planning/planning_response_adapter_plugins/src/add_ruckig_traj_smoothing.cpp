/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, PickNik Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/* Author: Jack Center, Wyatt Rees, Andy Zelenak
 * Desc:  An adapter that uses ruckig (https://github.com/pantor/ruckig) to adapt the trajectory to be jerk-constrained and
 * time-optimal. It is necessary to run another trajectory generation algorithm before this adapter, because the
 * algorithm requires a fully configured trajectory as initial guess.
 */

#include <moveit/utils/logger.hpp>
#include <moveit/planning_interface/planning_response_adapter.h>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#include <class_loader/class_loader.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

namespace default_planning_response_adapters
{
using namespace trajectory_processing;

/** @brief Use ruckig (https://github.com/pantor/ruckig) to adapt the trajectory to be jerk-constrained and
 * time-optimal.*/
class AddRuckigTrajectorySmoothing : public planning_interface::PlanningResponseAdapter
{
public:
  AddRuckigTrajectorySmoothing() : logger_(moveit::getLogger("add_ruckig_trajectory_smoothing"))
  {
  }

  [[nodiscard]] std::string getDescription() const override
  {
    return std::string("AddRuckigTrajectorySmoothing");
  }

  void adapt(const planning_scene::PlanningSceneConstPtr& /*planning_scene*/,
             const planning_interface::MotionPlanRequest& req,
             planning_interface::MotionPlanResponse& res) const override
  {
    RCLCPP_DEBUG(logger_, " Running '%s'", getDescription().c_str());
    if (!res.trajectory)
    {
      RCLCPP_ERROR(
          logger_,
          "Cannot apply response adapter '%s' because MotionPlanResponse does not contain a trajectory to smooth.",
          getDescription().c_str());
      res.error_code = moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN;
      return;
    }

    if (smoother_.applySmoothing(*res.trajectory, req.max_velocity_scaling_factor, req.max_acceleration_scaling_factor))
    {
      res.error_code = moveit::core::MoveItErrorCode::SUCCESS;
    }
    else
    {
      RCLCPP_ERROR(logger_, "Response adapter '%s' failed to smooth trajectory.", getDescription().c_str());
      res.error_code = moveit::core::MoveItErrorCode::FAILURE;
    }
  }

private:
  RuckigSmoothing smoother_;
  rclcpp::Logger logger_;
};

}  // namespace default_planning_response_adapters

CLASS_LOADER_REGISTER_CLASS(default_planning_response_adapters::AddRuckigTrajectorySmoothing,
                            planning_interface::PlanningResponseAdapter)
