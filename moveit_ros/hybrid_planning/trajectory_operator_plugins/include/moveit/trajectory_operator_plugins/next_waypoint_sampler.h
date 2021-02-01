/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

/* Author: Sebastian Jahr
   Description: Simple trajectory operator that samples the next global trajectory waypoint as local goal constraint based
   on the current robot state. When the waypoint is reached the index that marks the current local goal constraint is updated
   to the next global trajectory waypoint. Global trajectory updates are simply appended to the reference trajectory.
 */

#include <moveit/local_planner/trajectory_operator_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

namespace moveit_hybrid_planning
{
class NextWaypointSampler : public TrajectoryOperatorInterface
{
public:
  NextWaypointSampler(){};
  ~NextWaypointSampler() override{};

  bool initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelConstPtr& robot_model,
                  const std::string& group_name) override;
  bool addTrajectorySegment(const robot_trajectory::RobotTrajectory& new_trajectory) override;
  robot_trajectory::RobotTrajectory getLocalTrajectory(const moveit::core::RobotState& current_state) override;
  double getTrajectoryProgress(const moveit::core::RobotState& current_state) override;

private:
  std::size_t index_;
  trajectory_processing::TimeOptimalTrajectoryGeneration time_parametrization_;
};
}  // namespace moveit_hybrid_planning
