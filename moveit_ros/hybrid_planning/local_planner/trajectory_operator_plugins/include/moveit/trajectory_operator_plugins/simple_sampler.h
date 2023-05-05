/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
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

/* Author: Sebastian Jahr
   Description: Simple trajectory operator that samples the next global trajectory waypoint as local goal constraint
   based on the current robot state. When the waypoint is reached the index that marks the current local goal constraint
   is updated to the next global trajectory waypoint. Global trajectory updates simply replace the reference trajectory.
 */

#include <moveit/local_planner/trajectory_operator_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

namespace moveit::hybrid_planning
{
class SimpleSampler : public TrajectoryOperatorInterface
{
public:
  SimpleSampler() = default;
  ~SimpleSampler() override = default;

  bool initialize([[maybe_unused]] const rclcpp::Node::SharedPtr& node,
                  const moveit::core::RobotModelConstPtr& robot_model, const std::string& group_name) override;
  moveit_msgs::action::LocalPlanner::Feedback
  addTrajectorySegment(const robot_trajectory::RobotTrajectory& new_trajectory) override;
  moveit_msgs::action::LocalPlanner::Feedback
  getLocalTrajectory(const moveit::core::RobotState& current_state,
                     robot_trajectory::RobotTrajectory& local_trajectory) override;
  double getTrajectoryProgress([[maybe_unused]] const moveit::core::RobotState& current_state) override;
  bool reset() override;

private:
  std::size_t
      next_waypoint_index_;  // Indicates which reference trajectory waypoint is the current local goal constrained
  moveit_msgs::action::LocalPlanner::Feedback feedback_;  // Empty feedback
  trajectory_processing::TimeOptimalTrajectoryGeneration time_parametrization_;
  const moveit::core::JointModelGroup* joint_group_;
};
}  // namespace moveit::hybrid_planning
