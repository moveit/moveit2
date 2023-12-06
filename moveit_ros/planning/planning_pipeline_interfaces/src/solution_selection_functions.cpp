/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, PickNik Inc.
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

/* Author: Sebastian Jahr */

#include <moveit/planning_pipeline_interfaces/solution_selection_functions.hpp>

namespace moveit
{
namespace planning_pipeline_interfaces
{
::planning_interface::MotionPlanResponse
getShortestSolution(const std::vector<::planning_interface::MotionPlanResponse>& solutions)
{
  // Find trajectory with minimal path
  const auto shortest_trajectory = std::min_element(solutions.begin(), solutions.end(),
                                                    [](const ::planning_interface::MotionPlanResponse& solution_a,
                                                       const ::planning_interface::MotionPlanResponse& solution_b) {
                                                      // If both solutions were successful, check which path is shorter
                                                      if (solution_a && solution_b)
                                                      {
                                                        return robot_trajectory::pathLength(*solution_a.trajectory) <
                                                               robot_trajectory::pathLength(*solution_b.trajectory);
                                                      }
                                                      // If only solution a is successful, return a
                                                      else if (solution_a)
                                                      {
                                                        return true;
                                                      }
                                                      // Else return solution b, either because it is successful or not
                                                      return false;
                                                    });
  return *shortest_trajectory;
}

}  // namespace planning_pipeline_interfaces
}  // namespace moveit
