
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

/* Author: Sebastian Jahr
   Desc: TODO */

#include <moveit/pipeline_planning_interface/plan_responses_container.hpp>

namespace moveit
{
namespace planning_interface
{
PlanResponsesContainer::PlanResponsesContainer(const size_t expected_size)
{
  solutions_.reserve(expected_size);
}

/** \brief Thread safe method to add PlanResponsesContainer to this data structure TODO(sjahr): Refactor this method to
 * an insert method similar to https://github.com/ompl/ompl/blob/main/src/ompl/base/src/ProblemDefinition.cpp#L54-L161.
 * This way, it is possible to create a sorted container e.g. according to a user specified criteria
 */
void PlanResponsesContainer::pushBack(const ::planning_interface::MotionPlanResponse& plan_solution)
{
  std::lock_guard<std::mutex> lock_guard(solutions_mutex_);
  solutions_.push_back(plan_solution);
}

/** \brief Get solutions */
const std::vector<::planning_interface::MotionPlanResponse>& PlanResponsesContainer::getSolutions() const
{
  return solutions_;
}

}  // namespace planning_interface
}  // namespace moveit
