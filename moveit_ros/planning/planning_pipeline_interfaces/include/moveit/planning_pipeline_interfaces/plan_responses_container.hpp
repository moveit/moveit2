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
   Desc: A thread safe container to store motion plan responses */

#pragma once

#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_interface/planning_request.h>

namespace moveit
{
namespace planning_pipeline_interfaces
{
MOVEIT_CLASS_FORWARD(PlanResponsesContainer);  // Defines PlanningComponentPtr, ConstPtr, WeakPtr... etc
/** \brief A container to thread-safely store multiple MotionPlanResponses */
class PlanResponsesContainer
{
public:
  /** \brief Constructor
   * \param [in] expected_size Number of expected solutions
   */
  PlanResponsesContainer(const size_t expected_size = 0);

  /** \brief Thread safe method to add PlanResponsesContainer to this data structure TODO(sjahr): Refactor this method to an
   * insert method similar to https://github.com/ompl/ompl/blob/main/src/ompl/base/src/ProblemDefinition.cpp#L54-L161.
   * This way, it is possible to create a sorted container e.g. according to a user specified criteria
   * \param [in] plan_solution MotionPlanResponse to push back into the vector
   */
  void pushBack(const ::planning_interface::MotionPlanResponse& plan_solution);

  /** \brief Get solutions
   * \return Read-only access to the responses vector
   */
  const std::vector<::planning_interface::MotionPlanResponse>& getSolutions() const;

private:
  std::vector<::planning_interface::MotionPlanResponse> solutions_;
  std::mutex solutions_mutex_;
};
}  // namespace planning_pipeline_interfaces
}  // namespace moveit
