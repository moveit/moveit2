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
   Description: Defines events that could occur during hybrid planning
 */
#pragma once

namespace moveit::hybrid_planning
{
/**
 * Enum class HybridPlanningEvent - This class defines the most basic events that are likely to occur during hybrid planning
 */
enum class HybridPlanningEvent
{
  // Occurs when the hybrid planning manager receives a planning request
  HYBRID_PLANNING_REQUEST_RECEIVED,
  // Result of the global planning action
  GLOBAL_PLANNING_ACTION_SUCCESSFUL,
  GLOBAL_PLANNING_ACTION_ABORTED,
  GLOBAL_PLANNING_ACTION_CANCELED,
  // Indicates that the global planner found a solution (This solution is not necessarily the last or best solution)
  GLOBAL_SOLUTION_AVAILABLE,
  // Result of the local planning action
  LOCAL_PLANNING_ACTION_SUCCESSFUL,
  LOCAL_PLANNING_ACTION_ABORTED,
  LOCAL_PLANNING_ACTION_CANCELED,
  // Undefined event to allow empty reaction events to indicate failure
  UNDEFINED
};
}  // namespace moveit::hybrid_planning
