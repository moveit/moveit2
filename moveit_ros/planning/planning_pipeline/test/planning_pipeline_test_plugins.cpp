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

#include <chrono>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_interface/planning_request_adapter.h>
#include <moveit/planning_interface/planning_response_adapter.h>
#include <class_loader/class_loader.hpp>

namespace planning_pipeline_test
{
/// @brief A dummy request adapter that does nothing and is always successful
class AlwaysSuccessRequestAdapter : public planning_interface::PlanningRequestAdapter
{
public:
  std::string getDescription() const override
  {
    return "AlwaysSuccessRequestAdapter";
  }
  moveit::core::MoveItErrorCode adapt(const planning_scene::PlanningSceneConstPtr& /*planning_scene*/,
                                      planning_interface::MotionPlanRequest& /*req*/) const override
  {
    // Mock light computations
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return moveit::core::MoveItErrorCode(moveit_msgs::msg::MoveItErrorCodes::SUCCESS, std::string(""), getDescription());
  }
};

/// @brief A dummy response adapter that does nothing and is always successful
class AlwaysSuccessResponseAdapter : public planning_interface::PlanningResponseAdapter
{
public:
  std::string getDescription() const override
  {
    return "AlwaysSuccessResponseAdapter";
  }
  void adapt(const planning_scene::PlanningSceneConstPtr& /*planning_scene*/,
             const planning_interface::MotionPlanRequest& /*req*/,
             planning_interface::MotionPlanResponse& res) const override
  {
    // Mock light computations
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }
};

/// @brief A dummy planning context that does nothing and is always successful
class DummyPlanningContext : public planning_interface::PlanningContext
{
public:
  DummyPlanningContext() : planning_interface::PlanningContext("DummyPlanningContext", "empty_group")
  {
  }
  void solve(planning_interface::MotionPlanResponse& res) override
  {
    // Mock heavy computations
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // Planning succeeded
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }
  void solve(planning_interface::MotionPlanDetailedResponse& res) override
  {
    // Mock heavy computations
    std::this_thread::sleep_for(std::chrono::seconds(1));
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }
  bool terminate() override
  {
    return true;
  }
  void clear() override{};
};

/// @brief A dummy planning manager that does nothing
class DummyPlannerManager : public planning_interface::PlannerManager
{
public:
  ~DummyPlannerManager() override
  {
  }
  planning_interface::PlanningContextPtr
  getPlanningContext(const planning_scene::PlanningSceneConstPtr& /*planning_scene*/,
                     const planning_interface::MotionPlanRequest& /*req*/,
                     moveit_msgs::msg::MoveItErrorCodes& /*error_code*/) const override
  {
    return std::make_shared<DummyPlanningContext>();
  }
  bool canServiceRequest(const planning_interface::MotionPlanRequest& /*req*/) const override
  {
    return true;
  }
  std::string getDescription() const override
  {
    return std::string("DummyPlannerManager");
  }
};
}  // namespace planning_pipeline_test

CLASS_LOADER_REGISTER_CLASS(planning_pipeline_test::DummyPlannerManager, planning_interface::PlannerManager)
CLASS_LOADER_REGISTER_CLASS(planning_pipeline_test::AlwaysSuccessRequestAdapter,
                            planning_interface::PlanningRequestAdapter)
CLASS_LOADER_REGISTER_CLASS(planning_pipeline_test::AlwaysSuccessResponseAdapter,
                            planning_interface::PlanningResponseAdapter)
