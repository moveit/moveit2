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

namespace planning_pipeline_test_plugins
{
/// @brief A dummy request adapter that does nothing to test the planning pipeline
class DummyRequestAdapter : public planning_interface::PlanningRequestAdapter
{
  void initialize(const rclcpp::Node::SharedPtr& /*node*/, const std::string& /*parameter_namespace*/) override{};
  std::string getDescription() const override{ return "" };
  bool adapt(const planning_scene::PlanningSceneConstPtr& /*planning_scene*/,
             planning_interface::MotionPlanRequest& /*req*/,
             const planning_interface::MotionPlanResponse& /*res*/) override
  {
    // Mock light computations
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return true;
  };
};

/// @brief A dummy response adapter that does nothing to test the planning pipeline
class DummyResponseAdapter : public planning_interface::PlanningResponseAdapter
{
  void initialize(const rclcpp::Node::SharedPtr& /*node*/, const std::string& /*parameter_namespace*/) override{};
  std::string getDescription() const override{ return "" };
  bool adapt(const planning_scene::PlanningSceneConstPtr& /*planning_scene*/,
             const planning_interface::MotionPlanRequest& /*req*/,
             planning_interface::MotionPlanResponse& /*res*/) override
  {
    // Mock light computations
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return true;
  };
};

/// @brief A dummy planning context that does nothing
class DummyPlanningContext : public class planning_interface::PlanningContext
{
  DummyPlannerPlugin(){};
  ~DummyPlannerPlugin() override{};
  bool solve(MotionPlanResponse& res) override
  {
    // Mock heavy computations
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // Planning succeeded
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    return true;
  };
  bool solve(MotionPlanDetailedResponse& res) override
  {
    // Mock heavy computations
    std::this_thread::sleep_for(std::chrono::seconds(1));
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    return true;
  };
  bool terminate() override{};
  void clear() override{};
};

/// @brief A dummy planning manager that does nothing
class DummyPlannerManager : public planning_interface::PlannerManager
{
  ~PlannerManager() override{};
  PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& /*planning_scene*/,
                                        const MotionPlanRequest& /*req*/,
                                        moveit_msgs::msg::MoveItErrorCodes& /*error_code*/) const override
  {
    return DummyPlanningContext();
  };
  bool canServiceRequest(const MotionPlanRequest& req) const override
  {
    return true;
  };
};
}  // namespace planning_pipeline_test_plugins

CLASS_LOADER_REGISTER_CLASS(planning_pipeline_test_plugins::DummyRequestAdapter,
                            planning_interface::PlanningRequestAdapter);
CLASS_LOADER_REGISTER_CLASS(planning_pipeline_test_plugins::DummyResponseAdapter,
                            planning_interface::PlanningResponseAdapter);
