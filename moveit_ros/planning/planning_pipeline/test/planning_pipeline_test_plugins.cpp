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
#include <cmath>
#include <memory>
#include <vector>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_interface/planning_request_adapter.hpp>
#include <moveit/planning_interface/planning_response_adapter.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
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

/// @brief The joint positions per waypoint of the trajectory created by TrajectoryPlannerManager
const std::vector<std::vector<double>> TRAJECTORY_WAYPOINTS{ { 0.1, 0.2 }, { 0.3, 0.4 } };

/// @brief A dummy planning context that returns the trajectory defined by TRAJECTORY_WAYPOINTS
class TrajectoryPlanningContext : public planning_interface::PlanningContext
{
public:
  TrajectoryPlanningContext(const moveit::core::RobotModelConstPtr& robot_model, const std::string& group_name)
    : planning_interface::PlanningContext("TrajectoryPlanningContext", group_name), robot_model_(robot_model)
  {
  }
  void solve(planning_interface::MotionPlanResponse& res) override
  {
    res.trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, getGroupName());
    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();
    for (const auto& waypoint : TRAJECTORY_WAYPOINTS)
    {
      state.setJointGroupPositions(getGroupName(), waypoint);
      res.trajectory->addSuffixWayPoint(state, 0.1);
    }
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }
  void solve(planning_interface::MotionPlanDetailedResponse& res) override
  {
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }
  bool terminate() override
  {
    return true;
  }
  void clear() override{};

private:
  moveit::core::RobotModelConstPtr robot_model_;
};

/// @brief A dummy planning context that reports a fixed error code
class FixedResultPlanningContext : public planning_interface::PlanningContext
{
public:
  explicit FixedResultPlanningContext(int32_t error_code)
    : planning_interface::PlanningContext("FixedResultPlanningContext", "empty_group"), error_code_(error_code)
  {
  }
  void solve(planning_interface::MotionPlanResponse& res) override
  {
    res.error_code.val = error_code_;
  }
  void solve(planning_interface::MotionPlanDetailedResponse& res) override
  {
    res.error_code.val = error_code_;
  }
  bool terminate() override
  {
    return true;
  }
  void clear() override{};

private:
  int32_t error_code_;
};

/// @brief A dummy planning manager whose planner always returns the trajectory defined by TRAJECTORY_WAYPOINTS
class TrajectoryPlannerManager : public planning_interface::PlannerManager
{
public:
  planning_interface::PlanningContextPtr
  getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                     const planning_interface::MotionPlanRequest& req,
                     moveit_msgs::msg::MoveItErrorCodes& /*error_code*/) const override
  {
    return std::make_shared<TrajectoryPlanningContext>(planning_scene->getRobotModel(), req.group_name);
  }
  bool canServiceRequest(const planning_interface::MotionPlanRequest& /*req*/) const override
  {
    return true;
  }
  std::string getDescription() const override
  {
    return std::string("TrajectoryPlannerManager");
  }
};

/// @brief A dummy planning manager whose planner only succeeds if the request contains one joint constraint set per
/// waypoint of TRAJECTORY_WAYPOINTS
class CheckTrajectoryConstraintsPlannerManager : public planning_interface::PlannerManager
{
public:
  planning_interface::PlanningContextPtr
  getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                     const planning_interface::MotionPlanRequest& req,
                     moveit_msgs::msg::MoveItErrorCodes& /*error_code*/) const override
  {
    const auto* group = planning_scene->getRobotModel()->getJointModelGroup(req.group_name);
    const auto& joint_names = group->getActiveJointModelNames();
    const auto& constraints = req.trajectory_constraints.constraints;

    bool matches = constraints.size() == TRAJECTORY_WAYPOINTS.size();
    for (size_t i = 0; matches && i < constraints.size(); ++i)
    {
      const auto& joint_constraints = constraints[i].joint_constraints;
      matches = joint_constraints.size() == joint_names.size();
      for (size_t j = 0; matches && j < joint_constraints.size(); ++j)
      {
        matches = joint_constraints[j].joint_name == joint_names[j] &&
                  std::abs(joint_constraints[j].position - TRAJECTORY_WAYPOINTS[i][j]) < 1e-9;
      }
    }

    const int32_t result =
        matches ? moveit_msgs::msg::MoveItErrorCodes::SUCCESS : moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return std::make_shared<FixedResultPlanningContext>(result);
  }
  bool canServiceRequest(const planning_interface::MotionPlanRequest& /*req*/) const override
  {
    return true;
  }
  std::string getDescription() const override
  {
    return std::string("CheckTrajectoryConstraintsPlannerManager");
  }
};
}  // namespace planning_pipeline_test

CLASS_LOADER_REGISTER_CLASS(planning_pipeline_test::DummyPlannerManager, planning_interface::PlannerManager)
CLASS_LOADER_REGISTER_CLASS(planning_pipeline_test::TrajectoryPlannerManager, planning_interface::PlannerManager)
CLASS_LOADER_REGISTER_CLASS(planning_pipeline_test::CheckTrajectoryConstraintsPlannerManager,
                            planning_interface::PlannerManager)
CLASS_LOADER_REGISTER_CLASS(planning_pipeline_test::AlwaysSuccessRequestAdapter,
                            planning_interface::PlanningRequestAdapter)
CLASS_LOADER_REGISTER_CLASS(planning_pipeline_test::AlwaysSuccessResponseAdapter,
                            planning_interface::PlanningResponseAdapter)
