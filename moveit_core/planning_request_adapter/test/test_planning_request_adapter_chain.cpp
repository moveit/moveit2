/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Hugo Laloge */

#include <gmock/gmock.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <moveit/planning_request_adapter/planning_request_adapter.h>

class AppendingPlanningRequestAdapter final : public planning_request_adapter::PlanningRequestAdapter
{
public:
  void initialize(const rclcpp::Node::SharedPtr& /*node*/, const std::string& /*parameter_namespace*/) override
  {
  }
  bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const planning_interface::MotionPlanRequest& req,
                    planning_interface::MotionPlanResponse& res) const override
  {
    bool success = planner(planning_scene, req, res);
    if (success)
    {
      res.trajectory->addSuffixWayPoint(res.trajectory->getLastWayPoint(), 1);
      res.added_path_index.push_back(res.trajectory->getWayPointCount() - 1);
    }
    return success;
  }
  std::string getDescription() const override
  {
    return "AppendingPlanningRequestAdapter";
  }
};

class PrependingPlanningRequestAdapter final : public planning_request_adapter::PlanningRequestAdapter
{
public:
  void initialize(const rclcpp::Node::SharedPtr& /*node*/, const std::string& /*parameter_namespace*/) override
  {
  }
  bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const planning_interface::MotionPlanRequest& req,
                    planning_interface::MotionPlanResponse& res) const override
  {
    bool success = planner(planning_scene, req, res);
    if (success)
    {
      res.trajectory->addPrefixWayPoint(res.trajectory->getFirstWayPoint(), 0);
      res.added_path_index.push_back(0);
    }
    return success;
  }
  std::string getDescription() const override
  {
    return "PrependingPlanningRequestAdapter";
  }
};

class PlannerManagerStub : public planning_interface::PlannerManager
{
public:
  PlannerManagerStub(planning_interface::PlanningContextPtr planningContext)
    : m_planningContext_{ std::move(planningContext) }
  {
  }
  planning_interface::PlanningContextPtr
  getPlanningContext(const planning_scene::PlanningSceneConstPtr& /*planning_scene*/,
                     const planning_interface::MotionPlanRequest& /*req*/,
                     moveit_msgs::msg::MoveItErrorCodes& /*error*/) const override
  {
    return m_planningContext_;
  }

  bool canServiceRequest(const planning_interface::MotionPlanRequest& /*req*/) const override
  {
    return true;
  }

private:
  planning_interface::PlanningContextPtr m_planningContext_;
};

class PlanningContextStub : public planning_interface::PlanningContext
{
public:
  using planning_interface::PlanningContext::PlanningContext;

  void setTrajectory(robot_trajectory::RobotTrajectoryPtr trajectory)
  {
    m_trajectory_ = std::move(trajectory);
  }

  bool solve(planning_interface::MotionPlanResponse& res) override
  {
    if (!m_trajectory_)
      return false;

    res.trajectory = m_trajectory_;
    return true;
  }

  bool solve(planning_interface::MotionPlanDetailedResponse& res) override
  {
    if (!m_trajectory_)
      return false;

    res.trajectory.push_back(m_trajectory_);
    return true;
  }

  bool terminate() override
  {
    return true;
  }

  void clear() override
  {
  }

private:
  robot_trajectory::RobotTrajectoryPtr m_trajectory_;
};

class PlanningRequestAdapterTests : public testing::Test
{
protected:
  void SetUp() override
  {
    robot_model_ = moveit::core::loadTestingRobotModel("panda");
    group_name_ = "panda_arm";
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
    planning_context_ = std::make_shared<PlanningContextStub>("stub", group_name_);
    planner_manager_ = std::make_shared<PlannerManagerStub>(planning_context_);
  }

  robot_trajectory::RobotTrajectoryPtr createRandomTrajectory(std::size_t waypointCount)
  {
    robot_trajectory::RobotTrajectoryPtr robot_trajectory =
        std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, group_name_);
    for (std::size_t i = 0; i < waypointCount; ++i)
      robot_trajectory->addSuffixWayPoint(std::make_shared<moveit::core::RobotState>(robot_model_), 1.);
    return robot_trajectory;
  }

protected:
  std::string group_name_;
  planning_request_adapter::PlanningRequestAdapterChain sut_;
  moveit::core::RobotModelPtr robot_model_;
  planning_scene::PlanningScenePtr planning_scene_;
  std::shared_ptr<PlanningContextStub> planning_context_;
  std::shared_ptr<PlannerManagerStub> planner_manager_;
};

TEST_F(PlanningRequestAdapterTests, testMergeAddedPathIndex)
{
  sut_.addAdapter(std::make_shared<AppendingPlanningRequestAdapter>());
  sut_.addAdapter(std::make_shared<PrependingPlanningRequestAdapter>());
  sut_.addAdapter(std::make_shared<AppendingPlanningRequestAdapter>());

  planning_context_->setTrajectory(createRandomTrajectory(4));

  planning_interface::MotionPlanRequest req;
  req.group_name = group_name_;
  planning_interface::MotionPlanResponse res;
  std::ignore = sut_.adaptAndPlan(planner_manager_, planning_scene_, req, res);

  EXPECT_THAT(res.added_path_index, testing::ElementsAre(0, 5, 6));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
