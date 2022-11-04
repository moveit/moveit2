/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <class_loader/class_loader.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter_value.hpp>

namespace default_planner_request_adapters
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_ros.fix_start_state_collision");

class FixStartStateCollision : public planning_request_adapter::PlanningRequestAdapter
{
public:
  static const std::string DT_PARAM_NAME;
  static const std::string JIGGLE_PARAM_NAME;
  static const std::string ATTEMPTS_PARAM_NAME;

  void initialize(const rclcpp::Node::SharedPtr& node, const std::string& parameter_namespace) override
  {
    node_ = node;
    max_dt_offset_ = getParam(node_, LOGGER, parameter_namespace, DT_PARAM_NAME, 0.5);
    jiggle_fraction_ = getParam(node_, LOGGER, parameter_namespace, JIGGLE_PARAM_NAME, 0.02);
    sampling_attempts_ = getParam(node_, LOGGER, parameter_namespace, ATTEMPTS_PARAM_NAME, 100);
    if (sampling_attempts_ < 1)
    {
      sampling_attempts_ = 1;
      RCLCPP_WARN(LOGGER, "Param '%s' needs to be at least 1.", ATTEMPTS_PARAM_NAME.c_str());
    }
  }

  std::string getDescription() const override
  {
    return "Fix Start State In Collision";
  }

  bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
                    std::vector<std::size_t>& added_path_index) const override
  {
    RCLCPP_DEBUG(LOGGER, "Running '%s'", getDescription().c_str());

    // get the specified start state
    moveit::core::RobotState start_state = planning_scene->getCurrentState();
    moveit::core::robotStateMsgToRobotState(planning_scene->getTransforms(), req.start_state, start_state);

    collision_detection::CollisionRequest creq;
    creq.group_name = req.group_name;
    collision_detection::CollisionResult cres;
    planning_scene->checkCollision(creq, cres, start_state);
    if (cres.collision)
    {
      // Rerun in verbose mode
      collision_detection::CollisionRequest vcreq = creq;
      collision_detection::CollisionResult vcres;
      vcreq.verbose = true;
      planning_scene->checkCollision(vcreq, vcres, start_state);

      if (creq.group_name.empty())
        RCLCPP_INFO(LOGGER, "Start state appears to be in collision");
      else
        RCLCPP_INFO(LOGGER, "Start state appears to be in collision with respect to group %s", creq.group_name.c_str());

      auto prefix_state = std::make_shared<moveit::core::RobotState>(start_state);
      random_numbers::RandomNumberGenerator& rng = prefix_state->getRandomNumberGenerator();

      const std::vector<const moveit::core::JointModel*>& jmodels =
          planning_scene->getRobotModel()->hasJointModelGroup(req.group_name) ?
              planning_scene->getRobotModel()->getJointModelGroup(req.group_name)->getJointModels() :
              planning_scene->getRobotModel()->getJointModels();

      bool found = false;
      for (int c = 0; !found && c < sampling_attempts_; ++c)
      {
        for (std::size_t i = 0; !found && i < jmodels.size(); ++i)
        {
          std::vector<double> sampled_variable_values(jmodels[i]->getVariableCount());
          const double* original_values = prefix_state->getJointPositions(jmodels[i]);
          jmodels[i]->getVariableRandomPositionsNearBy(rng, &sampled_variable_values[0], original_values,
                                                       jmodels[i]->getMaximumExtent() * jiggle_fraction_);
          start_state.setJointPositions(jmodels[i], sampled_variable_values);
          collision_detection::CollisionResult cres;
          planning_scene->checkCollision(creq, cres, start_state);
          if (!cres.collision)
          {
            found = true;
            RCLCPP_INFO(LOGGER, "Found a valid state near the start state at distance %lf after %d attempts",
                        prefix_state->distance(start_state), c);
          }
        }
      }

      if (found)
      {
        planning_interface::MotionPlanRequest req2 = req;
        moveit::core::robotStateToRobotStateMsg(start_state, req2.start_state);
        bool solved = planner(planning_scene, req2, res);
        if (solved && !res.trajectory_->empty())
        {
          // heuristically decide a duration offset for the trajectory (induced by the additional point added as a
          // prefix to the computed trajectory)
          res.trajectory_->setWayPointDurationFromPrevious(0, std::min(max_dt_offset_,
                                                                       res.trajectory_->getAverageSegmentDuration()));
          res.trajectory_->addPrefixWayPoint(prefix_state, 0.0);
          // we add a prefix point, so we need to bump any previously added index positions
          for (std::size_t& added_index : added_path_index)
            added_index++;
          added_path_index.push_back(0);
        }
        return solved;
      }
      else
      {
        RCLCPP_WARN(LOGGER,
                    "Unable to find a valid state nearby the start state (using jiggle fraction of %lf and %u sampling "
                    "attempts). Passing the original planning request to the planner.",
                    jiggle_fraction_, sampling_attempts_);
        return planner(planning_scene, req, res);
      }
    }
    else
    {
      if (creq.group_name.empty())
        RCLCPP_DEBUG(LOGGER, "Start state is valid");
      else
        RCLCPP_DEBUG(LOGGER, "Start state is valid with respect to group %s", creq.group_name.c_str());
      return planner(planning_scene, req, res);
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  double max_dt_offset_;
  double jiggle_fraction_;
  int sampling_attempts_;
};

const std::string FixStartStateCollision::DT_PARAM_NAME = "start_state_max_dt";
const std::string FixStartStateCollision::JIGGLE_PARAM_NAME = "jiggle_fraction";
const std::string FixStartStateCollision::ATTEMPTS_PARAM_NAME = "max_sampling_attempts";
}  // namespace default_planner_request_adapters

CLASS_LOADER_REGISTER_CLASS(default_planner_request_adapters::FixStartStateCollision,
                            planning_request_adapter::PlanningRequestAdapter)
