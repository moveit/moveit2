/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  Copyright (c) 2018, Michael 'v4hn' Goerner
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

/* Authors: Ioan Sucan, Michael Goerner */

#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/ompl_interface/detail/constraints_library.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/utils/message_checks.h>

#include <boost/math/constants/constants.hpp>
#include <sstream>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit.planners_ompl.generate_state_database");

static const std::string ROBOT_DESCRIPTION = "robot_description";

static const std::string CONSTRAINT_PARAMETER = "constraints";

static bool get_uint_parameter_or(const rclcpp::Node::SharedPtr& node, const std::string& param_name,
                                  size_t&& result_value, const size_t default_value)
{
  int param_value;
  if (node->get_parameter(param_name, param_value))
  {
    if (param_value >= 0)
    {
      result_value = static_cast<size_t>(param_value);
      return true;
    }

    RCLCPP_WARN_STREAM(LOGGER, "Value for parameter '" << param_name
                                                       << "' must be positive\n"
                                                          "Using back to default value:"
                                                       << default_value);
  }

  result_value = default_value;
  return true;
}

struct GenerateStateDatabaseParameters
{
  bool setFromNode(const rclcpp::Node::SharedPtr& node)
  {
    node->get_parameter_or("use_current_scene", use_current_scene, false);

    // number of states in joint space approximation
    get_uint_parameter_or(node, "state_cnt", construction_opts.samples, 10000);

    // generate edges together with states?
    get_uint_parameter_or(node, "edges_per_sample", construction_opts.edges_per_sample, 0);

    node->get_parameter_or("max_edge_length", construction_opts.max_edge_length, 0.2);

    // verify constraint validity on edges
    node->get_parameter_or("explicit_motions", construction_opts.explicit_motions, true);
    node->get_parameter_or("explicit_points_resolution", construction_opts.explicit_points_resolution, 0.05);
    get_uint_parameter_or(node, "max_explicit_points", construction_opts.max_explicit_points, 200);

    // local planning in JointModel state space
    node->get_parameter_or("state_space_parameterization", construction_opts.state_space_parameterization,
                           std::string("JointModel"));

    node->get_parameter_or("output_folder", output_folder, std::string("constraint_approximation_database"));

    if (!node->get_parameter("planning_group", planning_group))
    {
      RCLCPP_FATAL(LOGGER, "~planning_group parameter has to be specified.");
      return false;
    }

    if (!kinematic_constraints::constructConstraints(node, CONSTRAINT_PARAMETER, constraints))
    {
      RCLCPP_FATAL_STREAM(LOGGER,
                          "Could not find valid constraint description in parameter '"
                              << node->get_fully_qualified_name() << '.' << CONSTRAINT_PARAMETER
                              << "'. "
                                 "Please upload correct correct constraint description or remap the parameter.");
      return false;
    }

    return true;
  };

  std::string planning_group;

  // path to folder for generated database
  std::string output_folder;

  // request the current scene via get_planning_scene service
  bool use_current_scene;

  // constraints the approximation should satisfy
  moveit_msgs::msg::Constraints constraints;

  // internal parameters of approximation generator
  ompl_interface::ConstraintApproximationConstructionOptions construction_opts;
};

void computeDB(const rclcpp::Node::SharedPtr& node, const planning_scene::PlanningScenePtr& scene,
               struct GenerateStateDatabaseParameters& params)
{
  // required by addConstraintApproximation
  scene->getCurrentStateNonConst().update();

  ompl_interface::OMPLInterface ompl_interface(scene->getRobotModel(), node, "ompl");
  planning_interface::MotionPlanRequest req;
  req.group_name = params.planning_group;
  req.path_constraints = params.constraints;
  moveit::core::robotStateToRobotStateMsg(scene->getCurrentState(), req.start_state);
  req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(
      scene->getCurrentState(), scene->getRobotModel()->getJointModelGroup(params.planning_group)));

  ompl_interface::ModelBasedPlanningContextPtr context = ompl_interface.getPlanningContext(scene, req);

  RCLCPP_INFO_STREAM(LOGGER, "Generating Joint Space Constraint Approximation Database for constraint:\n"
                                 << params.constraints.name);

  ompl_interface::ConstraintApproximationConstructionResults result =
      context->getConstraintsLibraryNonConst()->addConstraintApproximation(params.constraints, params.planning_group,
                                                                           scene, params.construction_opts);

  if (!result.approx)
  {
    RCLCPP_FATAL(LOGGER, "Failed to generate approximation.");
    return;
  }
  context->getConstraintsLibraryNonConst()->saveConstraintApproximations(params.output_folder);
  RCLCPP_INFO_STREAM(LOGGER, "Successfully generated Joint Space Constraint Approximation Database for constraint:\n"
                                 << params.constraints.name);
  RCLCPP_INFO_STREAM(LOGGER, "The database has been saved in your local folder '" << params.output_folder << '\'');
}

/**
 * Generates a database of states that follow the given constraints.
 * An example of the constraint yaml that should be loaded to rosparam:
 * """
 * name: constraint_name
 * constraint_ids: [constraint_1, constraint_2]
 * constraints:
 *   constraint_1:
 *     type: orientation
 *     frame_id: world
 *     link_name: tool0
 *     orientation: [0, 0, 0]  # [r, p, y]
 *     tolerances: [0.01, 0.01, 3.15]
 *     weight: 1.0
 *   constraint_2:
 *     type: position
 *     frame_id: base_link
 *     link_name: tool0
 *     target_offset: [0.1, 0.1, 0.1]  # [x, y, z]
 *     region:
 *       x: [0.1, 0.4]  # [min, max]
 *       y: [0.2, 0.3]
 *       z: [0.1, 0.6]
 *     weight: 1.0
 * """
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opt;
  opt.automatically_declare_parameters_from_overrides(true);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("generate_state_database", opt);

  GenerateStateDatabaseParameters params;
  if (!params.setFromNode(node))
    return 1;

  planning_scene_monitor::PlanningSceneMonitor psm(node, ROBOT_DESCRIPTION);
  if (!psm.getRobotModel())
    return 1;

  if (params.use_current_scene)
  {
    RCLCPP_INFO(LOGGER, "Requesting current planning scene to generate database");
    if (!psm.requestPlanningSceneState())
    {
      RCLCPP_FATAL(LOGGER, "Abort. The current scene could not be retrieved.");
      return 1;
    }
  }

  if (moveit::core::isEmpty(params.constraints))
  {
    RCLCPP_FATAL(LOGGER, "Abort. Constraint description is an empty set of constraints.");
    return 1;
  }

  computeDB(node, psm.getPlanningScene(), params);

  rclcpp::shutdown();

  return 0;
}
