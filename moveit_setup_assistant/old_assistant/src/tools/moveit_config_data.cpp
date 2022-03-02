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

/* Author: Dave Coleman */

#include <moveit/setup_assistant/tools/moveit_config_data.hpp>
// Reading/Writing Files
#include <iostream>  // For writing yaml and launch files
#include <fstream>
#include <boost/filesystem/path.hpp>        // for creating folders/files
#include <boost/filesystem/operations.hpp>  // is_regular_file, is_directory, etc.
#include <boost/algorithm/string/trim.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>  // for getting file path for loading images

// OMPL version
#include <ompl/config.h>

namespace moveit_setup_assistant
{
// File system
namespace fs = boost::filesystem;

// ******************************************************************************************
// Constructor
// ******************************************************************************************
MoveItConfigData::MoveItConfigData() : config_pkg_generated_timestamp_(0)
{
  // Not in debug mode
  debug_ = false;

  // Get MoveIt Setup Assistant package path
  setup_assistant_path_ = ament_index_cpp::get_package_share_directory("moveit_setup_assistant");
  if (setup_assistant_path_.empty())
  {
    setup_assistant_path_ = ".";
  }
}

// ******************************************************************************************
// Destructor
// ******************************************************************************************
MoveItConfigData::~MoveItConfigData() = default;

// ******************************************************************************************
// Output OMPL Planning config files
// ******************************************************************************************
bool MoveItConfigData::outputOMPLPlanningYAML(const std::string& file_path)
{
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;

  // Output every available planner ---------------------------------------------------
  emitter << YAML::Key << "planner_configs";

  emitter << YAML::Value << YAML::BeginMap;

  std::vector<OMPLPlannerDescription> planner_des = getOMPLPlanners();

  // Add Planners with parameter values
  std::vector<std::string> pconfigs;
  for (OMPLPlannerDescription& planner_de : planner_des)
  {
    std::string defaultconfig = planner_de.name_;
    emitter << YAML::Key << defaultconfig;
    emitter << YAML::Value << YAML::BeginMap;
    emitter << YAML::Key << "type" << YAML::Value << "geometric::" + planner_de.name_;
    for (OmplPlanningParameter& ompl_planner_param : planner_de.parameter_list_)
    {
      emitter << YAML::Key << ompl_planner_param.name;
      emitter << YAML::Value << ompl_planner_param.value;
      emitter << YAML::Comment(ompl_planner_param.comment);
    }
    emitter << YAML::EndMap;

    pconfigs.push_back(defaultconfig);
  }

  // End of every avail planner
  emitter << YAML::EndMap;

  // Output every group and the planners it can use ----------------------------------
  for (srdf::Model::Group& group : srdf_->groups_)
  {
    emitter << YAML::Key << group.name_;
    emitter << YAML::Value << YAML::BeginMap;
    // Output associated planners
    if (!group_meta_data_[group.name_].default_planner_.empty())
      emitter << YAML::Key << "default_planner_config" << YAML::Value << group_meta_data_[group.name_].default_planner_;
    emitter << YAML::Key << "planner_configs";
    emitter << YAML::Value << YAML::BeginSeq;
    for (const std::string& pconfig : pconfigs)
      emitter << pconfig;
    emitter << YAML::EndSeq;

    // Output projection_evaluator
    std::string projection_joints = decideProjectionJoints(group.name_);
    if (!projection_joints.empty())
    {
      emitter << YAML::Key << "projection_evaluator";
      emitter << YAML::Value << projection_joints;
      // OMPL collision checking discretization
      emitter << YAML::Key << "longest_valid_segment_fraction";
      emitter << YAML::Value << "0.005";
    }

    emitter << YAML::EndMap;
  }

  emitter << YAML::EndMap;

  std::ofstream output_stream(file_path.c_str(), std::ios_base::trunc);
  if (!output_stream.good())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Unable to open file for writing " << file_path);
    return false;
  }

  output_stream << emitter.c_str() << '\n';
  output_stream.close();

  return true;  // file created successfully
}

// ******************************************************************************************
// Output STOMP Planning config files
// ******************************************************************************************
bool MoveItConfigData::outputSTOMPPlanningYAML(const std::string& file_path)
{
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;

  // Add STOMP default for every group
  for (srdf::Model::Group& group : srdf_->groups_)
  {
    emitter << YAML::Key << "stomp/" + group.name_;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "group_name";
    emitter << YAML::Value << group.name_;

    emitter << YAML::Key << "optimization";
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "num_timesteps";
    emitter << YAML::Value << "60";
    emitter << YAML::Key << "num_iterations";
    emitter << YAML::Value << "40";
    emitter << YAML::Key << "num_iterations_after_valid";
    emitter << YAML::Value << "0";
    emitter << YAML::Key << "num_rollouts";
    emitter << YAML::Value << "30";
    emitter << YAML::Key << "max_rollouts";
    emitter << YAML::Value << "30";
    emitter << YAML::Key << "initialization_method";
    emitter << YAML::Value << "1";
    emitter << YAML::Comment("[1 : LINEAR_INTERPOLATION, 2 : CUBIC_POLYNOMIAL, 3 : MININUM_CONTROL_COST]");
    emitter << YAML::Key << "control_cost_weight";
    emitter << YAML::Value << "0.0";
    emitter << YAML::EndMap;

    emitter << YAML::Key << "task";
    emitter << YAML::BeginMap;

    emitter << YAML::Key << "noise_generator";
    emitter << YAML::BeginSeq;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "class";
    emitter << YAML::Value << "stomp_moveit/NormalDistributionSampling";
    emitter << YAML::Key << "stddev";
    emitter << YAML::Flow;
    const moveit::core::JointModelGroup* joint_model_group = getRobotModel()->getJointModelGroup(group.name_);
    const std::vector<const moveit::core::JointModel*>& joint_models = joint_model_group->getActiveJointModels();
    std::vector<float> stddev(joint_models.size(), 0.05);
    emitter << YAML::Value << stddev;
    emitter << YAML::EndMap;
    emitter << YAML::EndSeq;

    emitter << YAML::Key << "cost_functions";
    emitter << YAML::BeginSeq;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "class";
    emitter << YAML::Value << "stomp_moveit/CollisionCheck";
    emitter << YAML::Key << "collision_penalty";
    emitter << YAML::Value << "1.0";
    emitter << YAML::Key << "cost_weight";
    emitter << YAML::Value << "1.0";
    emitter << YAML::Key << "kernel_window_percentage";
    emitter << YAML::Value << "0.2";
    emitter << YAML::Key << "longest_valid_joint_move";
    emitter << YAML::Value << "0.05";
    emitter << YAML::EndMap;
    emitter << YAML::EndSeq;

    emitter << YAML::Key << "noisy_filters";
    emitter << YAML::BeginSeq;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "class";
    emitter << YAML::Value << "stomp_moveit/JointLimits";
    emitter << YAML::Key << "lock_start";
    emitter << YAML::Value << "True";
    emitter << YAML::Key << "lock_goal";
    emitter << YAML::Value << "True";
    emitter << YAML::EndMap;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "class";
    emitter << YAML::Value << "stomp_moveit/MultiTrajectoryVisualization";
    emitter << YAML::Key << "line_width";
    emitter << YAML::Value << "0.02";
    emitter << YAML::Key << "rgb";
    emitter << YAML::Flow;
    std::vector<float> noisy_filters_rgb{ 255, 255, 0 };
    emitter << YAML::Value << noisy_filters_rgb;
    emitter << YAML::Key << "marker_array_topic";
    emitter << YAML::Value << "stomp_trajectories";
    emitter << YAML::Key << "marker_namespace";
    emitter << YAML::Value << "noisy";
    emitter << YAML::EndMap;
    emitter << YAML::EndSeq;

    emitter << YAML::Key << "update_filters";
    emitter << YAML::BeginSeq;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "class";
    emitter << YAML::Value << "stomp_moveit/PolynomialSmoother";
    emitter << YAML::Key << "poly_order";
    emitter << YAML::Value << "6";
    emitter << YAML::EndMap;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "class";
    emitter << YAML::Value << "stomp_moveit/TrajectoryVisualization";
    emitter << YAML::Key << "line_width";
    emitter << YAML::Value << "0.05";
    emitter << YAML::Key << "rgb";
    emitter << YAML::Flow;
    std::vector<float> update_filters_rgb{ 0, 191, 255 };
    emitter << YAML::Value << update_filters_rgb;
    emitter << YAML::Key << "error_rgb";
    emitter << YAML::Flow;
    std::vector<float> update_filters_error_rgb{ 255, 0, 0 };
    emitter << YAML::Value << update_filters_error_rgb;
    emitter << YAML::Key << "publish_intermediate";
    emitter << YAML::Value << "True";
    emitter << YAML::Key << "marker_topic";
    emitter << YAML::Value << "stomp_trajectory";
    emitter << YAML::Key << "marker_namespace";
    emitter << YAML::Value << "optimized";
    emitter << YAML::EndMap;
    emitter << YAML::EndSeq;

    emitter << YAML::EndMap;
    emitter << YAML::EndMap;
  }

  emitter << YAML::EndMap;

  std::ofstream output_stream(file_path.c_str(), std::ios_base::trunc);
  if (!output_stream.good())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Unable to open file for writing " << file_path);
    return false;
  }

  output_stream << emitter.c_str();
  output_stream.close();

  return true;  // file created successfully
}

// ******************************************************************************************
// Helper function to get the controller that is controlling the joint
// ******************************************************************************************
std::string MoveItConfigData::getJointHardwareInterface(const std::string& joint_name)
{
  for (ControllerConfig& ros_control_config : controller_configs_)
  {
    std::vector<std::string>::iterator joint_it =
        std::find(ros_control_config.joints_.begin(), ros_control_config.joints_.end(), joint_name);
    if (joint_it != ros_control_config.joints_.end())
    {
      if (ros_control_config.type_.substr(0, 8) == "position")
        return "hardware_interface/PositionJointInterface";
      else if (ros_control_config.type_.substr(0, 8) == "velocity")
        return "hardware_interface/VelocityJointInterface";
      // As of writing this, available joint command interfaces are position, velocity and effort.
      else
        return "hardware_interface/EffortJointInterface";
    }
  }
  // If the joint was not found in any controller return EffortJointInterface
  return "hardware_interface/EffortJointInterface";
}

bool MoveItConfigData::outputFakeControllersYAML(const std::string& file_path)
{
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;

  emitter << YAML::Key << "controller_list";
  emitter << YAML::Value << YAML::BeginSeq;

  // Loop through groups
  for (srdf::Model::Group& group : srdf_->groups_)
  {
    // Get list of associated joints
    const moveit::core::JointModelGroup* joint_model_group = getRobotModel()->getJointModelGroup(group.name_);
    emitter << YAML::BeginMap;
    const std::vector<const moveit::core::JointModel*>& joint_models = joint_model_group->getActiveJointModels();
    emitter << YAML::Key << "name";
    emitter << YAML::Value << "fake_" + group.name_ + "_controller";
    emitter << YAML::Key << "type";
    emitter << YAML::Value << "$(arg fake_execution_type)";
    emitter << YAML::Key << "joints";
    emitter << YAML::Value << YAML::BeginSeq;

    // Iterate through the joints
    for (const moveit::core::JointModel* joint : joint_models)
    {
      if (joint->isPassive() || joint->getMimic() != nullptr || joint->getType() == moveit::core::JointModel::FIXED)
        continue;
      emitter << joint->getName();
    }
    emitter << YAML::EndSeq;
    emitter << YAML::EndMap;
  }

  emitter << YAML::EndSeq;

  // Add an initial pose for each group
  emitter << YAML::Key << "initial" << YAML::Comment("Define initial robot poses per group");

  bool poses_found = false;
  std::string default_group_name;
  for (const srdf::Model::Group& group : srdf_->groups_)
  {
    if (default_group_name.empty())
      default_group_name = group.name_;
    for (const srdf::Model::GroupState& group_state : srdf_->group_states_)
    {
      if (group.name_ == group_state.group_)
      {
        if (!poses_found)
        {
          poses_found = true;
          emitter << YAML::Value << YAML::BeginSeq;
        }
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "group";
        emitter << YAML::Value << group.name_;
        emitter << YAML::Key << "pose";
        emitter << YAML::Value << group_state.name_;
        emitter << YAML::EndMap;
        break;
      }
    }
  }
  if (poses_found)
    emitter << YAML::EndSeq;
  else
  {
    // Add commented lines to show how the feature can be used
    if (default_group_name.empty())
      default_group_name = "group";
    emitter << YAML::Newline;
    emitter << YAML::Comment(" - group: " + default_group_name) << YAML::Newline;
    emitter << YAML::Comment("   pose: home") << YAML::Newline;

    // Add empty list for valid yaml
    emitter << YAML::BeginSeq;
    emitter << YAML::EndSeq;
  }

  emitter << YAML::EndMap;

  std::ofstream output_stream(file_path.c_str(), std::ios_base::trunc);
  if (!output_stream.good())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Unable to open file for writing " << file_path);
    return false;
  }

  output_stream << emitter.c_str();
  output_stream.close();

  return true;  // file created successfully
}

std::map<std::string, double> MoveItConfigData::getInitialJoints() const
{
  std::map<std::string, double> joints;
  for (const srdf::Model::Group& group : srdf_->groups_)
  {
    // use first pose of each group as initial pose
    for (const srdf::Model::GroupState& group_state : srdf_->group_states_)
    {
      if (group.name_ != group_state.group_)
        continue;
      for (const auto& pair : group_state.joint_values_)
      {
        if (pair.second.size() != 1)
          continue;  // only handle simple joints here
        joints[pair.first] = pair.second.front();
      }
      break;
    }
  }
  return joints;
}

std::vector<OMPLPlannerDescription> MoveItConfigData::getOMPLPlanners() const
{
  std::vector<OMPLPlannerDescription> planner_des;

  OMPLPlannerDescription aps("AnytimePathShortening", "geometric");
  aps.addParameter("shortcut", "true", "Attempt to shortcut all new solution paths");
  aps.addParameter("hybridize", "true", "Compute hybrid solution trajectories");
  aps.addParameter("max_hybrid_paths", "24", "Number of hybrid paths generated per iteration");
  aps.addParameter("num_planners", "4", "The number of default planners to use for planning");
// TODO: remove when ROS Melodic and older are no longer supported
#if OMPL_VERSION_VALUE >= 1005000
  // This parameter was added in OMPL 1.5.0
  aps.addParameter("planners", "",
                   "A comma-separated list of planner types (e.g., \"PRM,EST,RRTConnect\""
                   "Optionally, planner parameters can be passed to change the default:"
                   "\"PRM[max_nearest_neighbors=5],EST[goal_bias=.5],RRT[range=10. goal_bias=.1]\"");
#endif
  planner_des.push_back(aps);

  OMPLPlannerDescription sbl("SBL", "geometric");
  sbl.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()");
  planner_des.push_back(sbl);

  OMPLPlannerDescription est("EST", "geometric");
  est.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0 setup()");
  est.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability. default: 0.05");
  planner_des.push_back(est);

  OMPLPlannerDescription lbkpiece("LBKPIECE", "geometric");
  lbkpiece.addParameter("range", "0.0",
                        "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                        "setup()");
  lbkpiece.addParameter("border_fraction", "0.9", "Fraction of time focused on boarder default: 0.9");
  lbkpiece.addParameter("min_valid_path_fraction", "0.5", "Accept partially valid moves above fraction. default: 0.5");
  planner_des.push_back(lbkpiece);

  OMPLPlannerDescription bkpiece("BKPIECE", "geometric");
  bkpiece.addParameter("range", "0.0",
                       "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                       "setup()");
  bkpiece.addParameter("border_fraction", "0.9", "Fraction of time focused on boarder default: 0.9");
  bkpiece.addParameter("failed_expansion_score_factor", "0.5",
                       "When extending motion fails, scale score by factor. "
                       "default: 0.5");
  bkpiece.addParameter("min_valid_path_fraction", "0.5", "Accept partially valid moves above fraction. default: 0.5");
  planner_des.push_back(bkpiece);

  OMPLPlannerDescription kpiece("KPIECE", "geometric");
  kpiece.addParameter("range", "0.0",
                      "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                      "setup()");
  kpiece.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability. default: 0.05");
  kpiece.addParameter("border_fraction", "0.9", "Fraction of time focused on boarder default: 0.9 (0.0,1.]");
  kpiece.addParameter("failed_expansion_score_factor", "0.5",
                      "When extending motion fails, scale score by factor. "
                      "default: 0.5");
  kpiece.addParameter("min_valid_path_fraction", "0.5", "Accept partially valid moves above fraction. default: 0.5");
  planner_des.push_back(kpiece);

  OMPLPlannerDescription rrt("RRT", "geometric");
  rrt.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()");
  rrt.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability? default: 0.05");
  planner_des.push_back(rrt);

  OMPLPlannerDescription rrt_connect("RRTConnect", "geometric");
  rrt_connect.addParameter("range", "0.0",
                           "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                           "setup()");
  planner_des.push_back(rrt_connect);

  OMPLPlannerDescription rr_tstar("RRTstar", "geometric");
  rr_tstar.addParameter("range", "0.0",
                        "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                        "setup()");
  rr_tstar.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability? default: 0.05");
  rr_tstar.addParameter("delay_collision_checking", "1",
                        "Stop collision checking as soon as C-free parent found. "
                        "default 1");
  planner_des.push_back(rr_tstar);

  OMPLPlannerDescription trrt("TRRT", "geometric");
  trrt.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()");
  trrt.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability? default: 0.05");
  trrt.addParameter("max_states_failed", "10", "when to start increasing temp. default: 10");
  trrt.addParameter("temp_change_factor", "2.0", "how much to increase or decrease temp. default: 2.0");
  trrt.addParameter("min_temperature", "10e-10", "lower limit of temp change. default: 10e-10");
  trrt.addParameter("init_temperature", "10e-6", "initial temperature. default: 10e-6");
  trrt.addParameter("frountier_threshold", "0.0",
                    "dist new state to nearest neighbor to disqualify as frontier. "
                    "default: 0.0 set in setup()");
  trrt.addParameter("frountierNodeRatio", "0.1", "1/10, or 1 nonfrontier for every 10 frontier. default: 0.1");
  trrt.addParameter("k_constant", "0.0", "value used to normalize expression. default: 0.0 set in setup()");
  planner_des.push_back(trrt);

  OMPLPlannerDescription prm("PRM", "geometric");
  prm.addParameter("max_nearest_neighbors", "10", "use k nearest neighbors. default: 10");
  planner_des.push_back(prm);

  OMPLPlannerDescription pr_mstar("PRMstar", "geometric");  // no declares in code
  planner_des.push_back(pr_mstar);

  OMPLPlannerDescription fmt("FMT", "geometric");
  fmt.addParameter("num_samples", "1000", "number of states that the planner should sample. default: 1000");
  fmt.addParameter("radius_multiplier", "1.1", "multiplier used for the nearest neighbors search radius. default: 1.1");
  fmt.addParameter("nearest_k", "1", "use Knearest strategy. default: 1");
  fmt.addParameter("cache_cc", "1", "use collision checking cache. default: 1");
  fmt.addParameter("heuristics", "0", "activate cost to go heuristics. default: 0");
  fmt.addParameter("extended_fmt", "1",
                   "activate the extended FMT*: adding new samples if planner does not finish "
                   "successfully. default: 1");
  planner_des.push_back(fmt);

  OMPLPlannerDescription bfmt("BFMT", "geometric");
  bfmt.addParameter("num_samples", "1000", "number of states that the planner should sample. default: 1000");
  bfmt.addParameter("radius_multiplier", "1.0",
                    "multiplier used for the nearest neighbors search radius. default: "
                    "1.0");
  bfmt.addParameter("nearest_k", "1", "use the Knearest strategy. default: 1");
  bfmt.addParameter("balanced", "0",
                    "exploration strategy: balanced true expands one tree every iteration. False will "
                    "select the tree with lowest maximum cost to go. default: 1");
  bfmt.addParameter("optimality", "1",
                    "termination strategy: optimality true finishes when the best possible path is "
                    "found. Otherwise, the algorithm will finish when the first feasible path is "
                    "found. default: 1");
  bfmt.addParameter("heuristics", "1", "activates cost to go heuristics. default: 1");
  bfmt.addParameter("cache_cc", "1", "use the collision checking cache. default: 1");
  bfmt.addParameter("extended_fmt", "1",
                    "Activates the extended FMT*: adding new samples if planner does not finish "
                    "successfully. default: 1");
  planner_des.push_back(bfmt);

  OMPLPlannerDescription pdst("PDST", "geometric");
  rrt.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability? default: 0.05");
  planner_des.push_back(pdst);

  OMPLPlannerDescription stride("STRIDE", "geometric");
  stride.addParameter("range", "0.0",
                      "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                      "setup()");
  stride.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability. default: 0.05");
  stride.addParameter("use_projected_distance", "0",
                      "whether nearest neighbors are computed based on distances in a "
                      "projection of the state rather distances in the state space "
                      "itself. default: 0");
  stride.addParameter("degree", "16",
                      "desired degree of a node in the Geometric Near-neightbor Access Tree (GNAT). "
                      "default: 16");
  stride.addParameter("max_degree", "18", "max degree of a node in the GNAT. default: 12");
  stride.addParameter("min_degree", "12", "min degree of a node in the GNAT. default: 12");
  stride.addParameter("max_pts_per_leaf", "6", "max points per leaf in the GNAT. default: 6");
  stride.addParameter("estimated_dimension", "0.0", "estimated dimension of the free space. default: 0.0");
  stride.addParameter("min_valid_path_fraction", "0.2", "Accept partially valid moves above fraction. default: 0.2");
  planner_des.push_back(stride);

  OMPLPlannerDescription bi_trrt("BiTRRT", "geometric");
  bi_trrt.addParameter("range", "0.0",
                       "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                       "setup()");
  bi_trrt.addParameter("temp_change_factor", "0.1", "how much to increase or decrease temp. default: 0.1");
  bi_trrt.addParameter("init_temperature", "100", "initial temperature. default: 100");
  bi_trrt.addParameter("frountier_threshold", "0.0",
                       "dist new state to nearest neighbor to disqualify as frontier. "
                       "default: 0.0 set in setup()");
  bi_trrt.addParameter("frountier_node_ratio", "0.1", "1/10, or 1 nonfrontier for every 10 frontier. default: 0.1");
  bi_trrt.addParameter("cost_threshold", "1e300",
                       "the cost threshold. Any motion cost that is not better will not be "
                       "expanded. default: inf");
  planner_des.push_back(bi_trrt);

  OMPLPlannerDescription lbtrrt("LBTRRT", "geometric");
  lbtrrt.addParameter("range", "0.0",
                      "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                      "setup()");
  lbtrrt.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability. default: 0.05");
  lbtrrt.addParameter("epsilon", "0.4", "optimality approximation factor. default: 0.4");
  planner_des.push_back(lbtrrt);

  OMPLPlannerDescription bi_est("BiEST", "geometric");
  bi_est.addParameter("range", "0.0",
                      "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                      "setup()");
  planner_des.push_back(bi_est);

  OMPLPlannerDescription proj_est("ProjEST", "geometric");
  proj_est.addParameter("range", "0.0",
                        "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                        "setup()");
  proj_est.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability. default: 0.05");
  planner_des.push_back(proj_est);

  OMPLPlannerDescription lazy_prm("LazyPRM", "geometric");
  lazy_prm.addParameter("range", "0.0",
                        "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                        "setup()");
  planner_des.push_back(lazy_prm);

  OMPLPlannerDescription lazy_pr_mstar("LazyPRMstar", "geometric");  // no declares in code
  planner_des.push_back(lazy_pr_mstar);

  OMPLPlannerDescription spars("SPARS", "geometric");
  spars.addParameter("stretch_factor", "3.0",
                     "roadmap spanner stretch factor. multiplicative upper bound on path "
                     "quality. It does not make sense to make this parameter more than 3. "
                     "default: 3.0");
  spars.addParameter("sparse_delta_fraction", "0.25",
                     "delta fraction for connection distance. This value represents "
                     "the visibility range of sparse samples. default: 0.25");
  spars.addParameter("dense_delta_fraction", "0.001", "delta fraction for interface detection. default: 0.001");
  spars.addParameter("max_failures", "1000", "maximum consecutive failure limit. default: 1000");
  planner_des.push_back(spars);

  OMPLPlannerDescription spar_stwo("SPARStwo", "geometric");
  spar_stwo.addParameter("stretch_factor", "3.0",
                         "roadmap spanner stretch factor. multiplicative upper bound on path "
                         "quality. It does not make sense to make this parameter more than 3. "
                         "default: 3.0");
  spar_stwo.addParameter("sparse_delta_fraction", "0.25",
                         "delta fraction for connection distance. This value represents "
                         "the visibility range of sparse samples. default: 0.25");
  spar_stwo.addParameter("dense_delta_fraction", "0.001", "delta fraction for interface detection. default: 0.001");
  spar_stwo.addParameter("max_failures", "5000", "maximum consecutive failure limit. default: 5000");
  planner_des.push_back(spar_stwo);

  return planner_des;
}

// ******************************************************************************************
// Generate simple_moveit_controllers.yaml config file
// ******************************************************************************************
bool MoveItConfigData::outputSimpleControllersYAML(const std::string& file_path)
{
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;
  emitter << YAML::Key << "controller_list";
  emitter << YAML::Value << YAML::BeginSeq;
  for (const auto& controller : controller_configs_)
  {
    // Only process FollowJointTrajectory types
    std::string type = controller.type_;
    if (boost::ends_with(type, "/JointTrajectoryController"))
      type = "FollowJointTrajectory";
    if (type == "FollowJointTrajectory" || type == "GripperCommand")
    {
      emitter << YAML::BeginMap;
      emitter << YAML::Key << "name";
      emitter << YAML::Value << controller.name_;
      emitter << YAML::Key << "action_ns";
      emitter << YAML::Value << (type == "FollowJointTrajectory" ? "follow_joint_trajectory" : "gripper_action");
      emitter << YAML::Key << "type";
      emitter << YAML::Value << type;
      emitter << YAML::Key << "default";
      emitter << YAML::Value << "True";

      // Write joints
      emitter << YAML::Key << "joints";
      emitter << YAML::Value << YAML::BeginSeq;
      // Iterate through the joints
      for (const std::string& joint : controller.joints_)
        emitter << joint;
      emitter << YAML::EndSeq;

      emitter << YAML::EndMap;
    }
  }
  emitter << YAML::EndSeq;
  emitter << YAML::EndMap;

  std::ofstream output_stream(file_path.c_str(), std::ios_base::trunc);
  if (!output_stream.good())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Unable to open file for writing " << file_path);
    return false;
  }
  output_stream << emitter.c_str();
  output_stream.close();

  return true;  // file created successfully
}

// ******************************************************************************************
// Helper function to get the default start pose for moveit_sim_hw_interface
// ******************************************************************************************
srdf::Model::GroupState MoveItConfigData::getDefaultStartPose()
{
  if (!srdf_->group_states_.empty())
    return srdf_->group_states_[0];
  else
    return srdf::Model::GroupState{ .name_ = "todo_state_name", .group_ = "todo_group_name", .joint_values_ = {} };
}

// ******************************************************************************************
// Generate ros_controllers.yaml config file
// ******************************************************************************************
bool MoveItConfigData::outputROSControllersYAML(const std::string& file_path)
{
  // Cache the joints' names.
  std::vector<std::vector<std::string>> planning_groups;

  // We are going to write the joints names many times.
  // Loop through groups to store the joints names in group_joints vector and reuse is.
  for (srdf::Model::Group& group : srdf_->groups_)
  {
    std::vector<std::string> group_joints;
    // Get list of associated joints
    const moveit::core::JointModelGroup* joint_model_group = getRobotModel()->getJointModelGroup(group.name_);
    const std::vector<const moveit::core::JointModel*>& joint_models = joint_model_group->getActiveJointModels();
    // Iterate through the joints and push into group_joints vector.
    for (const moveit::core::JointModel* joint : joint_models)
    {
      if (joint->isPassive() || joint->getMimic() != nullptr || joint->getType() == moveit::core::JointModel::FIXED)
        continue;
      else
        group_joints.push_back(joint->getName());
    }
    // Push all the group joints into planning_groups vector.
    planning_groups.push_back(group_joints);
  }

  YAML::Emitter emitter;
  emitter << YAML::BeginMap;

  {
#if 0  // TODO: This is only for fake ROS controllers, which should go into a separate file
    // Also replace moveit_sim_controllers with http://wiki.ros.org/fake_joint
    emitter << YAML::Comment("Simulation settings for using moveit_sim_controllers");
    emitter << YAML::Key << "moveit_sim_hw_interface" << YAML::Value << YAML::BeginMap;
    // MoveIt Simulation Controller settings for setting initial pose
    {
      // Use the first planning group if initial joint_model_group was not set, else write a default value
      emitter << YAML::Key << "joint_model_group";
      emitter << YAML::Value << getDefaultStartPose().group_;

      // Use the first robot pose if initial joint_model_group_pose was not set, else write a default value
      emitter << YAML::Key << "joint_model_group_pose";
      emitter << YAML::Value << getDefaultStartPose().name_;

      emitter << YAML::EndMap;
    }
    // Settings for ros_control control loop
    emitter << YAML::Newline;
    emitter << YAML::Comment("Settings for ros_control_boilerplate control loop");
    emitter << YAML::Key << "generic_hw_control_loop" << YAML::Value << YAML::BeginMap;
    {
      emitter << YAML::Key << "loop_hz";
      emitter << YAML::Value << "300";
      emitter << YAML::Key << "cycle_time_error_threshold";
      emitter << YAML::Value << "0.01";
      emitter << YAML::EndMap;
    }
    // Settings for ros_control hardware interface
    emitter << YAML::Newline;
    emitter << YAML::Comment("Settings for ros_control hardware interface");
    emitter << YAML::Key << "hardware_interface" << YAML::Value << YAML::BeginMap;
    {
      // Get list of all joints for the robot
      const std::vector<const moveit::core::JointModel*>& joint_models = getRobotModel()->getJointModels();

      emitter << YAML::Key << "joints";
      {
        if (joint_models.size() != 1)
        {
          emitter << YAML::Value << YAML::BeginSeq;
          // Iterate through the joints
          for (std::vector<const moveit::core::JointModel*>::const_iterator joint_it = joint_models.begin();
               joint_it < joint_models.end(); ++joint_it)
          {
            if ((*joint_it)->isPassive() || (*joint_it)->getMimic() != nullptr ||
                (*joint_it)->getType() == moveit::core::JointModel::FIXED)
              continue;
            else
              emitter << (*joint_it)->getName();
          }
          emitter << YAML::EndSeq;
        }
        else
        {
          emitter << YAML::Value << YAML::BeginMap;
          emitter << joint_models[0]->getName();
          emitter << YAML::EndMap;
        }
      }
      emitter << YAML::Key << "sim_control_mode";
      emitter << YAML::Value << "1";
      emitter << YAML::Comment("0: position, 1: velocity");
      emitter << YAML::Newline;
      emitter << YAML::EndMap;
    }
#endif
    for (const auto& controller : controller_configs_)
    {
      if (controller.type_ == "FollowJointTrajectory" || controller.type_ == "GripperCommand")
        continue;  // these are handled by outputSimpleControllersYAML()

      emitter << YAML::Key << controller.name_;
      emitter << YAML::Value << YAML::BeginMap;
      emitter << YAML::Key << "type";
      emitter << YAML::Value << controller.type_;

      // Write joints
      emitter << YAML::Key << "joints";
      emitter << YAML::Value << YAML::BeginSeq;
      // Iterate through the joints
      for (const std::string& joint : controller.joints_)
        emitter << joint;
      emitter << YAML::EndSeq;

      // Write gains as they are required for vel and effort controllers
      emitter << YAML::Key << "gains";
      emitter << YAML::Value << YAML::BeginMap;
      {
        // Iterate through the joints
        for (const std::string& joint : controller.joints_)
        {
          emitter << YAML::Key << joint << YAML::Value << YAML::BeginMap;
          emitter << YAML::Key << "p";
          emitter << YAML::Value << "100";
          emitter << YAML::Key << "d";
          emitter << YAML::Value << "1";
          emitter << YAML::Key << "i";
          emitter << YAML::Value << "1";
          emitter << YAML::Key << "i_clamp";
          emitter << YAML::Value << "1" << YAML::EndMap;
        }
        emitter << YAML::EndMap;
      }
      emitter << YAML::EndMap;
    }
  }

  std::ofstream output_stream(file_path.c_str(), std::ios_base::trunc);
  if (!output_stream.good())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Unable to open file for writing " << file_path);
    return false;
  }
  output_stream << emitter.c_str();
  output_stream.close();

  return true;  // file created successfully
}

// ******************************************************************************************
// Output joint limits config files
// ******************************************************************************************
bool MoveItConfigData::outputJointLimitsYAML(const std::string& file_path)
{
  YAML::Emitter emitter;
  emitter << YAML::Comment("joint_limits.yaml allows the dynamics properties specified in the URDF "
                           "to be overwritten or augmented as needed");
  emitter << YAML::Newline;

  emitter << YAML::BeginMap;

  emitter << YAML::Comment("For beginners, we downscale velocity and acceleration limits.") << YAML::Newline;
  emitter << YAML::Comment("You can always specify higher scaling factors (<= 1.0) in your motion requests.");
  emitter << YAML::Comment("Increase the values below to 1.0 to always move at maximum speed.");
  emitter << YAML::Key << "default_velocity_scaling_factor";
  emitter << YAML::Value << "0.1";

  emitter << YAML::Key << "default_acceleration_scaling_factor";
  emitter << YAML::Value << "0.1";

  emitter << YAML::Newline << YAML::Newline;
  emitter << YAML::Comment("Specific joint properties can be changed with the keys "
                           "[max_position, min_position, max_velocity, max_acceleration]")
          << YAML::Newline;
  emitter << YAML::Comment("Joint limits can be turned off with [has_velocity_limits, has_acceleration_limits]");

  emitter << YAML::Key << "joint_limits";
  emitter << YAML::Value << YAML::BeginMap;

  // Union all the joints in groups. Uses a custom comparator to allow the joints to be sorted by name
  std::set<const moveit::core::JointModel*, JointModelCompare> joints;

  // Loop through groups
  for (srdf::Model::Group& group : srdf_->groups_)
  {
    // Get list of associated joints
    const moveit::core::JointModelGroup* joint_model_group = getRobotModel()->getJointModelGroup(group.name_);

    const std::vector<const moveit::core::JointModel*>& joint_models = joint_model_group->getJointModels();

    // Iterate through the joints
    for (const moveit::core::JointModel* joint_model : joint_models)
    {
      // Check that this joint only represents 1 variable.
      if (joint_model->getVariableCount() == 1)
        joints.insert(joint_model);
    }
  }

  // Add joints to yaml file, if no more than 1 dof
  for (const moveit::core::JointModel* joint : joints)
  {
    emitter << YAML::Key << joint->getName();
    emitter << YAML::Value << YAML::BeginMap;

    const moveit::core::VariableBounds& b = joint->getVariableBounds()[0];

    // Output property
    emitter << YAML::Key << "has_velocity_limits";
    if (b.velocity_bounded_)
      emitter << YAML::Value << "true";
    else
      emitter << YAML::Value << "false";

    // Output property
    emitter << YAML::Key << "max_velocity";
    emitter << YAML::Value << std::min(fabs(b.max_velocity_), fabs(b.min_velocity_));

    // Output property
    emitter << YAML::Key << "has_acceleration_limits";
    if (b.acceleration_bounded_)
      emitter << YAML::Value << "true";
    else
      emitter << YAML::Value << "false";

    // Output property
    emitter << YAML::Key << "max_acceleration";
    emitter << YAML::Value << std::min(fabs(b.max_acceleration_), fabs(b.min_acceleration_));

    emitter << YAML::EndMap;
  }

  emitter << YAML::EndMap;

  std::ofstream output_stream(file_path.c_str(), std::ios_base::trunc);
  if (!output_stream.good())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Unable to open file for writing " << file_path);
    return false;
  }
  output_stream << emitter.c_str();
  output_stream.close();

  return true;  // file created successfully
}

// ******************************************************************************************
// Decide the best two joints to be used for the projection evaluator
// ******************************************************************************************
std::string MoveItConfigData::decideProjectionJoints(const std::string& planning_group)
{
  std::string joint_pair = "";

  // Retrieve pointer to the shared kinematic model
  const moveit::core::RobotModelConstPtr& model = getRobotModel();

  // Error check
  if (!model->hasJointModelGroup(planning_group))
    return joint_pair;

  // Get the joint model group
  const moveit::core::JointModelGroup* group = model->getJointModelGroup(planning_group);

  // get vector of joint names
  const std::vector<std::string>& joints = group->getJointModelNames();

  if (joints.size() >= 2)
  {
    // Check that the first two joints have only 1 variable
    if (group->getJointModel(joints[0])->getVariableCount() == 1 &&
        group->getJointModel(joints[1])->getVariableCount() == 1)
    {
      // Just choose the first two joints.
      joint_pair = "joints(" + joints[0] + "," + joints[1] + ")";
    }
  }

  return joint_pair;
}

template <typename T>
bool parse(const YAML::Node& node, const std::string& key, T& storage, const T& default_value = T())
{
  const YAML::Node& n = node[key];
  bool valid = n.IsDefined();
  storage = valid ? n.as<T>() : default_value;
  return valid;
}

bool MoveItConfigData::inputOMPLYAML(const std::string& file_path)
{
  // Load file
  std::ifstream input_stream(file_path.c_str());
  if (!input_stream.good())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Unable to open file for reading " << file_path);
    return false;
  }

  // Begin parsing
  try
  {
    YAML::Node doc = YAML::Load(input_stream);

    // Loop through all groups
    for (YAML::const_iterator group_it = doc.begin(); group_it != doc.end(); ++group_it)
    {
      // get group name
      const std::string group_name = group_it->first.as<std::string>();

      // compare group name found to list of groups in group_meta_data_
      std::map<std::string, GroupMetaData>::iterator group_meta_it;
      group_meta_it = group_meta_data_.find(group_name);
      if (group_meta_it != group_meta_data_.end())
      {
        std::string planner;
        parse(group_it->second, "default_planner_config", planner);
        std::size_t pos = planner.find("kConfigDefault");
        if (pos != std::string::npos)
        {
          planner = planner.substr(0, pos);
        }
        group_meta_data_[group_name].default_planner_ = planner;
      }
    }
  }
  catch (YAML::ParserException& e)  // Catch errors
  {
    RCLCPP_ERROR_STREAM(LOGGER, e.what());
    return false;
  }
  return true;
}

// ******************************************************************************************
// Input planning_context.launch file
// ******************************************************************************************
bool MoveItConfigData::inputPlanningContextLaunch(const std::string& file_path)
{
  TiXmlDocument launch_document(file_path);
  if (!launch_document.LoadFile())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed parsing " << file_path);
    return false;
  }

  // find the kinematics section
  TiXmlHandle doc(&launch_document);
  TiXmlElement* kinematics_group = doc.FirstChild("launch").FirstChild("group").ToElement();
  while (kinematics_group && kinematics_group->Attribute("ns") &&
         kinematics_group->Attribute("ns") != std::string("$(arg robot_description)_kinematics"))
  {
    kinematics_group = kinematics_group->NextSiblingElement("group");
  }
  if (!kinematics_group)
  {
    RCLCPP_ERROR(LOGGER, "<group ns=\"$(arg robot_description)_kinematics\"> not found");
    return false;
  }

  // iterate over all <rosparam namespace="group" file="..."/> elements
  // and if 'group' matches an existing group, copy the filename
  for (TiXmlElement* kinematics_parameter_file = kinematics_group->FirstChildElement("rosparam");
       kinematics_parameter_file; kinematics_parameter_file = kinematics_parameter_file->NextSiblingElement("rosparam"))
  {
    const char* ns = kinematics_parameter_file->Attribute("ns");
    if (ns && (group_meta_data_.find(ns) != group_meta_data_.end()))
    {
      group_meta_data_[ns].kinematics_parameters_file_ = kinematics_parameter_file->Attribute("file");
    }
  }

  return true;
}

// ******************************************************************************************
// Helper function for parsing an individual ROSController from ros_controllers yaml file
// ******************************************************************************************
bool MoveItConfigData::parseROSController(const YAML::Node& controller)
{
  // Used in parsing ROS controllers
  ControllerConfig control_setting;

  if (const YAML::Node& trajectory_controllers = controller)
  {
    for (const YAML::Node& trajectory_controller : trajectory_controllers)
    {
      // Controller node
      if (const YAML::Node& controller_node = trajectory_controller)
      {
        if (const YAML::Node& joints = controller_node["joints"])
        {
          control_setting.joints_.clear();
          for (YAML::const_iterator joint_it = joints.begin(); joint_it != joints.end(); ++joint_it)
          {
            control_setting.joints_.push_back(joint_it->as<std::string>());
          }
          if (!parse(controller_node, "name", control_setting.name_))
          {
            RCLCPP_ERROR_STREAM(LOGGER, "Couldn't parse ros_controllers.yaml");
            return false;
          }
          if (!parse(controller_node, "type", control_setting.type_))
          {
            RCLCPP_ERROR_STREAM(LOGGER, "Couldn't parse ros_controllers.yaml");
            return false;
          }
          // All required fields were parsed correctly
          controller_configs_.push_back(control_setting);
        }
        else
        {
          RCLCPP_ERROR_STREAM(LOGGER, "Couldn't parse ros_controllers.yaml");
          return false;
        }
      }
    }
  }
  return true;
}

// ******************************************************************************************
// Helper function for parsing ROSControllers from ros_controllers yaml file
// ******************************************************************************************
bool MoveItConfigData::processROSControllers(std::ifstream& input_stream)
{
  // Used in parsing ROS controllers
  ControllerConfig control_setting;
  YAML::Node controllers = YAML::Load(input_stream);

  // Loop through all controllers
  for (YAML::const_iterator controller_it = controllers.begin(); controller_it != controllers.end(); ++controller_it)
  {
    // Follow Joint Trajectory action controllers
    if (controller_it->first.as<std::string>() == "controller_list")
    {
      if (!parseROSController(controller_it->second))
        return false;
    }
    // Other settings found in the ros_controllers file
    else
    {
      const std::string& controller_name = controller_it->first.as<std::string>();
      control_setting.joints_.clear();

      // Push joints if found in the controller
      if (const YAML::Node& joints = controller_it->second["joints"])
      {
        if (joints.IsSequence())
        {
          for (YAML::const_iterator joint_it = joints.begin(); joint_it != joints.end(); ++joint_it)
          {
            control_setting.joints_.push_back(joint_it->as<std::string>());
          }
        }
        else
        {
          control_setting.joints_.push_back(joints.as<std::string>());
        }
      }

      // If the setting has joints then it is a controller that needs to be parsed
      if (!control_setting.joints_.empty())
      {
        if (const YAML::Node& urdf_node = controller_it->second["type"])
        {
          control_setting.type_ = controller_it->second["type"].as<std::string>();
          control_setting.name_ = controller_name;
          controller_configs_.push_back(control_setting);
          control_setting.joints_.clear();
        }
      }
    }
  }
  return true;
}

// ******************************************************************************************
// Input ros_controllers.yaml file
// ******************************************************************************************
bool MoveItConfigData::inputROSControllersYAML(const std::string& file_path)
{
  // Load file
  std::ifstream input_stream(file_path.c_str());
  if (!input_stream.good())
  {
    RCLCPP_WARN_STREAM(LOGGER, "Does not exist " << file_path);
    return false;
  }

  // Begin parsing
  try
  {
    processROSControllers(input_stream);
  }
  catch (YAML::ParserException& e)  // Catch errors
  {
    RCLCPP_ERROR_STREAM(LOGGER, e.what());
    return false;
  }

  return true;  // file read successfully
}

// ******************************************************************************************
// Add a Follow Joint Trajectory action Controller for each Planning Group
// ******************************************************************************************
bool MoveItConfigData::addDefaultControllers(const std::string& controller_type)
{
  if (srdf_->srdf_model_->getGroups().empty())
    return false;
  // Loop through groups
  for (const srdf::Model::Group& group_it : srdf_->srdf_model_->getGroups())
  {
    ControllerConfig group_controller;
    // Get list of associated joints
    const moveit::core::JointModelGroup* joint_model_group = getRobotModel()->getJointModelGroup(group_it.name_);
    const std::vector<const moveit::core::JointModel*>& joint_models = joint_model_group->getActiveJointModels();

    // Iterate through the joints
    for (const moveit::core::JointModel* joint : joint_models)
    {
      if (joint->isPassive() || joint->getMimic() != nullptr || joint->getType() == moveit::core::JointModel::FIXED)
        continue;
      group_controller.joints_.push_back(joint->getName());
    }
    if (!group_controller.joints_.empty())
    {
      group_controller.name_ = group_it.name_ + "_controller";
      group_controller.type_ = controller_type;
      addController(group_controller);
    }
  }
  return true;
}

// ******************************************************************************************
// Make the full URDF path using the loaded .setup_assistant data
// ******************************************************************************************
bool MoveItConfigData::createFullURDFPath()
{
  boost::trim(urdf_pkg_name_);

  // Check if a package name was provided
  if (urdf_pkg_name_.empty() || urdf_pkg_name_ == "\"\"")
  {
    urdf_path_ = urdf_pkg_relative_path_;
    urdf_pkg_name_.clear();
  }
  else
  {
    // Check that ROS can find the package
    std::string robot_desc_pkg_path = ament_index_cpp::get_package_share_directory(urdf_pkg_name_);

    if (robot_desc_pkg_path.empty())
    {
      urdf_path_.clear();
      return false;
    }

    // Append the relative URDF url path
    urdf_path_ = appendPaths(robot_desc_pkg_path, urdf_pkg_relative_path_);
  }

  // Check that this file exits -------------------------------------------------
  return fs::is_regular_file(urdf_path_);
}

// ******************************************************************************************
// Find a controller by name
// ******************************************************************************************
ControllerConfig* MoveItConfigData::findControllerByName(const std::string& controller_name)
{
  // Find the controller we are editing based on its name
  for (ControllerConfig& controller : controller_configs_)
  {
    if (controller.name_ == controller_name)  // string match
      return &controller;                     // convert to pointer from iterator
  }

  return nullptr;  // not found
}

// ******************************************************************************************
// Deletes a controller by name
// ******************************************************************************************
bool MoveItConfigData::deleteController(const std::string& controller_name)
{
  for (std::vector<ControllerConfig>::iterator controller_it = controller_configs_.begin();
       controller_it != controller_configs_.end(); ++controller_it)
  {
    if (controller_it->name_ == controller_name)  // string match
    {
      controller_configs_.erase(controller_it);
      // we are done searching
      return true;
    }
  }
  return false;
}

// ******************************************************************************************
// Adds a controller to controller_configs_ vector
// ******************************************************************************************
bool MoveItConfigData::addController(const ControllerConfig& new_controller)
{
  // Find if there is an existing controller with the same name
  ControllerConfig* controller = findControllerByName(new_controller.name_);

  if (controller && controller->type_ == new_controller.type_)
    return false;

  controller_configs_.push_back(new_controller);
  return true;
}

// ******************************************************************************************
// Gets ros_controllers_config_ vector
// ******************************************************************************************
void MoveItConfigData::addGenericParameterToSensorPluginConfig(const std::string& name, const std::string& value,
                                                               const std::string& /*comment*/)
{
  // Use index 0 since we only write one plugin
  GenericParameter new_parameter;
  new_parameter.setName(name);
  new_parameter.setValue(value);
  sensors_plugin_config_parameter_list_[0][name] = new_parameter;
}

}  // namespace moveit_setup_assistant
