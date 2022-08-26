/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Rice University
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
 *   * Neither the name of the Rice University nor the names of its
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

/* Author: Ryan Luna */

#include <moveit/benchmarks/BenchmarkExecutor.h>
#include <moveit/utils/lexical_casts.h>
#include <moveit/version.h>
#include <tf2_eigen/tf2_eigen.hpp>

// TODO(henningkayser): Switch to boost/timer/progress_display.hpp with Boost 1.72
// boost/progress.hpp is deprecated and will be replaced by boost/timer/progress_display.hpp in Boost 1.72.
// Until then we need to suppress the deprecation warning.
#define BOOST_ALLOW_DEPRECATED_HEADERS
#include <boost/regex.hpp>
#include <boost/progress.hpp>
#undef BOOST_ALLOW_DEPRECATED_HEADERS
#include <boost/date_time/posix_time/posix_time.hpp>
#include <math.h>
#include <limits>
#include <filesystem>
#ifndef _WIN32
#include <unistd.h>
#else
#include <winsock2.h>
#endif

#undef max

using namespace moveit_ros_benchmarks;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit.ros.benchmarks.BenchmarkExecutor");

template <class Clock, class Duration>
boost::posix_time::ptime toBoost(const std::chrono::time_point<Clock, Duration>& from)
{
  typedef std::chrono::nanoseconds duration_t;
  typedef long rep_t;
  rep_t d = std::chrono::duration_cast<duration_t>(from.time_since_epoch()).count();
  rep_t sec = d / 1000000000;
  rep_t nsec = d % 1000000000;
  namespace pt = boost::posix_time;
#ifdef BOOST_DATE_TIME_HAS_NANOSECONDS
  return pt::from_time_t(sec) + pt::nanoseconds(nsec)
#else
  return pt::from_time_t(sec) + pt::microseconds(nsec / 1000);
#endif
}

static std::string getHostname()
{
  static const int BUF_SIZE = 1024;
  char buffer[BUF_SIZE];
  int err = gethostname(buffer, sizeof(buffer));
  if (err != 0)
    return std::string();
  else
  {
    buffer[BUF_SIZE - 1] = '\0';
    return std::string(buffer);
  }
}

BenchmarkExecutor::BenchmarkExecutor(const rclcpp::Node::SharedPtr& node, const std::string& robot_description_param)
  : node_(node), dbloader(node)
{
  pss_ = nullptr;
  psws_ = nullptr;
  rs_ = nullptr;
  cs_ = nullptr;
  tcs_ = nullptr;
  psm_ = new planning_scene_monitor::PlanningSceneMonitor(node, robot_description_param);
  planning_scene_ = psm_->getPlanningScene();
}

BenchmarkExecutor::~BenchmarkExecutor()
{
  delete pss_;
  delete psws_;
  delete rs_;
  delete cs_;
  delete tcs_;
  delete psm_;
}

void BenchmarkExecutor::initialize(const std::vector<std::string>& planning_pipeline_names)
{
  planning_pipelines_.clear();

  // ros::NodeHandle pnh("~");
  std::string parent_node_name = node_->get_name();
  for (const std::string& planning_pipeline_name : planning_pipeline_names)
  {
    // Initialize planning pipelines from configured child namespaces
    static rclcpp::Node::SharedPtr child_node = rclcpp::Node::make_shared(planning_pipeline_name, parent_node_name);
    planning_pipeline::PlanningPipelinePtr pipeline(new planning_pipeline::PlanningPipeline(
        planning_scene_->getRobotModel(), child_node, "planning_plugin", "request_adapters"));

    // Verify the pipeline has successfully initialized a planner
    if (!pipeline->getPlannerManager())
    {
      RCLCPP_ERROR(LOGGER, "Failed to initialize planning pipeline '%s'", planning_pipeline_name.c_str());
      continue;
    }

    // Disable visualizations and store pipeline
    pipeline->displayComputedMotionPlans(false);
    pipeline->checkSolutionPaths(false);
    planning_pipelines_[planning_pipeline_name] = pipeline;
  }

  // Error check
  if (planning_pipelines_.empty())
    RCLCPP_ERROR(LOGGER, "No planning pipelines have been loaded. Nothing to do for the benchmarking service.");
  else
  {
    RCLCPP_INFO(LOGGER, "Available planning pipelines:");
    for (const std::pair<const std::string, planning_pipeline::PlanningPipelinePtr>& entry : planning_pipelines_)
      RCLCPP_INFO_STREAM(LOGGER, "Pipeline: " << entry.first << ", Planner: " << entry.second->getPlannerPluginName());
  }
}

void BenchmarkExecutor::clear()
{
  if (pss_)
  {
    delete pss_;
    pss_ = nullptr;
  }
  if (psws_)
  {
    delete psws_;
    psws_ = nullptr;
  }
  if (rs_)
  {
    delete rs_;
    rs_ = nullptr;
  }
  if (cs_)
  {
    delete cs_;
    cs_ = nullptr;
  }
  if (tcs_)
  {
    delete tcs_;
    tcs_ = nullptr;
  }

  benchmark_data_.clear();
  pre_event_fns_.clear();
  post_event_fns_.clear();
  planner_start_fns_.clear();
  planner_completion_fns_.clear();
  query_start_fns_.clear();
  query_end_fns_.clear();
}

void BenchmarkExecutor::addPreRunEvent(const PreRunEventFunction& func)
{
  pre_event_fns_.push_back(func);
}

void BenchmarkExecutor::addPostRunEvent(const PostRunEventFunction& func)
{
  post_event_fns_.push_back(func);
}

void BenchmarkExecutor::addPlannerStartEvent(const PlannerStartEventFunction& func)
{
  planner_start_fns_.push_back(func);
}

void BenchmarkExecutor::addPlannerCompletionEvent(const PlannerCompletionEventFunction& func)
{
  planner_completion_fns_.push_back(func);
}

void BenchmarkExecutor::addQueryStartEvent(const QueryStartEventFunction& func)
{
  query_start_fns_.push_back(func);
}

void BenchmarkExecutor::addQueryCompletionEvent(const QueryCompletionEventFunction& func)
{
  query_end_fns_.push_back(func);
}

bool BenchmarkExecutor::runBenchmarks(const BenchmarkOptions& opts)
{
  if (planning_pipelines_.empty())
  {
    RCLCPP_ERROR(LOGGER, "No planning pipelines configured.  Did you call BenchmarkExecutor::initialize?");
    return false;
  }

  std::vector<BenchmarkRequest> queries;
  moveit_msgs::msg::PlanningScene scene_msg;

  if (initializeBenchmarks(opts, scene_msg, queries))
  {
    if (!queriesAndPlannersCompatible(queries, opts.getPlanningPipelineConfigurations()))
      return false;

    for (std::size_t i = 0; i < queries.size(); ++i)
    {
      // Configure planning scene
      if (scene_msg.robot_model_name != planning_scene_->getRobotModel()->getName())
      {
        // Clear all geometry from the scene
        planning_scene_->getWorldNonConst()->clearObjects();
        planning_scene_->getCurrentStateNonConst().clearAttachedBodies();
        planning_scene_->getCurrentStateNonConst().setToDefaultValues();

        planning_scene_->processPlanningSceneWorldMsg(scene_msg.world);
      }
      else
        planning_scene_->usePlanningSceneMsg(scene_msg);

      // Calling query start events
      for (QueryStartEventFunction& query_start_fn : query_start_fns_)
        query_start_fn(queries[i].request, planning_scene_);

      RCLCPP_INFO(LOGGER, "Benchmarking query '%s' (%lu of %lu)", queries[i].name.c_str(), i + 1, queries.size());
      std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();
      runBenchmark(queries[i].request, options_.getPlanningPipelineConfigurations(), options_.getNumRuns());
      std::chrono::duration<double> dt = std::chrono::system_clock::now() - start_time;
      double duration = dt.count();

      for (QueryCompletionEventFunction& query_end_fn : query_end_fns_)
        query_end_fn(queries[i].request, planning_scene_);

      writeOutput(queries[i], boost::posix_time::to_iso_extended_string(toBoost(start_time)), duration);
    }

    return true;
  }
  return false;
}

bool BenchmarkExecutor::queriesAndPlannersCompatible(const std::vector<BenchmarkRequest>& requests,
                                                     const std::map<std::string, std::vector<std::string>>& /*planners*/)
{
  // Make sure that the planner interfaces can service the desired queries
  for (const std::pair<const std::string, planning_pipeline::PlanningPipelinePtr>& pipeline_entry : planning_pipelines_)
  {
    for (const BenchmarkRequest& request : requests)
    {
      if (!pipeline_entry.second->getPlannerManager()->canServiceRequest(request.request))
      {
        RCLCPP_ERROR(LOGGER, "Interface '%s' in pipeline '%s' cannot service the benchmark request '%s'",
                     pipeline_entry.second->getPlannerPluginName().c_str(), pipeline_entry.first.c_str(),
                     request.name.c_str());
        return false;
      }
    }
  }

  return true;
}

bool BenchmarkExecutor::initializeBenchmarks(const BenchmarkOptions& opts, moveit_msgs::msg::PlanningScene& scene_msg,
                                             std::vector<BenchmarkRequest>& requests)
{
  if (!plannerConfigurationsExist(opts.getPlanningPipelineConfigurations(), opts.getGroupName()))
    return false;

  std::vector<StartState> start_states;
  std::vector<PathConstraints> path_constraints;
  std::vector<PathConstraints> goal_constraints;
  std::vector<TrajectoryConstraints> traj_constraints;
  std::vector<BenchmarkRequest> queries;

  if (!loadBenchmarkQueryData(opts, scene_msg, start_states, path_constraints, goal_constraints, traj_constraints,
                              queries))
  {
    RCLCPP_ERROR(LOGGER, "Failed to load benchmark query data");
    return false;
  }

  RCLCPP_INFO(
      LOGGER,
      "Benchmark loaded %lu starts, %lu goals, %lu path constraints, %lu trajectory constraints, and %lu queries",
      start_states.size(), goal_constraints.size(), path_constraints.size(), traj_constraints.size(), queries.size());

  moveit_msgs::msg::WorkspaceParameters workspace_parameters = opts.getWorkspaceParameters();
  // Make sure that workspace_parameters are set
  if (workspace_parameters.min_corner.x == workspace_parameters.max_corner.x &&
      workspace_parameters.min_corner.x == 0.0 &&
      workspace_parameters.min_corner.y == workspace_parameters.max_corner.y &&
      workspace_parameters.min_corner.y == 0.0 &&
      workspace_parameters.min_corner.z == workspace_parameters.max_corner.z &&
      workspace_parameters.min_corner.z == 0.0)
  {
    workspace_parameters.min_corner.x = workspace_parameters.min_corner.y = workspace_parameters.min_corner.z = -5.0;

    workspace_parameters.max_corner.x = workspace_parameters.max_corner.y = workspace_parameters.max_corner.z = 5.0;
  }

  std::vector<double> goal_offset;
  opts.getGoalOffsets(goal_offset);

  // Create the combinations of BenchmarkRequests

  // 1) Create requests for combinations of start states,
  //    goal constraints, and path constraints
  for (PathConstraints& goal_constraint : goal_constraints)
  {
    // Common benchmark request properties
    BenchmarkRequest brequest;
    brequest.name = goal_constraint.name;
    brequest.request.workspace_parameters = workspace_parameters;
    brequest.request.goal_constraints = goal_constraint.constraints;
    brequest.request.group_name = opts.getGroupName();
    brequest.request.allowed_planning_time = opts.getTimeout();
    brequest.request.num_planning_attempts = 1;

    if (brequest.request.goal_constraints.size() == 1 &&
        brequest.request.goal_constraints[0].position_constraints.size() == 1 &&
        brequest.request.goal_constraints[0].orientation_constraints.size() == 1 &&
        brequest.request.goal_constraints[0].visibility_constraints.empty() &&
        brequest.request.goal_constraints[0].joint_constraints.empty())
      shiftConstraintsByOffset(brequest.request.goal_constraints[0], goal_offset);

    std::vector<BenchmarkRequest> request_combos;
    createRequestCombinations(brequest, start_states, path_constraints, request_combos);
    requests.insert(requests.end(), request_combos.begin(), request_combos.end());
  }

  // 2) Existing queries are treated like goal constraints.
  //    Create all combos of query, start states, and path constraints
  for (BenchmarkRequest& query : queries)
  {
    // Common benchmark request properties
    BenchmarkRequest brequest;
    brequest.name = query.name;
    brequest.request = query.request;
    brequest.request.group_name = opts.getGroupName();
    brequest.request.allowed_planning_time = opts.getTimeout();
    brequest.request.num_planning_attempts = 1;

    // Make sure that workspace_parameters are set
    if (brequest.request.workspace_parameters.min_corner.x == brequest.request.workspace_parameters.max_corner.x &&
        brequest.request.workspace_parameters.min_corner.x == 0.0 &&
        brequest.request.workspace_parameters.min_corner.y == brequest.request.workspace_parameters.max_corner.y &&
        brequest.request.workspace_parameters.min_corner.y == 0.0 &&
        brequest.request.workspace_parameters.min_corner.z == brequest.request.workspace_parameters.max_corner.z &&
        brequest.request.workspace_parameters.min_corner.z == 0.0)
    {
      // ROS_WARN("Workspace parameters are not set for request %s.  Setting defaults", queries[i].name.c_str());
      brequest.request.workspace_parameters = workspace_parameters;
    }

    // Create all combinations of start states and path constraints
    std::vector<BenchmarkRequest> request_combos;
    createRequestCombinations(brequest, start_states, path_constraints, request_combos);
    requests.insert(requests.end(), request_combos.begin(), request_combos.end());
  }

  // 3) Trajectory constraints are also treated like goal constraints
  for (TrajectoryConstraints& traj_constraint : traj_constraints)
  {
    // Common benchmark request properties
    BenchmarkRequest brequest;
    brequest.name = traj_constraint.name;
    brequest.request.trajectory_constraints = traj_constraint.constraints;
    brequest.request.group_name = opts.getGroupName();
    brequest.request.allowed_planning_time = opts.getTimeout();
    brequest.request.num_planning_attempts = 1;

    if (brequest.request.trajectory_constraints.constraints.size() == 1 &&
        brequest.request.trajectory_constraints.constraints[0].position_constraints.size() == 1 &&
        brequest.request.trajectory_constraints.constraints[0].orientation_constraints.size() == 1 &&
        brequest.request.trajectory_constraints.constraints[0].visibility_constraints.empty() &&
        brequest.request.trajectory_constraints.constraints[0].joint_constraints.empty())
      shiftConstraintsByOffset(brequest.request.trajectory_constraints.constraints[0], goal_offset);

    std::vector<BenchmarkRequest> request_combos;
    std::vector<PathConstraints> no_path_constraints;
    createRequestCombinations(brequest, start_states, no_path_constraints, request_combos);
    requests.insert(requests.end(), request_combos.begin(), request_combos.end());
  }

  options_ = opts;
  return true;
}

bool BenchmarkExecutor::loadBenchmarkQueryData(const BenchmarkOptions& opts, moveit_msgs::msg::PlanningScene& scene_msg,
                                               std::vector<StartState>& start_states,
                                               std::vector<PathConstraints>& path_constraints,
                                               std::vector<PathConstraints>& goal_constraints,
                                               std::vector<TrajectoryConstraints>& traj_constraints,
                                               std::vector<BenchmarkRequest>& queries)
{
  try
  {
    warehouse_ros::DatabaseConnection::Ptr warehouse_connection = dbloader.loadDatabase();
    warehouse_connection->setParams(opts.getHostName(), opts.getPort(), 20);
    if (warehouse_connection->connect())
    {
      pss_ = new moveit_warehouse::PlanningSceneStorage(warehouse_connection);
      psws_ = new moveit_warehouse::PlanningSceneWorldStorage(warehouse_connection);
      rs_ = new moveit_warehouse::RobotStateStorage(warehouse_connection);
      cs_ = new moveit_warehouse::ConstraintsStorage(warehouse_connection);
      tcs_ = new moveit_warehouse::TrajectoryConstraintsStorage(warehouse_connection);
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Failed to connect to DB");
      return false;
    }
  }
  catch (std::exception& e)
  {
    RCLCPP_ERROR(LOGGER, "Failed to initialize benchmark server: '%s'", e.what());
    return false;
  }

  return loadPlanningScene(opts.getSceneName(), scene_msg) && loadStates(opts.getStartStateRegex(), start_states) &&
         loadPathConstraints(opts.getGoalConstraintRegex(), goal_constraints) &&
         loadPathConstraints(opts.getPathConstraintRegex(), path_constraints) &&
         loadTrajectoryConstraints(opts.getTrajectoryConstraintRegex(), traj_constraints) &&
         loadQueries(opts.getQueryRegex(), opts.getSceneName(), queries);
  return true;
}

void BenchmarkExecutor::shiftConstraintsByOffset(moveit_msgs::msg::Constraints& constraints,
                                                 const std::vector<double>& offset)
{
  Eigen::Isometry3d offset_tf(Eigen::AngleAxis<double>(offset[3], Eigen::Vector3d::UnitX()) *
                              Eigen::AngleAxis<double>(offset[4], Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxis<double>(offset[5], Eigen::Vector3d::UnitZ()));
  offset_tf.translation() = Eigen::Vector3d(offset[0], offset[1], offset[2]);

  geometry_msgs::msg::Pose constraint_pose_msg;
  constraint_pose_msg.position = constraints.position_constraints[0].constraint_region.primitive_poses[0].position;
  constraint_pose_msg.orientation = constraints.orientation_constraints[0].orientation;
  Eigen::Isometry3d constraint_pose;
  tf2::fromMsg(constraint_pose_msg, constraint_pose);

  Eigen::Isometry3d new_pose = constraint_pose * offset_tf;
  geometry_msgs::msg::Pose new_pose_msg;
  new_pose_msg = tf2::toMsg(new_pose);

  constraints.position_constraints[0].constraint_region.primitive_poses[0].position = new_pose_msg.position;
  constraints.orientation_constraints[0].orientation = new_pose_msg.orientation;
}

void BenchmarkExecutor::createRequestCombinations(const BenchmarkRequest& brequest,
                                                  const std::vector<StartState>& start_states,
                                                  const std::vector<PathConstraints>& path_constraints,
                                                  std::vector<BenchmarkRequest>& requests)
{
  // Use default start state
  if (start_states.empty())
  {
    // Adding path constraints
    for (const PathConstraints& path_constraint : path_constraints)
    {
      BenchmarkRequest new_brequest = brequest;
      new_brequest.request.path_constraints = path_constraint.constraints[0];
      new_brequest.name = brequest.name + "_" + path_constraint.name;
      requests.push_back(new_brequest);
    }

    if (path_constraints.empty())
      requests.push_back(brequest);
  }
  else  // Create a request for each start state specified
  {
    for (const StartState& start_state : start_states)
    {
      // Skip start states that have the same name as the goal
      if (start_state.name == brequest.name)
        continue;

      BenchmarkRequest new_brequest = brequest;
      new_brequest.request.start_state = start_state.state;

      // Duplicate the request for each of the path constraints
      for (const PathConstraints& path_constraint : path_constraints)
      {
        new_brequest.request.path_constraints = path_constraint.constraints[0];
        new_brequest.name = start_state.name + "_" + new_brequest.name + "_" + path_constraint.name;
        requests.push_back(new_brequest);
      }

      if (path_constraints.empty())
      {
        new_brequest.name = start_state.name + "_" + brequest.name;
        requests.push_back(new_brequest);
      }
    }
  }
}

bool BenchmarkExecutor::plannerConfigurationsExist(
    const std::map<std::string, std::vector<std::string>>& pipeline_configurations, const std::string& group_name)
{
  // Make sure planner plugins exist
  for (const std::pair<const std::string, std::vector<std::string>>& pipeline_config_entry : pipeline_configurations)
  {
    bool pipeline_exists = false;
    for (const std::pair<const std::string, planning_pipeline::PlanningPipelinePtr>& pipeline_entry :
         planning_pipelines_)
    {
      pipeline_exists = pipeline_entry.first == pipeline_config_entry.first;
      if (pipeline_exists)
        break;
    }

    if (!pipeline_exists)
    {
      RCLCPP_ERROR(LOGGER, "Planning pipeline '%s' does NOT exist", pipeline_config_entry.first.c_str());
      return false;
    }
  }

  // Make sure planners exist within those pipelines
  for (const std::pair<const std::string, std::vector<std::string>>& entry : pipeline_configurations)
  {
    planning_interface::PlannerManagerPtr pm = planning_pipelines_[entry.first]->getPlannerManager();
    const planning_interface::PlannerConfigurationMap& config_map = pm->getPlannerConfigurations();

    // if the planner is chomp or stomp skip this function and return true for checking planner configurations for the
    // planning group otherwise an error occurs, because for OMPL a specific planning algorithm needs to be defined for
    // a planning group, whereas with STOMP and CHOMP this is not necessary
    if (pm->getDescription().compare("stomp") || pm->getDescription().compare("chomp"))
      continue;

    for (std::size_t i = 0; i < entry.second.size(); ++i)
    {
      bool planner_exists = false;
      for (const std::pair<const std::string, planning_interface::PlannerConfigurationSettings>& config_entry :
           config_map)
      {
        std::string planner_name = group_name + "[" + entry.second[i] + "]";
        planner_exists = (config_entry.second.group == group_name && config_entry.second.name == planner_name);
      }

      if (!planner_exists)
      {
        RCLCPP_ERROR(LOGGER, "Planner '%s' does NOT exist for group '%s' in pipeline '%s'", entry.second[i].c_str(),
                     group_name.c_str(), entry.first.c_str());
        std::cout << "There are " << config_map.size() << " planner entries: " << '\n';
        for (const auto& config_map_entry : config_map)
          std::cout << config_map_entry.second.name << '\n';
        return false;
      }
    }
  }

  return true;
}

bool BenchmarkExecutor::loadPlanningScene(const std::string& scene_name, moveit_msgs::msg::PlanningScene& scene_msg)
{
  bool ok = false;
  try
  {
    if (pss_->hasPlanningScene(scene_name))  // whole planning scene
    {
      moveit_warehouse::PlanningSceneWithMetadata pswm;
      ok = pss_->getPlanningScene(pswm, scene_name);
      scene_msg = static_cast<moveit_msgs::msg::PlanningScene>(*pswm);

      if (!ok)
        RCLCPP_ERROR(LOGGER, "Failed to load planning scene '%s'", scene_name.c_str());
    }
    else if (psws_->hasPlanningSceneWorld(scene_name))  // Just the world (no robot)
    {
      moveit_warehouse::PlanningSceneWorldWithMetadata pswwm;
      ok = psws_->getPlanningSceneWorld(pswwm, scene_name);
      scene_msg.world = static_cast<moveit_msgs::msg::PlanningSceneWorld>(*pswwm);
      scene_msg.robot_model_name =
          "NO ROBOT INFORMATION. ONLY WORLD GEOMETRY";  // this will be fixed when running benchmark

      if (!ok)
        RCLCPP_ERROR(LOGGER, "Failed to load planning scene '%s'", scene_name.c_str());
    }
    else
      RCLCPP_ERROR(LOGGER, "Failed to find planning scene '%s'", scene_name.c_str());
  }
  catch (std::exception& ex)
  {
    RCLCPP_ERROR(LOGGER, "Error loading planning scene: %s", ex.what());
  }
  RCLCPP_INFO(LOGGER, "Loaded planning scene successfully");
  return ok;
}

bool BenchmarkExecutor::loadQueries(const std::string& regex, const std::string& scene_name,
                                    std::vector<BenchmarkRequest>& queries)
{
  if (regex.empty())
    return true;

  std::vector<std::string> query_names;
  try
  {
    pss_->getPlanningQueriesNames(regex, query_names, scene_name);
  }
  catch (std::exception& ex)
  {
    RCLCPP_ERROR(LOGGER, "Error loading motion planning queries: %s", ex.what());
    return false;
  }

  if (query_names.empty())
  {
    RCLCPP_ERROR(LOGGER, "Scene '%s' has no associated queries", scene_name.c_str());
    return false;
  }

  for (const std::string& query_name : query_names)
  {
    moveit_warehouse::MotionPlanRequestWithMetadata planning_query;
    try
    {
      pss_->getPlanningQuery(planning_query, scene_name, query_name);
    }
    catch (std::exception& ex)
    {
      RCLCPP_ERROR(LOGGER, "Error loading motion planning query '%s': %s", query_name.c_str(), ex.what());
      continue;
    }

    BenchmarkRequest query;
    query.name = query_name;
    query.request = static_cast<moveit_msgs::msg::MotionPlanRequest>(*planning_query);
    queries.push_back(query);
  }
  RCLCPP_INFO(LOGGER, "Loaded queries successfully");
  return true;
}

bool BenchmarkExecutor::loadStates(const std::string& regex, std::vector<StartState>& start_states)
{
  if (!regex.empty())
  {
    std::regex start_regex(regex);
    std::vector<std::string> state_names;
    rs_->getKnownRobotStates(state_names);
    for (const std::string& state_name : state_names)
    {
      std::smatch match;
      if (std::regex_match(state_name, match, start_regex))
      {
        moveit_warehouse::RobotStateWithMetadata robot_state;
        try
        {
          if (rs_->getRobotState(robot_state, state_name))
          {
            StartState start_state;
            start_state.state = moveit_msgs::msg::RobotState(*robot_state);
            start_state.name = state_name;
            start_states.push_back(start_state);
          }
        }
        catch (std::exception& ex)
        {
          RCLCPP_ERROR(LOGGER, "Runtime error when loading state '%s': %s", state_name.c_str(), ex.what());
          continue;
        }
      }
    }

    if (start_states.empty())
      RCLCPP_WARN(LOGGER, "No stored states matched the provided start state regex: '%s'", regex.c_str());
  }
  RCLCPP_INFO(LOGGER, "Loaded states successfully");
  return true;
}

bool BenchmarkExecutor::loadPathConstraints(const std::string& regex, std::vector<PathConstraints>& constraints)
{
  if (!regex.empty())
  {
    std::vector<std::string> cnames;
    cs_->getKnownConstraints(regex, cnames);

    for (const std::string& cname : cnames)
    {
      moveit_warehouse::ConstraintsWithMetadata constr;
      try
      {
        if (cs_->getConstraints(constr, cname))
        {
          PathConstraints constraint;
          constraint.constraints.push_back(*constr);
          constraint.name = cname;
          constraints.push_back(constraint);
        }
      }
      catch (std::exception& ex)
      {
        RCLCPP_ERROR(LOGGER, "Runtime error when loading path constraint '%s': %s", cname.c_str(), ex.what());
        continue;
      }
    }

    if (constraints.empty())
      RCLCPP_WARN(LOGGER, "No path constraints found that match regex: '%s'", regex.c_str());
    else
      RCLCPP_INFO(LOGGER, "Loaded path constraints successfully");
  }
  return true;
}

bool BenchmarkExecutor::loadTrajectoryConstraints(const std::string& regex,
                                                  std::vector<TrajectoryConstraints>& constraints)
{
  if (!regex.empty())
  {
    std::vector<std::string> cnames;
    tcs_->getKnownTrajectoryConstraints(regex, cnames);

    for (const std::string& cname : cnames)
    {
      moveit_warehouse::TrajectoryConstraintsWithMetadata constr;
      try
      {
        if (tcs_->getTrajectoryConstraints(constr, cname))
        {
          TrajectoryConstraints constraint;
          constraint.constraints = *constr;
          constraint.name = cname;
          constraints.push_back(constraint);
        }
      }
      catch (std::exception& ex)
      {
        RCLCPP_ERROR(LOGGER, "Runtime error when loading trajectory constraint '%s': %s", cname.c_str(), ex.what());
        continue;
      }
    }

    if (constraints.empty())
      RCLCPP_WARN(LOGGER, "No trajectory constraints found that match regex: '%s'", regex.c_str());
    else
      RCLCPP_INFO(LOGGER, "Loaded trajectory constraints successfully");
  }
  return true;
}

void BenchmarkExecutor::runBenchmark(moveit_msgs::msg::MotionPlanRequest request,
                                     const std::map<std::string, std::vector<std::string>>& pipeline_map, int runs)
{
  benchmark_data_.clear();

  unsigned int num_planners = 0;
  for (const std::pair<const std::string, std::vector<std::string>>& pipeline_entry : pipeline_map)
    num_planners += pipeline_entry.second.size();

  boost::progress_display progress(num_planners * runs, std::cout);

  // Iterate through all planning pipelines
  for (const std::pair<const std::string, std::vector<std::string>>& pipeline_entry : pipeline_map)
  {
    planning_pipeline::PlanningPipelinePtr planning_pipeline = planning_pipelines_[pipeline_entry.first];
    // Use the planning context if the pipeline only contains the planner plugin
    bool use_planning_context = planning_pipeline->getAdapterPluginNames().empty();
    // Iterate through all planners configured for the pipeline
    for (const std::string& planner_id : pipeline_entry.second)
    {
      // This container stores all of the benchmark data for this planner
      PlannerBenchmarkData planner_data(runs);
      // This vector stores all motion plan results for further evaluation
      std::vector<planning_interface::MotionPlanDetailedResponse> responses(runs);
      std::vector<bool> solved(runs);

      request.planner_id = planner_id;

      // Planner start events
      for (PlannerStartEventFunction& planner_start_fn : planner_start_fns_)
        planner_start_fn(request, planner_data);

      planning_interface::PlanningContextPtr planning_context;
      if (use_planning_context)
        planning_context = planning_pipeline->getPlannerManager()->getPlanningContext(planning_scene_, request);

      // Iterate runs
      for (int j = 0; j < runs; ++j)
      {
        // Pre-run events
        for (PreRunEventFunction& pre_event_fn : pre_event_fns_)
          pre_event_fn(request);

        // Solve problem
        std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
        if (use_planning_context)
        {
          solved[j] = planning_context->solve(responses[j]);
        }
        else
        {
          // The planning pipeline does not support MotionPlanDetailedResponse
          planning_interface::MotionPlanResponse response;
          solved[j] = planning_pipeline->generatePlan(planning_scene_, request, response);
          responses[j].error_code_ = response.error_code_;
          if (response.trajectory_)
          {
            responses[j].description_.push_back("plan");
            responses[j].trajectory_.push_back(response.trajectory_);
            responses[j].processing_time_.push_back(response.planning_time_);
          }
        }
        std::chrono::duration<double> dt = std::chrono::system_clock::now() - start;
        double total_time = dt.count();

        // Collect data
        start = std::chrono::system_clock::now();

        // Post-run events
        for (PostRunEventFunction& post_event_fn : post_event_fns_)
          post_event_fn(request, responses[j], planner_data[j]);
        collectMetrics(planner_data[j], responses[j], solved[j], total_time);
        dt = std::chrono::system_clock::now() - start;
        double metrics_time = dt.count();
        RCLCPP_DEBUG(LOGGER, "Spent %lf seconds collecting metrics", metrics_time);

        ++progress;
      }

      computeAveragePathSimilarities(planner_data, responses, solved);

      // Planner completion events
      for (PlannerCompletionEventFunction& planner_completion_fn : planner_completion_fns_)
        planner_completion_fn(request, planner_data);

      benchmark_data_.push_back(planner_data);
    }
  }
}

void BenchmarkExecutor::collectMetrics(PlannerRunData& metrics,
                                       const planning_interface::MotionPlanDetailedResponse& mp_res, bool solved,
                                       double total_time)
{
  metrics["time REAL"] = moveit::core::toString(total_time);
  metrics["solved BOOLEAN"] = solved ? "true" : "false";

  if (solved)
  {
    // Analyzing the trajectory(ies) geometrically
    double traj_len = 0.0;   // trajectory length
    double clearance = 0.0;  // trajectory clearance (average)
    bool correct = true;     // entire trajectory collision free and in bounds

    double process_time = total_time;
    for (std::size_t j = 0; j < mp_res.trajectory_.size(); ++j)
    {
      correct = true;
      traj_len = 0.0;
      clearance = 0.0;
      const robot_trajectory::RobotTrajectory& p = *mp_res.trajectory_[j];

      // compute path length
      traj_len = robot_trajectory::path_length(p);

      // compute correctness and clearance
      collision_detection::CollisionRequest req;
      for (std::size_t k = 0; k < p.getWayPointCount(); ++k)
      {
        collision_detection::CollisionResult res;
        planning_scene_->checkCollisionUnpadded(req, res, p.getWayPoint(k));
        if (res.collision)
          correct = false;
        if (!p.getWayPoint(k).satisfiesBounds())
          correct = false;
        double d = planning_scene_->distanceToCollisionUnpadded(p.getWayPoint(k));
        if (d > 0.0)  // in case of collision, distance is negative
          clearance += d;
      }
      clearance /= (double)p.getWayPointCount();

      // compute smoothness
      const auto smoothness = [&]() {
        const auto s = robot_trajectory::smoothness(p);
        return s.has_value() ? s.value() : 0.0;
      }();

      metrics["path_" + mp_res.description_[j] + "_correct BOOLEAN"] = correct ? "true" : "false";
      metrics["path_" + mp_res.description_[j] + "_length REAL"] = moveit::core::toString(traj_len);
      metrics["path_" + mp_res.description_[j] + "_clearance REAL"] = moveit::core::toString(clearance);
      metrics["path_" + mp_res.description_[j] + "_smoothness REAL"] = moveit::core::toString(smoothness);
      metrics["path_" + mp_res.description_[j] + "_time REAL"] = moveit::core::toString(mp_res.processing_time_[j]);

      if (j == mp_res.trajectory_.size() - 1)
      {
        metrics["final_path_correct BOOLEAN"] = correct ? "true" : "false";
        metrics["final_path_length REAL"] = moveit::core::toString(traj_len);
        metrics["final_path_clearance REAL"] = moveit::core::toString(clearance);
        metrics["final_path_smoothness REAL"] = moveit::core::toString(smoothness);
        metrics["final_path_time REAL"] = moveit::core::toString(mp_res.processing_time_[j]);
      }
      process_time -= mp_res.processing_time_[j];
    }
    if (process_time <= 0.0)
      process_time = 0.0;
    metrics["process_time REAL"] = moveit::core::toString(process_time);
  }
}

void BenchmarkExecutor::computeAveragePathSimilarities(
    PlannerBenchmarkData& planner_data, const std::vector<planning_interface::MotionPlanDetailedResponse>& responses,
    const std::vector<bool>& solved)
{
  RCLCPP_INFO(LOGGER, "Computing result path similarity");
  const size_t result_count = planner_data.size();
  size_t unsolved = std::count_if(solved.begin(), solved.end(), [](bool s) { return !s; });
  std::vector<double> average_distances(responses.size());
  for (size_t first_traj_i = 0; first_traj_i < result_count; ++first_traj_i)
  {
    // If trajectory was not solved there is no valid average distance so it's set to max double only
    if (!solved[first_traj_i])
    {
      average_distances[first_traj_i] = std::numeric_limits<double>::max();
      continue;
    }
    // Iterate all result trajectories that haven't been compared yet
    for (size_t second_traj_i = first_traj_i + 1; second_traj_i < result_count; ++second_traj_i)
    {
      // Ignore if other result has not been solved
      if (!solved[second_traj_i])
        continue;

      // Get final trajectories
      const robot_trajectory::RobotTrajectory& traj_first = *responses[first_traj_i].trajectory_.back();
      const robot_trajectory::RobotTrajectory& traj_second = *responses[second_traj_i].trajectory_.back();

      // Compute trajectory distance
      double trajectory_distance;
      if (!computeTrajectoryDistance(traj_first, traj_second, trajectory_distance))
        continue;

      // Add average distance to counters of both trajectories
      average_distances[first_traj_i] += trajectory_distance;
      average_distances[second_traj_i] += trajectory_distance;
    }
    // Normalize average distance by number of actual comparisons
    average_distances[first_traj_i] /= result_count - unsolved - 1;
  }

  // Store results in planner_data
  for (size_t i = 0; i < result_count; ++i)
    planner_data[i]["average_waypoint_distance REAL"] = moveit::core::toString(average_distances[i]);
}

bool BenchmarkExecutor::computeTrajectoryDistance(const robot_trajectory::RobotTrajectory& traj_first,
                                                  const robot_trajectory::RobotTrajectory& traj_second,
                                                  double& result_distance)
{
  // Abort if trajectories are empty
  if (traj_first.empty() || traj_second.empty())
    return false;

  // Waypoint counter
  size_t pos_first = 0;
  size_t pos_second = 0;
  const size_t max_pos_first = traj_first.getWayPointCount() - 1;
  const size_t max_pos_second = traj_second.getWayPointCount() - 1;

  // Compute total distance between pairwise waypoints of both trajectories.
  // The selection of waypoint pairs is based on what steps results in the minimal distance between the next pair of
  // waypoints. We first check what steps are still possible or if we reached the end of the trajectories. Then we
  // compute the pairwise waypoint distances of the pairs from increasing both, the first, or the second trajectory.
  // Finally we select the pair that results in the minimal distance, summarize the total distance and iterate
  // accordingly. After that we compute the average trajectory distance by normalizing over the number of steps.
  double total_distance = 0;
  size_t steps = 0;
  double current_distance = traj_first.getWayPoint(pos_first).distance(traj_second.getWayPoint(pos_second));
  while (true)
  {
    // Keep track of total distance and number of comparisons
    total_distance += current_distance;
    ++steps;
    if (pos_first == max_pos_first && pos_second == max_pos_second)  // end reached
      break;

    // Determine what steps are still possible
    bool can_up_first = pos_first < max_pos_first;
    bool can_up_second = pos_second < max_pos_second;
    bool can_up_both = can_up_first && can_up_second;

    // Compute pair-wise waypoint distances (increasing both, first, or second trajectories).
    double up_both = std::numeric_limits<double>::max();
    double up_first = std::numeric_limits<double>::max();
    double up_second = std::numeric_limits<double>::max();
    if (can_up_both)
      up_both = traj_first.getWayPoint(pos_first + 1).distance(traj_second.getWayPoint(pos_second + 1));
    if (can_up_first)
      up_first = traj_first.getWayPoint(pos_first + 1).distance(traj_second.getWayPoint(pos_second));
    if (can_up_second)
      up_second = traj_first.getWayPoint(pos_first).distance(traj_second.getWayPoint(pos_second + 1));

    // Select actual step, store new distance value and iterate trajectory positions
    if (can_up_both && up_both < up_first && up_both < up_second)
    {
      ++pos_first;
      ++pos_second;
      current_distance = up_both;
    }
    else if ((can_up_first && up_first < up_second) || !can_up_second)
    {
      ++pos_first;
      current_distance = up_first;
    }
    else if (can_up_second)
    {
      ++pos_second;
      current_distance = up_second;
    }
  }
  // Normalize trajectory distance by number of comparison steps
  result_distance = total_distance / static_cast<double>(steps);
  return true;
}

void BenchmarkExecutor::writeOutput(const BenchmarkRequest& brequest, const std::string& start_time,
                                    double benchmark_duration)
{
  const std::map<std::string, std::vector<std::string>>& pipelines = options_.getPlanningPipelineConfigurations();

  size_t num_planners = 0;
  for (const std::pair<const std::string, std::vector<std::string>>& pipeline : pipelines)
    num_planners += pipeline.second.size();

  std::string hostname = getHostname();
  if (hostname.empty())
    hostname = "UNKNOWN";

  std::string filename = options_.getOutputDirectory();
  if (!filename.empty() && filename[filename.size() - 1] != '/')
    filename.append("/");

  // Ensure directories exist
  std::filesystem::create_directories(filename);

  filename += (options_.getBenchmarkName().empty() ? "" : options_.getBenchmarkName() + "_") + brequest.name + "_" +
              getHostname() + "_" + start_time + ".log";
  std::ofstream out(filename.c_str());
  if (!out)
  {
    RCLCPP_ERROR(LOGGER, "Failed to open '%s' for benchmark output", filename.c_str());
    return;
  }

  out << "MoveIt version " << MOVEIT_VERSION_STR << '\n';
  out << "Experiment " << brequest.name << '\n';
  out << "Running on " << hostname << '\n';
  out << "Starting at " << start_time << '\n';

  // Experiment setup
  moveit_msgs::msg::PlanningScene scene_msg;
  planning_scene_->getPlanningSceneMsg(scene_msg);
  out << "<<<|" << '\n';
  // TODO(YuYan): implement stream print for the message formats
  // out << "Motion plan request:" << '\n' << brequest.request << '\n';
  // out << "Planning scene: " << '\n' << scene_msg << '\n' << "|>>>" << '\n';
  out << "Motion plan request:" << '\n'
      << "  planner_id: " << brequest.request.planner_id << '\n'
      << "  group_name: " << brequest.request.group_name << '\n'
      << "  num_planning_attempts: " << brequest.request.num_planning_attempts << '\n'
      << "  allowed_planning_time: " << brequest.request.allowed_planning_time << '\n';
  out << "Planning scene:" << '\n'
      << "  scene_name: " << scene_msg.name << '\n'
      << "  robot_model_name: " << scene_msg.robot_model_name << '\n'
      << "|>>>" << '\n';

  // Not writing optional cpu information

  // The real random seed is unknown.  Writing a fake value
  out << "0 is the random seed" << '\n';
  out << brequest.request.allowed_planning_time << " seconds per run" << '\n';
  // There is no memory cap
  out << "-1 MB per run" << '\n';
  out << options_.getNumRuns() << " runs per planner" << '\n';
  out << benchmark_duration << " seconds spent to collect the data" << '\n';

  // No enum types
  out << "0 enum types" << '\n';

  out << num_planners << " planners" << '\n';

  size_t run_id = 0;
  for (const std::pair<const std::string, std::vector<std::string>>& pipeline : pipelines)
  {
    for (std::size_t i = 0; i < pipeline.second.size(); ++i, ++run_id)
    {
      // Write the name of the planner and the used pipeline
      out << pipeline.second[i] << " (" << pipeline.first << ")" << '\n';

      // in general, we could have properties specific for a planner;
      // right now, we do not include such properties
      out << "0 common properties" << '\n';

      // Create a list of the benchmark properties for this planner
      std::set<std::string> properties_set;
      for (PlannerRunData& planner_run_data : benchmark_data_[run_id])  // each run of this planner
        for (PlannerRunData::const_iterator pit = planner_run_data.begin(); pit != planner_run_data.end();
             ++pit)  // each benchmark property of the given run
          properties_set.insert(pit->first);

      // Writing property list
      out << properties_set.size() << " properties for each run" << '\n';
      for (const std::string& property : properties_set)
        out << property << '\n';

      // Number of runs
      out << benchmark_data_[run_id].size() << " runs" << '\n';

      // And the benchmark properties
      for (PlannerRunData& planner_run_data : benchmark_data_[run_id])  // each run of this planner
      {
        // Write out properties in the order we listed them above
        for (const std::string& property : properties_set)
        {
          // Make sure this run has this property
          PlannerRunData::const_iterator runit = planner_run_data.find(property);
          if (runit != planner_run_data.end())
            out << runit->second;
          out << "; ";
        }
        out << '\n';  // end of the run
      }
      out << "." << '\n';  // end the planner
    }
  }

  out.close();
  RCLCPP_INFO(LOGGER, "Benchmark results saved to '%s'", filename.c_str());
}
