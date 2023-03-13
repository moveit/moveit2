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

#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit/robot_state/robot_state.h>
#include <geometric_shapes/check_isometry.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace trajectory_execution_manager
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_ros.trajectory_execution_manager");

const std::string TrajectoryExecutionManager::EXECUTION_EVENT_TOPIC = "trajectory_execution_event";

static const auto DEFAULT_CONTROLLER_INFORMATION_VALIDITY_AGE = rclcpp::Duration::from_seconds(1);
static const double DEFAULT_CONTROLLER_GOAL_DURATION_MARGIN = 0.5;  // allow 0.5s more than the expected execution time
                                                                    // before triggering a trajectory cancel (applied
                                                                    // after scaling)
static const double DEFAULT_CONTROLLER_GOAL_DURATION_SCALING =
    1.1;  // allow the execution of a trajectory to take more time than expected (scaled by a value > 1)

TrajectoryExecutionManager::TrajectoryExecutionManager(const rclcpp::Node::SharedPtr& node,
                                                       const moveit::core::RobotModelConstPtr& robot_model,
                                                       const planning_scene_monitor::CurrentStateMonitorPtr& csm)
  : node_(node), robot_model_(robot_model), csm_(csm)
{
  if (!node_->get_parameter("moveit_manage_controllers", manage_controllers_))
    manage_controllers_ = false;
  initialize();
}

TrajectoryExecutionManager::TrajectoryExecutionManager(const rclcpp::Node::SharedPtr& node,
                                                       const moveit::core::RobotModelConstPtr& robot_model,
                                                       const planning_scene_monitor::CurrentStateMonitorPtr& csm,
                                                       bool manage_controllers)
  : node_(node), robot_model_(robot_model), csm_(csm), manage_controllers_(manage_controllers)
{
  initialize();
}

TrajectoryExecutionManager::~TrajectoryExecutionManager()
{
  stopExecution(true);
  if (private_executor_)
    private_executor_->cancel();
  if (private_executor_thread_.joinable())
    private_executor_thread_.join();
}

void TrajectoryExecutionManager::initialize()
{
  verbose_ = false;
  execution_complete_ = true;
  current_context_ = -1;
  last_execution_status_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
  execution_duration_monitoring_ = true;
  execution_velocity_scaling_ = 1.0;
  allowed_start_tolerance_ = 0.01;
  wait_for_trajectory_completion_ = true;

  allowed_execution_duration_scaling_ = DEFAULT_CONTROLLER_GOAL_DURATION_SCALING;
  allowed_goal_duration_margin_ = DEFAULT_CONTROLLER_GOAL_DURATION_MARGIN;

  // load controller-specific values for allowed_execution_duration_scaling and allowed_goal_duration_margin
  loadControllerParams();

  // load the controller manager plugin
  try
  {
    controller_manager_loader_ =
        std::make_unique<pluginlib::ClassLoader<moveit_controller_manager::MoveItControllerManager>>(
            "moveit_core", "moveit_controller_manager::MoveItControllerManager");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_FATAL_STREAM(LOGGER, "Exception while creating controller manager plugin loader: " << ex.what());
    return;
  }

  if (controller_manager_loader_)
  {
    std::string controller;

    if (!node_->get_parameter("moveit_controller_manager", controller))
    {
      const std::vector<std::string>& classes = controller_manager_loader_->getDeclaredClasses();
      if (classes.size() == 1)
      {
        controller = classes[0];
        RCLCPP_WARN(LOGGER,
                    "Parameter '~moveit_controller_manager' is not specified but only one "
                    "matching plugin was found: '%s'. Using that one.",
                    controller.c_str());
      }
      else
      {
        RCLCPP_FATAL(LOGGER, "Parameter '~moveit_controller_manager' not specified. This is needed to "
                             "identify the plugin to use for interacting with controllers. No paths can "
                             "be executed.");
        return;
      }
    }

    // Deprecation warnings, October 2022
    if (controller == "moveit_ros_control_interface/MoveItControllerManager")
    {
      RCLCPP_WARN(LOGGER, "moveit_ros_control_interface/MoveItControllerManager is deprecated. Replace with "
                          "`moveit_ros_control_interface/Ros2ControlManager.`");
    }
    if (controller == "moveit_ros_control_interface/MoveItMultiControllerManager")
    {
      RCLCPP_WARN(LOGGER, "moveit_ros_control_interface/MoveItMultiControllerManager is deprecated. Replace with "
                          "`moveit_ros_control_interface/Ros2ControlMultiManager.`");
    }

    if (!controller.empty())
    {
      try
      {
        // We make a node called moveit_simple_controller_manager so it's able to
        // receive callbacks on another thread. We then copy parameters from the move_group node
        // and then add it to the multithreadedexecutor
        rclcpp::NodeOptions opt;
        opt.allow_undeclared_parameters(true);
        opt.automatically_declare_parameters_from_overrides(true);
        controller_mgr_node_.reset(new rclcpp::Node("moveit_simple_controller_manager", opt));

        auto all_params = node_->get_node_parameters_interface()->get_parameter_overrides();
        for (const auto& param : all_params)
          controller_mgr_node_->set_parameter(rclcpp::Parameter(param.first, param.second));

        controller_manager_ = controller_manager_loader_->createUniqueInstance(controller);
        controller_manager_->initialize(controller_mgr_node_);
        private_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        private_executor_->add_node(controller_mgr_node_);

        // start executor on a different thread now
        private_executor_thread_ = std::thread([this]() { private_executor_->spin(); });
      }
      catch (pluginlib::PluginlibException& ex)
      {
        RCLCPP_FATAL_STREAM(LOGGER, "Exception while loading controller manager '" << controller << "': " << ex.what());
      }
    }
  }

  // other configuration steps
  reloadControllerInformation();
  // The default callback group for rclcpp::Node is MutuallyExclusive which means we cannot call
  // receiveEvent while processing a different callback. To fix this we create a new callback group (the type is not
  // important since we only use it to process one callback) and associate event_topic_subscriber_ with this callback group
  auto callback_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto options = rclcpp::SubscriptionOptions();
  options.callback_group = callback_group;
  event_topic_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
      EXECUTION_EVENT_TOPIC, 100,
      [this](const std_msgs::msg::String::ConstSharedPtr& event) { return receiveEvent(event); }, options);

  controller_mgr_node_->get_parameter("trajectory_execution.execution_duration_monitoring",
                                      execution_duration_monitoring_);
  controller_mgr_node_->get_parameter("trajectory_execution.allowed_execution_duration_scaling",
                                      allowed_execution_duration_scaling_);
  controller_mgr_node_->get_parameter("trajectory_execution.allowed_goal_duration_margin",
                                      allowed_goal_duration_margin_);
  controller_mgr_node_->get_parameter("trajectory_execution.allowed_start_tolerance", allowed_start_tolerance_);

  if (manage_controllers_)
  {
    RCLCPP_INFO(LOGGER, "Trajectory execution is managing controllers");
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Trajectory execution is not managing controllers");
  }

  auto controller_mgr_parameter_set_callback = [this](const std::vector<rclcpp::Parameter>& parameters) {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;
    for (const auto& parameter : parameters)
    {
      const std::string& name = parameter.get_name();
      if (name == "trajectory_execution.execution_duration_monitoring")
      {
        enableExecutionDurationMonitoring(parameter.as_bool());
      }
      else if (name == "trajectory_execution.allowed_execution_duration_scaling")
      {
        setAllowedExecutionDurationScaling(parameter.as_double());
      }
      else if (name == "trajectory_execution.allowed_goal_duration_margin")
      {
        setAllowedGoalDurationMargin(parameter.as_double());
      }
      else if (name == "trajectory_execution.execution_velocity_scaling")
      {
        setExecutionVelocityScaling(parameter.as_double());
      }
      else if (name == "trajectory_execution.allowed_start_tolerance")
      {
        setAllowedStartTolerance(parameter.as_double());
      }
      else if (name == "trajectory_execution.wait_for_trajectory_completion")
      {
        setWaitForTrajectoryCompletion(parameter.as_bool());
      }
      else
      {
        result.successful = false;
      }
    }
    return result;
  };
  callback_handler_ = controller_mgr_node_->add_on_set_parameters_callback(controller_mgr_parameter_set_callback);
}

void TrajectoryExecutionManager::enableExecutionDurationMonitoring(bool flag)
{
  execution_duration_monitoring_ = flag;
}

void TrajectoryExecutionManager::setAllowedExecutionDurationScaling(double scaling)
{
  allowed_execution_duration_scaling_ = scaling;
}

void TrajectoryExecutionManager::setAllowedGoalDurationMargin(double margin)
{
  allowed_goal_duration_margin_ = margin;
}

void TrajectoryExecutionManager::setExecutionVelocityScaling(double scaling)
{
  execution_velocity_scaling_ = scaling;
}

void TrajectoryExecutionManager::setAllowedStartTolerance(double tolerance)
{
  allowed_start_tolerance_ = tolerance;
}

void TrajectoryExecutionManager::setWaitForTrajectoryCompletion(bool flag)
{
  wait_for_trajectory_completion_ = flag;
}

bool TrajectoryExecutionManager::isManagingControllers() const
{
  return manage_controllers_;
}

const moveit_controller_manager::MoveItControllerManagerPtr& TrajectoryExecutionManager::getControllerManager() const
{
  return controller_manager_;
}

void TrajectoryExecutionManager::processEvent(const std::string& event)
{
  if (event == "stop")
  {
    stopExecution(true);
  }
  else
  {
    RCLCPP_WARN_STREAM(LOGGER, "Unknown event type: '" << event << '\'');
  }
}

void TrajectoryExecutionManager::receiveEvent(const std_msgs::msg::String::ConstSharedPtr& event)
{
  RCLCPP_INFO_STREAM(LOGGER, "Received event '" << event->data << '\'');
  processEvent(event->data);
}

bool TrajectoryExecutionManager::push(const moveit_msgs::msg::RobotTrajectory& trajectory, const std::string& controller)
{
  if (controller.empty())
  {
    return push(trajectory, std::vector<std::string>());
  }
  else
  {
    return push(trajectory, std::vector<std::string>(1, controller));
  }
}

bool TrajectoryExecutionManager::push(const trajectory_msgs::msg::JointTrajectory& trajectory,
                                      const std::string& controller)
{
  if (controller.empty())
  {
    return push(trajectory, std::vector<std::string>());
  }
  else
  {
    return push(trajectory, std::vector<std::string>(1, controller));
  }
}

bool TrajectoryExecutionManager::push(const trajectory_msgs::msg::JointTrajectory& trajectory,
                                      const std::vector<std::string>& controllers)
{
  moveit_msgs::msg::RobotTrajectory traj;
  traj.joint_trajectory = trajectory;
  return push(traj, controllers);
}

bool TrajectoryExecutionManager::push(const moveit_msgs::msg::RobotTrajectory& trajectory,
                                      const std::vector<std::string>& controllers)
{
  if (!execution_complete_)
  {
    RCLCPP_ERROR(LOGGER, "Cannot push a new trajectory while another is being executed");
    return false;
  }

  TrajectoryExecutionContext* context = new TrajectoryExecutionContext();
  if (configure(*context, trajectory, controllers))
  {
    if (verbose_)
    {
      std::stringstream ss;
      ss << "Pushed trajectory for execution using controllers [ ";
      for (const std::string& controller : context->controllers_)
        ss << controller << ' ';
      ss << "]:" << '\n';
      // TODO: Provide message serialization
      // for (const moveit_msgs::msg::RobotTrajectory& trajectory_part : context->trajectory_parts_)
      // ss << trajectory_part << '\n';
      RCLCPP_INFO_STREAM(LOGGER, ss.str());
    }
    trajectories_.push_back(context);
    return true;
  }
  else
  {
    delete context;
    last_execution_status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
  }

  return false;
}

void TrajectoryExecutionManager::reloadControllerInformation()
{
  known_controllers_.clear();
  if (controller_manager_)
  {
    std::vector<std::string> names;
    controller_manager_->getControllersList(names);
    for (const std::string& name : names)
    {
      std::vector<std::string> joints;
      controller_manager_->getControllerJoints(name, joints);
      ControllerInformation ci;
      ci.name_ = name;
      ci.joints_.insert(joints.begin(), joints.end());
      known_controllers_[ci.name_] = ci;
    }

    names.clear();
    controller_manager_->getActiveControllers(names);
    for (const auto& active_name : names)
    {
      auto found_it = std::find_if(known_controllers_.begin(), known_controllers_.end(),
                                   [&](const auto& known_controller) { return known_controller.first == active_name; });
      if (found_it != known_controllers_.end())
      {
        found_it->second.state_.active_ = true;
      }
    }

    for (std::map<std::string, ControllerInformation>::iterator it = known_controllers_.begin();
         it != known_controllers_.end(); ++it)
    {
      for (std::map<std::string, ControllerInformation>::iterator jt = known_controllers_.begin();
           jt != known_controllers_.end(); ++jt)
      {
        if (it != jt)
        {
          std::vector<std::string> intersect;
          std::set_intersection(it->second.joints_.begin(), it->second.joints_.end(), jt->second.joints_.begin(),
                                jt->second.joints_.end(), std::back_inserter(intersect));
          if (!intersect.empty())
          {
            it->second.overlapping_controllers_.insert(jt->first);
            jt->second.overlapping_controllers_.insert(it->first);
          }
        }
      }
    }
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Failed to reload controllers: `controller_manager_` does not exist.");
  }
}

void TrajectoryExecutionManager::updateControllerState(const std::string& controller, const rclcpp::Duration& age)
{
  std::map<std::string, ControllerInformation>::iterator it = known_controllers_.find(controller);
  if (it != known_controllers_.end())
  {
    updateControllerState(it->second, age);
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Controller '%s' is not known.", controller.c_str());
  }
}

void TrajectoryExecutionManager::updateControllerState(ControllerInformation& ci, const rclcpp::Duration& age)
{
  if (node_->now() - ci.last_update_ >= age)
  {
    if (controller_manager_)
    {
      if (verbose_)
        RCLCPP_INFO(LOGGER, "Updating information for controller '%s'.", ci.name_.c_str());
      ci.state_ = controller_manager_->getControllerState(ci.name_);
      ci.last_update_ = node_->now();
    }
  }
  else if (verbose_)
    RCLCPP_INFO(LOGGER, "Information for controller '%s' is assumed to be up to date.", ci.name_.c_str());
}

void TrajectoryExecutionManager::updateControllersState(const rclcpp::Duration& age)
{
  for (std::pair<const std::string, ControllerInformation>& known_controller : known_controllers_)
    updateControllerState(known_controller.second, age);
}

bool TrajectoryExecutionManager::checkControllerCombination(std::vector<std::string>& selected,
                                                            const std::set<std::string>& actuated_joints)
{
  std::set<std::string> combined_joints;
  for (const std::string& controller : selected)
  {
    const ControllerInformation& ci = known_controllers_[controller];
    combined_joints.insert(ci.joints_.begin(), ci.joints_.end());
  }

  if (verbose_)
  {
    std::stringstream ss, saj, sac;
    for (const std::string& controller : selected)
      ss << controller << ' ';
    for (const std::string& actuated_joint : actuated_joints)
      saj << actuated_joint << ' ';
    for (const std::string& combined_joint : combined_joints)
      sac << combined_joint << ' ';
    RCLCPP_INFO(LOGGER, "Checking if controllers [ %s] operating on joints [ %s] cover joints [ %s]", ss.str().c_str(),
                sac.str().c_str(), saj.str().c_str());
  }

  return std::includes(combined_joints.begin(), combined_joints.end(), actuated_joints.begin(), actuated_joints.end());
}

void TrajectoryExecutionManager::generateControllerCombination(std::size_t start_index, std::size_t controller_count,
                                                               const std::vector<std::string>& available_controllers,
                                                               std::vector<std::string>& selected_controllers,
                                                               std::vector<std::vector<std::string>>& selected_options,
                                                               const std::set<std::string>& actuated_joints)
{
  if (selected_controllers.size() == controller_count)
  {
    if (checkControllerCombination(selected_controllers, actuated_joints))
      selected_options.push_back(selected_controllers);
    return;
  }

  for (std::size_t i = start_index; i < available_controllers.size(); ++i)
  {
    bool overlap = false;
    const ControllerInformation& ci = known_controllers_[available_controllers[i]];
    for (std::size_t j = 0; j < selected_controllers.size() && !overlap; ++j)
    {
      if (ci.overlapping_controllers_.find(selected_controllers[j]) != ci.overlapping_controllers_.end())
        overlap = true;
    }
    if (overlap)
      continue;
    selected_controllers.push_back(available_controllers[i]);
    generateControllerCombination(i + 1, controller_count, available_controllers, selected_controllers,
                                  selected_options, actuated_joints);
    selected_controllers.pop_back();
  }
}

namespace
{
struct OrderPotentialControllerCombination
{
  bool operator()(const std::size_t a, const std::size_t b) const
  {
    // preference is given to controllers marked as default
    if (nrdefault[a] > nrdefault[b])
      return true;
    if (nrdefault[a] < nrdefault[b])
      return false;

    // and then to ones that operate on fewer joints
    if (nrjoints[a] < nrjoints[b])
      return true;
    if (nrjoints[a] > nrjoints[b])
      return false;

    // and then to active ones
    if (nractive[a] < nractive[b])
      return true;
    if (nractive[a] > nractive[b])
      return false;

    return false;
  }

  std::vector<std::vector<std::string>> selected_options;
  std::vector<std::size_t> nrdefault;
  std::vector<std::size_t> nrjoints;
  std::vector<std::size_t> nractive;
};
}  // namespace

bool TrajectoryExecutionManager::findControllers(const std::set<std::string>& actuated_joints,
                                                 std::size_t controller_count,
                                                 const std::vector<std::string>& available_controllers,
                                                 std::vector<std::string>& selected_controllers)
{
  // generate all combinations of controller_count controllers that operate on disjoint sets of joints
  std::vector<std::string> work_area;
  OrderPotentialControllerCombination order;
  std::vector<std::vector<std::string>>& selected_options = order.selected_options;
  generateControllerCombination(0, controller_count, available_controllers, work_area, selected_options,
                                actuated_joints);

  if (verbose_)
  {
    std::stringstream saj;
    std::stringstream sac;
    for (const std::string& available_controller : available_controllers)
      sac << available_controller << ' ';
    for (const std::string& actuated_joint : actuated_joints)
      saj << actuated_joint << ' ';
    RCLCPP_INFO(LOGGER, "Looking for %zu controllers among [ %s] that cover joints [ %s]. Found %zd options.",
                controller_count, sac.str().c_str(), saj.str().c_str(), selected_options.size());
  }

  // if none was found, this is a problem
  if (selected_options.empty())
    return false;

  // if only one was found, return it
  if (selected_options.size() == 1)
  {
    selected_controllers.swap(selected_options[0]);
    return true;
  }

  // if more options were found, evaluate them all and return the best one

  // count how many default controllers are used in each reported option, and how many joints are actuated in total by
  // the selected controllers,
  // to use that in the ranking of the options
  order.nrdefault.resize(selected_options.size(), 0);
  order.nrjoints.resize(selected_options.size(), 0);
  order.nractive.resize(selected_options.size(), 0);
  for (std::size_t i = 0; i < selected_options.size(); ++i)
  {
    for (std::size_t k = 0; k < selected_options[i].size(); ++k)
    {
      updateControllerState(selected_options[i][k], DEFAULT_CONTROLLER_INFORMATION_VALIDITY_AGE);
      const ControllerInformation& ci = known_controllers_[selected_options[i][k]];

      if (ci.state_.default_)
        order.nrdefault[i]++;
      if (ci.state_.active_)
        order.nractive[i]++;
      order.nrjoints[i] += ci.joints_.size();
    }
  }

  // define a bijection to compute the raking of the found options
  std::vector<std::size_t> bijection(selected_options.size(), 0);
  for (std::size_t i = 0; i < selected_options.size(); ++i)
    bijection[i] = i;

  // sort the options
  std::sort(bijection.begin(), bijection.end(), order);

  // depending on whether we are allowed to load & unload controllers,
  // we have different preference on deciding between options
  if (!manage_controllers_)
  {
    // if we can't load different options at will, just choose one that is already loaded
    for (std::size_t i = 0; i < selected_options.size(); ++i)
    {
      if (areControllersActive(selected_options[bijection[i]]))
      {
        selected_controllers.swap(selected_options[bijection[i]]);
        return true;
      }
    }
  }

  // otherwise, just use the first valid option
  selected_controllers.swap(selected_options[bijection[0]]);
  return true;
}

bool TrajectoryExecutionManager::isControllerActive(const std::string& controller)
{
  return areControllersActive(std::vector<std::string>(1, controller));
}

bool TrajectoryExecutionManager::areControllersActive(const std::vector<std::string>& controllers)
{
  for (const std::string& controller : controllers)
  {
    updateControllerState(controller, DEFAULT_CONTROLLER_INFORMATION_VALIDITY_AGE);
    std::map<std::string, ControllerInformation>::iterator it = known_controllers_.find(controller);
    if (it == known_controllers_.end() || !it->second.state_.active_)
      return false;
  }
  return true;
}

bool TrajectoryExecutionManager::selectControllers(const std::set<std::string>& actuated_joints,
                                                   const std::vector<std::string>& available_controllers,
                                                   std::vector<std::string>& selected_controllers)
{
  for (std::size_t i = 1; i <= available_controllers.size(); ++i)
  {
    if (findControllers(actuated_joints, i, available_controllers, selected_controllers))
    {
      // if we are not managing controllers, prefer to use active controllers even if there are more of them
      if (!manage_controllers_ && !areControllersActive(selected_controllers))
      {
        std::vector<std::string> other_option;
        for (std::size_t j = i + 1; j <= available_controllers.size(); ++j)
        {
          if (findControllers(actuated_joints, j, available_controllers, other_option))
          {
            if (areControllersActive(other_option))
            {
              selected_controllers = other_option;
              break;
            }
          }
        }
      }
      return true;
    }
  }
  return false;
}

bool TrajectoryExecutionManager::distributeTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory,
                                                      const std::vector<std::string>& controllers,
                                                      std::vector<moveit_msgs::msg::RobotTrajectory>& parts)
{
  parts.clear();
  parts.resize(controllers.size());

  std::set<std::string> actuated_joints_mdof;
  actuated_joints_mdof.insert(trajectory.multi_dof_joint_trajectory.joint_names.begin(),
                              trajectory.multi_dof_joint_trajectory.joint_names.end());
  std::set<std::string> actuated_joints_single;
  for (const std::string& joint_name : trajectory.joint_trajectory.joint_names)
  {
    const moveit::core::JointModel* jm = robot_model_->getJointModel(joint_name);
    if (jm)
    {
      if (jm->isPassive() || jm->getMimic() != nullptr || jm->getType() == moveit::core::JointModel::FIXED)
        continue;
      actuated_joints_single.insert(jm->getName());
    }
  }

  for (std::size_t i = 0; i < controllers.size(); ++i)
  {
    std::map<std::string, ControllerInformation>::iterator it = known_controllers_.find(controllers[i]);
    if (it == known_controllers_.end())
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Controller " << controllers[i] << " not found.");
      return false;
    }
    std::vector<std::string> intersect_mdof;
    std::set_intersection(it->second.joints_.begin(), it->second.joints_.end(), actuated_joints_mdof.begin(),
                          actuated_joints_mdof.end(), std::back_inserter(intersect_mdof));
    std::vector<std::string> intersect_single;
    std::set_intersection(it->second.joints_.begin(), it->second.joints_.end(), actuated_joints_single.begin(),
                          actuated_joints_single.end(), std::back_inserter(intersect_single));
    if (intersect_mdof.empty() && intersect_single.empty())
      RCLCPP_WARN_STREAM(LOGGER, "No joints to be distributed for controller " << controllers[i]);
    {
      if (!intersect_mdof.empty())
      {
        std::vector<std::string>& jnames = parts[i].multi_dof_joint_trajectory.joint_names;
        jnames.insert(jnames.end(), intersect_mdof.begin(), intersect_mdof.end());
        std::map<std::string, std::size_t> index;
        for (std::size_t j = 0; j < trajectory.multi_dof_joint_trajectory.joint_names.size(); ++j)
          index[trajectory.multi_dof_joint_trajectory.joint_names[j]] = j;
        std::vector<std::size_t> bijection(jnames.size());
        for (std::size_t j = 0; j < jnames.size(); ++j)
          bijection[j] = index[jnames[j]];

        parts[i].multi_dof_joint_trajectory.header.frame_id = trajectory.multi_dof_joint_trajectory.header.frame_id;
        parts[i].multi_dof_joint_trajectory.points.resize(trajectory.multi_dof_joint_trajectory.points.size());
        for (std::size_t j = 0; j < trajectory.multi_dof_joint_trajectory.points.size(); ++j)
        {
          parts[i].multi_dof_joint_trajectory.points[j].time_from_start =
              trajectory.multi_dof_joint_trajectory.points[j].time_from_start;
          parts[i].multi_dof_joint_trajectory.points[j].transforms.resize(bijection.size());
          for (std::size_t k = 0; k < bijection.size(); ++k)
          {
            parts[i].multi_dof_joint_trajectory.points[j].transforms[k] =
                trajectory.multi_dof_joint_trajectory.points[j].transforms[bijection[k]];

            if (!trajectory.multi_dof_joint_trajectory.points[j].velocities.empty())
            {
              parts[i].multi_dof_joint_trajectory.points[j].velocities.resize(bijection.size());

              parts[i].multi_dof_joint_trajectory.points[j].velocities[0].linear.x =
                  trajectory.multi_dof_joint_trajectory.points[j].velocities[0].linear.x * execution_velocity_scaling_;

              parts[i].multi_dof_joint_trajectory.points[j].velocities[0].linear.y =
                  trajectory.multi_dof_joint_trajectory.points[j].velocities[0].linear.y * execution_velocity_scaling_;

              parts[i].multi_dof_joint_trajectory.points[j].velocities[0].linear.z =
                  trajectory.multi_dof_joint_trajectory.points[j].velocities[0].linear.z * execution_velocity_scaling_;

              parts[i].multi_dof_joint_trajectory.points[j].velocities[0].angular.x =
                  trajectory.multi_dof_joint_trajectory.points[j].velocities[0].angular.x * execution_velocity_scaling_;

              parts[i].multi_dof_joint_trajectory.points[j].velocities[0].angular.y =
                  trajectory.multi_dof_joint_trajectory.points[j].velocities[0].angular.y * execution_velocity_scaling_;

              parts[i].multi_dof_joint_trajectory.points[j].velocities[0].angular.z =
                  trajectory.multi_dof_joint_trajectory.points[j].velocities[0].angular.z * execution_velocity_scaling_;
            }
          }
        }
      }
      if (!intersect_single.empty())
      {
        std::vector<std::string>& jnames = parts[i].joint_trajectory.joint_names;
        jnames.insert(jnames.end(), intersect_single.begin(), intersect_single.end());
        parts[i].joint_trajectory.header = trajectory.joint_trajectory.header;
        std::map<std::string, std::size_t> index;
        for (std::size_t j = 0; j < trajectory.joint_trajectory.joint_names.size(); ++j)
          index[trajectory.joint_trajectory.joint_names[j]] = j;
        std::vector<std::size_t> bijection(jnames.size());
        for (std::size_t j = 0; j < jnames.size(); ++j)
          bijection[j] = index[jnames[j]];
        parts[i].joint_trajectory.points.resize(trajectory.joint_trajectory.points.size());
        for (std::size_t j = 0; j < trajectory.joint_trajectory.points.size(); ++j)
        {
          parts[i].joint_trajectory.points[j].time_from_start = trajectory.joint_trajectory.points[j].time_from_start;
          if (!trajectory.joint_trajectory.points[j].positions.empty())
          {
            parts[i].joint_trajectory.points[j].positions.resize(bijection.size());
            for (std::size_t k = 0; k < bijection.size(); ++k)
            {
              parts[i].joint_trajectory.points[j].positions[k] =
                  trajectory.joint_trajectory.points[j].positions[bijection[k]];
            }
          }
          if (!trajectory.joint_trajectory.points[j].velocities.empty())
          {
            parts[i].joint_trajectory.points[j].velocities.resize(bijection.size());
            for (std::size_t k = 0; k < bijection.size(); ++k)
            {
              parts[i].joint_trajectory.points[j].velocities[k] =
                  trajectory.joint_trajectory.points[j].velocities[bijection[k]] * execution_velocity_scaling_;
            }
          }
          if (!trajectory.joint_trajectory.points[j].accelerations.empty())
          {
            parts[i].joint_trajectory.points[j].accelerations.resize(bijection.size());
            for (std::size_t k = 0; k < bijection.size(); ++k)
            {
              parts[i].joint_trajectory.points[j].accelerations[k] =
                  trajectory.joint_trajectory.points[j].accelerations[bijection[k]];
            }
          }
          if (!trajectory.joint_trajectory.points[j].effort.empty())
          {
            parts[i].joint_trajectory.points[j].effort.resize(bijection.size());
            for (std::size_t k = 0; k < bijection.size(); ++k)
            {
              parts[i].joint_trajectory.points[j].effort[k] =
                  trajectory.joint_trajectory.points[j].effort[bijection[k]];
            }
          }
        }
      }
    }
  }
  return true;
}

bool TrajectoryExecutionManager::validate(const TrajectoryExecutionContext& context) const
{
  if (allowed_start_tolerance_ == 0)  // skip validation on this magic number
    return true;

  RCLCPP_INFO(LOGGER, "Validating trajectory with allowed_start_tolerance %g", allowed_start_tolerance_);

  moveit::core::RobotStatePtr current_state;
  if (!csm_->waitForCurrentState(node_->now()) || !(current_state = csm_->getCurrentState()))
  {
    RCLCPP_WARN(LOGGER, "Failed to validate trajectory: couldn't receive full current joint state within 1s");
    return false;
  }

  for (const auto& trajectory : context.trajectory_parts_)
  {
    if (!trajectory.joint_trajectory.points.empty())
    {
      // Check single-dof trajectory
      const std::vector<double>& positions = trajectory.joint_trajectory.points.front().positions;
      const std::vector<std::string>& joint_names = trajectory.joint_trajectory.joint_names;
      if (positions.size() != joint_names.size())
      {
        RCLCPP_ERROR(LOGGER, "Wrong trajectory: #joints: %zu != #positions: %zu", joint_names.size(), positions.size());
        return false;
      }

      for (std::size_t i = 0, end = joint_names.size(); i < end; ++i)
      {
        const moveit::core::JointModel* jm = current_state->getJointModel(joint_names[i]);
        if (!jm)
        {
          RCLCPP_ERROR_STREAM(LOGGER, "Unknown joint in trajectory: " << joint_names[i]);
          return false;
        }

        double cur_position = current_state->getJointPositions(jm)[0];
        double traj_position = positions[i];
        // normalize positions and compare
        jm->enforcePositionBounds(&cur_position);
        jm->enforcePositionBounds(&traj_position);
        if (jm->distance(&cur_position, &traj_position) > allowed_start_tolerance_)
        {
          RCLCPP_ERROR(LOGGER,
                       "\nInvalid Trajectory: start point deviates from current robot state more than %g"
                       "\njoint '%s': expected: %g, current: %g",
                       allowed_start_tolerance_, joint_names[i].c_str(), traj_position, cur_position);
          return false;
        }
      }
    }
    if (!trajectory.multi_dof_joint_trajectory.points.empty())
    {
      // Check multi-dof trajectory
      const std::vector<geometry_msgs::msg::Transform>& transforms =
          trajectory.multi_dof_joint_trajectory.points.front().transforms;
      const std::vector<std::string>& joint_names = trajectory.multi_dof_joint_trajectory.joint_names;
      if (transforms.size() != joint_names.size())
      {
        RCLCPP_ERROR(LOGGER, "Wrong trajectory: #joints: %zu != #transforms: %zu", joint_names.size(),
                     transforms.size());
        return false;
      }

      for (std::size_t i = 0, end = joint_names.size(); i < end; ++i)
      {
        const moveit::core::JointModel* jm = current_state->getJointModel(joint_names[i]);
        if (!jm)
        {
          RCLCPP_ERROR_STREAM(LOGGER, "Unknown joint in trajectory: " << joint_names[i]);
          return false;
        }

        // compute difference (offset vector and rotation angle) between current transform
        // and start transform in trajectory
        Eigen::Isometry3d cur_transform, start_transform;
        // computeTransform() computes a valid isometry by contract
        jm->computeTransform(current_state->getJointPositions(jm), cur_transform);
        start_transform = tf2::transformToEigen(transforms[i]);
        ASSERT_ISOMETRY(start_transform)  // unsanitized input, could contain a non-isometry
        Eigen::Vector3d offset = cur_transform.translation() - start_transform.translation();
        Eigen::AngleAxisd rotation;
        rotation.fromRotationMatrix(cur_transform.linear().transpose() * start_transform.linear());
        if ((offset.array() > allowed_start_tolerance_).any() || rotation.angle() > allowed_start_tolerance_)
        {
          RCLCPP_ERROR_STREAM(LOGGER, "\nInvalid Trajectory: start point deviates from current robot state more than "
                                          << allowed_start_tolerance_ << "\nmulti-dof joint '" << joint_names[i]
                                          << "': pos delta: " << offset.transpose()
                                          << " rot delta: " << rotation.angle());
          return false;
        }
      }
    }
  }
  return true;
}

bool TrajectoryExecutionManager::configure(TrajectoryExecutionContext& context,
                                           const moveit_msgs::msg::RobotTrajectory& trajectory,
                                           const std::vector<std::string>& controllers)
{
  if (trajectory.multi_dof_joint_trajectory.points.empty() && trajectory.joint_trajectory.points.empty())
  {
    // empty trajectories don't need to configure anything
    return true;
  }

  reloadControllerInformation();
  std::set<std::string> actuated_joints;

  auto is_actuated = [this](const std::string& joint_name) -> bool {
    const moveit::core::JointModel* jm = robot_model_->getJointModel(joint_name);
    return (jm && !jm->isPassive() && !jm->getMimic() && jm->getType() != moveit::core::JointModel::FIXED);
  };
  for (const std::string& joint_name : trajectory.multi_dof_joint_trajectory.joint_names)
  {
    if (is_actuated(joint_name))
      actuated_joints.insert(joint_name);
  }
  for (const std::string& joint_name : trajectory.joint_trajectory.joint_names)
  {
    if (is_actuated(joint_name))
      actuated_joints.insert(joint_name);
  }

  if (actuated_joints.empty())
  {
    RCLCPP_WARN(LOGGER, "The trajectory to execute specifies no joints");
    return false;
  }

  if (controllers.empty())
  {
    bool retry = true;
    bool reloaded = false;
    while (retry)
    {
      retry = false;
      std::vector<std::string> all_controller_names;
      for (std::map<std::string, ControllerInformation>::const_iterator it = known_controllers_.begin();
           it != known_controllers_.end(); ++it)
        all_controller_names.push_back(it->first);
      if (selectControllers(actuated_joints, all_controller_names, context.controllers_))
      {
        if (distributeTrajectory(trajectory, context.controllers_, context.trajectory_parts_))
          return true;
      }
      else
      {
        // maybe we failed because we did not have a complete list of controllers
        if (!reloaded)
        {
          reloadControllerInformation();
          reloaded = true;
          retry = true;
        }
      }
    }
  }
  else
  {
    // check if the specified controllers are valid names;
    // if they appear not to be, try to reload the controller information, just in case they are new in the system
    bool reloaded = false;
    for (const std::string& controller : controllers)
    {
      if (known_controllers_.find(controller) == known_controllers_.end())
      {
        reloadControllerInformation();
        reloaded = true;
        break;
      }
    }
    if (reloaded)
    {
      for (const std::string& controller : controllers)
      {
        if (known_controllers_.find(controller) == known_controllers_.end())
        {
          RCLCPP_ERROR(LOGGER, "Controller '%s' is not known", controller.c_str());
          return false;
        }
      }
    }
    if (selectControllers(actuated_joints, controllers, context.controllers_))
    {
      if (distributeTrajectory(trajectory, context.controllers_, context.trajectory_parts_))
        return true;
    }
  }
  std::stringstream ss;
  for (const std::string& actuated_joint : actuated_joints)
    ss << actuated_joint << ' ';
  RCLCPP_ERROR(LOGGER, "Unable to identify any set of controllers that can actuate the specified joints: [ %s]",
               ss.str().c_str());

  std::stringstream ss2;
  std::map<std::string, ControllerInformation>::const_iterator mi;
  for (mi = known_controllers_.begin(); mi != known_controllers_.end(); ++mi)
  {
    ss2 << "controller '" << mi->second.name_ << "' controls joints:\n";

    std::set<std::string>::const_iterator ji;
    for (ji = mi->second.joints_.begin(); ji != mi->second.joints_.end(); ++ji)
    {
      ss2 << "  " << *ji << '\n';
    }
  }
  RCLCPP_ERROR(LOGGER, "Known controllers and their joints:\n%s", ss2.str().c_str());
  return false;
}

moveit_controller_manager::ExecutionStatus TrajectoryExecutionManager::executeAndWait(bool auto_clear)
{
  execute(ExecutionCompleteCallback(), auto_clear);
  return waitForExecution();
}

void TrajectoryExecutionManager::stopExecutionInternal()
{
  // execution_state_mutex_ needs to have been locked by the caller
  for (moveit_controller_manager::MoveItControllerHandlePtr& active_handle : active_handles_)
  {
    try
    {
      active_handle->cancelExecution();
    }
    catch (std::exception& ex)
    {
      RCLCPP_ERROR(LOGGER, "Caught %s when canceling execution.", ex.what());
    }
  }
}

void TrajectoryExecutionManager::stopExecution(bool auto_clear)
{
  if (!execution_complete_)
  {
    execution_state_mutex_.lock();
    if (!execution_complete_)
    {
      // we call cancel for all active handles; we know these are not being modified as we loop through them because of
      // the lock
      // we mark execution_complete_ as true ahead of time. Using this flag, executePart() will know that an external
      // trigger to stop has been received
      execution_complete_ = true;
      stopExecutionInternal();

      // we set the status here; executePart() will not set status when execution_complete_ is true ahead of time
      last_execution_status_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
      execution_state_mutex_.unlock();
      RCLCPP_INFO(LOGGER, "Stopped trajectory execution.");

      // wait for the execution thread to finish
      std::scoped_lock lock(execution_thread_mutex_);
      if (execution_thread_)
      {
        execution_thread_->join();
        execution_thread_.reset();
      }

      if (auto_clear)
        clear();
    }
    else
      execution_state_mutex_.unlock();
  }
  else if (execution_thread_)  // just in case we have some thread waiting to be joined from some point in the past, we
                               // join it now
  {
    std::scoped_lock lock(execution_thread_mutex_);
    if (execution_thread_)
    {
      execution_thread_->join();
      execution_thread_.reset();
    }
  }
}

void TrajectoryExecutionManager::execute(const ExecutionCompleteCallback& callback, bool auto_clear)
{
  execute(callback, PathSegmentCompleteCallback(), auto_clear);
}

void TrajectoryExecutionManager::execute(const ExecutionCompleteCallback& callback,
                                         const PathSegmentCompleteCallback& part_callback, bool auto_clear)
{
  // skip execution if no trajectory have been pushed
  // it crashes otherwise
  if (trajectories_.empty())
    return;

  stopExecution(false);

  // check whether first trajectory starts at current robot state
  if (!trajectories_.empty() && !validate(*trajectories_.front()))
  {
    last_execution_status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
    if (auto_clear)
      clear();
    if (callback)
      callback(last_execution_status_);
    return;
  }

  // start the execution thread
  execution_complete_ = false;
  execution_thread_ = std::make_unique<std::thread>(&TrajectoryExecutionManager::executeThread, this, callback,
                                                    part_callback, auto_clear);
}

moveit_controller_manager::ExecutionStatus TrajectoryExecutionManager::waitForExecution()
{
  {
    std::unique_lock<std::mutex> ulock(execution_state_mutex_);
    while (!execution_complete_)
      execution_complete_condition_.wait(ulock);
  }

  // this will join the thread for executing sequences of trajectories
  stopExecution(false);

  return last_execution_status_;
}

void TrajectoryExecutionManager::clear()
{
  if (execution_complete_)
  {
    for (TrajectoryExecutionContext* trajectory : trajectories_)
      delete trajectory;
    trajectories_.clear();
  }
  else
    RCLCPP_ERROR(LOGGER, "Cannot push a new trajectory while another is being executed");
}

void TrajectoryExecutionManager::executeThread(const ExecutionCompleteCallback& callback,
                                               const PathSegmentCompleteCallback& part_callback, bool auto_clear)
{
  // if we already got a stop request before we even started anything, we abort
  if (execution_complete_)
  {
    last_execution_status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
    if (callback)
      callback(last_execution_status_);
    return;
  }

  RCLCPP_INFO(LOGGER, "Starting trajectory execution ...");
  // assume everything will be OK
  last_execution_status_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;

  // execute each trajectory, one after the other (executePart() is blocking) or until one fails.
  // on failure, the status is set by executePart(). Otherwise, it will remain as set above (success)
  std::size_t i = 0;
  for (; i < trajectories_.size(); ++i)
  {
    bool epart = executePart(i);
    if (epart && part_callback)
      part_callback(i);
    if (!epart || execution_complete_)
    {
      ++i;
      break;
    }
  }

  // only report that execution finished successfully when the robot actually stopped moving
  if (last_execution_status_ == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
    waitForRobotToStop(*trajectories_[i - 1]);

  RCLCPP_INFO(LOGGER, "Completed trajectory execution with status %s ...", last_execution_status_.asString().c_str());

  // notify whoever is waiting for the event of trajectory completion
  execution_state_mutex_.lock();
  execution_complete_ = true;
  execution_state_mutex_.unlock();
  execution_complete_condition_.notify_all();

  // clear the paths just executed, if needed
  if (auto_clear)
    clear();

  // call user-specified callback
  if (callback)
    callback(last_execution_status_);
}

bool TrajectoryExecutionManager::executePart(std::size_t part_index)
{
  TrajectoryExecutionContext& context = *trajectories_[part_index];

  // first make sure desired controllers are active
  if (ensureActiveControllers(context.controllers_))
  {
    // stop if we are already asked to do so
    if (execution_complete_)
      return false;

    std::vector<moveit_controller_manager::MoveItControllerHandlePtr> handles;
    {
      std::scoped_lock slock(execution_state_mutex_);
      if (!execution_complete_)
      {
        // time indexing uses this member too, so we lock this mutex as well
        time_index_mutex_.lock();
        current_context_ = part_index;
        time_index_mutex_.unlock();
        active_handles_.resize(context.controllers_.size());
        for (std::size_t i = 0; i < context.controllers_.size(); ++i)
        {
          moveit_controller_manager::MoveItControllerHandlePtr h;
          try
          {
            h = controller_manager_->getControllerHandle(context.controllers_[i]);
          }
          catch (std::exception& ex)
          {
            RCLCPP_ERROR(LOGGER, "Caught %s when retrieving controller handle", ex.what());
          }
          if (!h)
          {
            active_handles_.clear();
            current_context_ = -1;
            last_execution_status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
            RCLCPP_ERROR(LOGGER, "No controller handle for controller '%s'. Aborting.", context.controllers_[i].c_str());
            return false;
          }
          active_handles_[i] = h;
        }
        handles = active_handles_;  // keep a copy for later, to avoid thread safety issues
        for (std::size_t i = 0; i < context.trajectory_parts_.size(); ++i)
        {
          bool ok = false;
          try
          {
            ok = active_handles_[i]->sendTrajectory(context.trajectory_parts_[i]);
          }
          catch (std::exception& ex)
          {
            RCLCPP_ERROR(LOGGER, "Caught %s when sending trajectory to controller", ex.what());
          }
          if (!ok)
          {
            for (std::size_t j = 0; j < i; ++j)
            {
              try
              {
                active_handles_[j]->cancelExecution();
              }
              catch (std::exception& ex)
              {
                RCLCPP_ERROR(LOGGER, "Caught %s when canceling execution", ex.what());
              }
            }
            RCLCPP_ERROR(LOGGER, "Failed to send trajectory part %zu of %zu to controller %s", i + 1,
                         context.trajectory_parts_.size(), active_handles_[i]->getName().c_str());
            if (i > 0)
              RCLCPP_ERROR(LOGGER, "Cancelling previously sent trajectory parts");
            active_handles_.clear();
            current_context_ = -1;
            last_execution_status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
            return false;
          }
        }
      }
    }

    // compute the expected duration of the trajectory and find the part of the trajectory that takes longest to execute
    rclcpp::Time current_time = node_->now();
    auto expected_trajectory_duration = rclcpp::Duration::from_seconds(0);
    int longest_part = -1;
    for (std::size_t i = 0; i < context.trajectory_parts_.size(); ++i)
    {
      auto d = rclcpp::Duration::from_seconds(0);
      if (!(context.trajectory_parts_[i].joint_trajectory.points.empty() &&
            context.trajectory_parts_[i].multi_dof_joint_trajectory.points.empty()))
      {
        if (rclcpp::Time(context.trajectory_parts_[i].joint_trajectory.header.stamp) > current_time)
          d = rclcpp::Time(context.trajectory_parts_[i].joint_trajectory.header.stamp) - current_time;
        if (rclcpp::Time(context.trajectory_parts_[i].multi_dof_joint_trajectory.header.stamp) > current_time)
        {
          d = std::max(d, rclcpp::Time(context.trajectory_parts_[i].multi_dof_joint_trajectory.header.stamp) -
                              current_time);
        }
        d = d +
            std::max(context.trajectory_parts_[i].joint_trajectory.points.empty() ?
                         rclcpp::Duration::from_seconds(0) :
                         rclcpp::Duration(context.trajectory_parts_[i].joint_trajectory.points.back().time_from_start),
                     context.trajectory_parts_[i].multi_dof_joint_trajectory.points.empty() ?
                         rclcpp::Duration::from_seconds(0) :
                         rclcpp::Duration(
                             context.trajectory_parts_[i].multi_dof_joint_trajectory.points.back().time_from_start));

        if (longest_part < 0 ||
            std::max(context.trajectory_parts_[i].joint_trajectory.points.size(),
                     context.trajectory_parts_[i].multi_dof_joint_trajectory.points.size()) >
                std::max(context.trajectory_parts_[longest_part].joint_trajectory.points.size(),
                         context.trajectory_parts_[longest_part].multi_dof_joint_trajectory.points.size()))
          longest_part = i;
      }

      // prefer controller-specific values over global ones if defined
      // TODO: the controller-specific parameters are static, but override
      //       the global ones are configurable via dynamic reconfigure
      std::map<std::string, double>::const_iterator scaling_it =
          controller_allowed_execution_duration_scaling_.find(context.controllers_[i]);
      const double current_scaling = scaling_it != controller_allowed_execution_duration_scaling_.end() ?
                                         scaling_it->second :
                                         allowed_execution_duration_scaling_;

      std::map<std::string, double>::const_iterator margin_it =
          controller_allowed_goal_duration_margin_.find(context.controllers_[i]);
      const double current_margin = margin_it != controller_allowed_goal_duration_margin_.end() ?
                                        margin_it->second :
                                        allowed_goal_duration_margin_;

      // expected duration is the duration of the longest part
      expected_trajectory_duration =
          std::max(d * current_scaling + rclcpp::Duration::from_seconds(current_margin), expected_trajectory_duration);
    }

    // construct a map from expected time to state index, for easy access to expected state location
    if (longest_part >= 0)
    {
      std::scoped_lock slock(time_index_mutex_);

      if (context.trajectory_parts_[longest_part].joint_trajectory.points.size() >=
          context.trajectory_parts_[longest_part].multi_dof_joint_trajectory.points.size())
      {
        auto d = rclcpp::Duration::from_seconds(0);
        if (rclcpp::Time(context.trajectory_parts_[longest_part].joint_trajectory.header.stamp) > current_time)
          d = rclcpp::Time(context.trajectory_parts_[longest_part].joint_trajectory.header.stamp) - current_time;
        for (trajectory_msgs::msg::JointTrajectoryPoint& point :
             context.trajectory_parts_[longest_part].joint_trajectory.points)
          time_index_.push_back(current_time + d + rclcpp::Duration(point.time_from_start));
      }
      else
      {
        auto d = rclcpp::Duration::from_seconds(0);
        if (rclcpp::Time(context.trajectory_parts_[longest_part].multi_dof_joint_trajectory.header.stamp) > current_time)
        {
          d = rclcpp::Time(context.trajectory_parts_[longest_part].multi_dof_joint_trajectory.header.stamp) -
              current_time;
        }
        for (trajectory_msgs::msg::MultiDOFJointTrajectoryPoint& point :
             context.trajectory_parts_[longest_part].multi_dof_joint_trajectory.points)
          time_index_.push_back(current_time + d + rclcpp::Duration(point.time_from_start));
      }
    }

    bool result = true;
    for (moveit_controller_manager::MoveItControllerHandlePtr& handle : handles)
    {
      if (execution_duration_monitoring_)
      {
        if (!handle->waitForExecution(expected_trajectory_duration))
        {
          if (!execution_complete_ && node_->now() - current_time > expected_trajectory_duration)
          {
            RCLCPP_ERROR(LOGGER,
                         "Controller is taking too long to execute trajectory (the expected upper "
                         "bound for the trajectory execution was %lf seconds). Stopping trajectory.",
                         expected_trajectory_duration.seconds());
            {
              std::scoped_lock slock(execution_state_mutex_);
              stopExecutionInternal();  // this is really tricky. we can't call stopExecution() here, so we call the
                                        // internal function only
            }
            last_execution_status_ = moveit_controller_manager::ExecutionStatus::TIMED_OUT;
            result = false;
            break;
          }
        }
      }
      else
        handle->waitForExecution();

      // if something made the trajectory stop, we stop this thread too
      if (execution_complete_)
      {
        result = false;
        break;
      }
      else if (handle->getLastExecutionStatus() != moveit_controller_manager::ExecutionStatus::SUCCEEDED)
      {
        RCLCPP_WARN_STREAM(LOGGER, "Controller handle " << handle->getName() << " reports status "
                                                        << handle->getLastExecutionStatus().asString());
        last_execution_status_ = handle->getLastExecutionStatus();
        result = false;
      }
    }

    // clear the active handles
    execution_state_mutex_.lock();
    active_handles_.clear();

    // clear the time index
    time_index_mutex_.lock();
    time_index_.clear();
    current_context_ = -1;
    time_index_mutex_.unlock();

    execution_state_mutex_.unlock();
    return result;
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Active status of required controllers can not be assured.");
    last_execution_status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
    return false;
  }
}

bool TrajectoryExecutionManager::waitForRobotToStop(const TrajectoryExecutionContext& context, double wait_time)
{
  // skip waiting for convergence?
  if (allowed_start_tolerance_ == 0 || !wait_for_trajectory_completion_)
  {
    RCLCPP_INFO(LOGGER, "Not waiting for trajectory completion");
    return true;
  }

  auto start = std::chrono::system_clock::now();
  double time_remaining = wait_time;

  moveit::core::RobotStatePtr prev_state, cur_state;
  prev_state = csm_->getCurrentState();
  prev_state->enforceBounds();

  // assume robot stopped when 3 consecutive checks yield the same robot state
  unsigned int no_motion_count = 0;  // count iterations with no motion
  while (time_remaining > 0. && no_motion_count < 3)
  {
    if (!csm_->waitForCurrentState(node_->now(), time_remaining) || !(cur_state = csm_->getCurrentState()))
    {
      RCLCPP_WARN(LOGGER, "Failed to receive current joint state");
      return false;
    }
    cur_state->enforceBounds();
    std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - start;
    time_remaining = wait_time - elapsed_seconds.count();  // remaining wait_time

    // check for motion in effected joints of execution context
    bool moved = false;
    for (const auto& trajectory : context.trajectory_parts_)
    {
      const std::vector<std::string>& joint_names = trajectory.joint_trajectory.joint_names;
      const std::size_t n = joint_names.size();

      for (std::size_t i = 0; i < n && !moved; ++i)
      {
        const moveit::core::JointModel* jm = cur_state->getJointModel(joint_names[i]);
        if (!jm)
          continue;  // joint vanished from robot state (shouldn't happen), but we don't care

        if (fabs(cur_state->getJointPositions(jm)[0] - prev_state->getJointPositions(jm)[0]) > allowed_start_tolerance_)
        {
          moved = true;
          no_motion_count = 0;
          break;
        }
      }
    }

    if (!moved)
      ++no_motion_count;

    std::swap(prev_state, cur_state);
  }

  return time_remaining > 0;
}

std::pair<int, int> TrajectoryExecutionManager::getCurrentExpectedTrajectoryIndex() const
{
  std::scoped_lock slock(time_index_mutex_);
  if (current_context_ < 0)
    return std::make_pair(-1, -1);
  if (time_index_.empty())
    return std::make_pair(static_cast<int>(current_context_), -1);
  std::vector<rclcpp::Time>::const_iterator time_index_it =
      std::lower_bound(time_index_.begin(), time_index_.end(), node_->now());
  int pos = time_index_it - time_index_.begin();
  return std::make_pair(static_cast<int>(current_context_), pos);
}

const std::vector<TrajectoryExecutionManager::TrajectoryExecutionContext*>&
TrajectoryExecutionManager::getTrajectories() const
{
  return trajectories_;
}

moveit_controller_manager::ExecutionStatus TrajectoryExecutionManager::getLastExecutionStatus() const
{
  return last_execution_status_;
}

bool TrajectoryExecutionManager::ensureActiveControllersForGroup(const std::string& group)
{
  const moveit::core::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group);
  if (joint_model_group)
  {
    return ensureActiveControllersForJoints(joint_model_group->getJointModelNames());
  }
  else
  {
    return false;
  }
}

bool TrajectoryExecutionManager::ensureActiveControllersForJoints(const std::vector<std::string>& joints)
{
  std::vector<std::string> all_controller_names;
  for (std::map<std::string, ControllerInformation>::const_iterator it = known_controllers_.begin();
       it != known_controllers_.end(); ++it)
    all_controller_names.push_back(it->first);
  std::vector<std::string> selected_controllers;
  std::set<std::string> jset;
  for (const std::string& joint : joints)
  {
    const moveit::core::JointModel* jm = robot_model_->getJointModel(joint);
    if (jm)
    {
      if (jm->isPassive() || jm->getMimic() != nullptr || jm->getType() == moveit::core::JointModel::FIXED)
        continue;
      jset.insert(joint);
    }
  }

  if (selectControllers(jset, all_controller_names, selected_controllers))
  {
    return ensureActiveControllers(selected_controllers);
  }
  else
  {
    return false;
  }
}

bool TrajectoryExecutionManager::ensureActiveController(const std::string& controller)
{
  return ensureActiveControllers(std::vector<std::string>(1, controller));
}

bool TrajectoryExecutionManager::ensureActiveControllers(const std::vector<std::string>& controllers)
{
  reloadControllerInformation();

  updateControllersState(DEFAULT_CONTROLLER_INFORMATION_VALIDITY_AGE);

  if (manage_controllers_)
  {
    std::vector<std::string> controllers_to_activate;
    std::vector<std::string> controllers_to_deactivate;
    std::set<std::string> joints_to_be_activated;
    std::set<std::string> joints_to_be_deactivated;
    for (const std::string& controller : controllers)
    {
      std::map<std::string, ControllerInformation>::const_iterator it = known_controllers_.find(controller);
      if (it == known_controllers_.end())
      {
        std::stringstream stream;
        for (const auto& controller : known_controllers_)
        {
          stream << " `" << controller.first << '`';
        }
        RCLCPP_WARN_STREAM(LOGGER, "Controller " << controller << " is not known. Known controllers: " << stream.str());
        return false;
      }
      if (!it->second.state_.active_)
      {
        RCLCPP_DEBUG_STREAM(LOGGER, "Need to activate " << controller);
        controllers_to_activate.push_back(controller);
        joints_to_be_activated.insert(it->second.joints_.begin(), it->second.joints_.end());
        for (const std::string& overlapping_controller : it->second.overlapping_controllers_)
        {
          const ControllerInformation& ci = known_controllers_[overlapping_controller];
          if (ci.state_.active_)
          {
            controllers_to_deactivate.push_back(overlapping_controller);
            joints_to_be_deactivated.insert(ci.joints_.begin(), ci.joints_.end());
          }
        }
      }
      else
        RCLCPP_DEBUG_STREAM(LOGGER, "Controller " << controller << " is already active");
    }
    std::set<std::string> diff;
    std::set_difference(joints_to_be_deactivated.begin(), joints_to_be_deactivated.end(),
                        joints_to_be_activated.begin(), joints_to_be_activated.end(), std::inserter(diff, diff.end()));
    if (!diff.empty())
    {
      // find the set of controllers that do not overlap with the ones we want to activate so far
      std::vector<std::string> possible_additional_controllers;
      for (std::map<std::string, ControllerInformation>::const_iterator it = known_controllers_.begin();
           it != known_controllers_.end(); ++it)
      {
        bool ok = true;
        for (const std::string& controller_to_activate : controllers_to_activate)
        {
          if (it->second.overlapping_controllers_.find(controller_to_activate) !=
              it->second.overlapping_controllers_.end())
          {
            ok = false;
            break;
          }
        }
        if (ok)
          possible_additional_controllers.push_back(it->first);
      }

      // out of the allowable controllers, try to find a subset of controllers that covers the joints to be actuated
      std::vector<std::string> additional_controllers;
      if (selectControllers(diff, possible_additional_controllers, additional_controllers))
      {
        controllers_to_activate.insert(controllers_to_activate.end(), additional_controllers.begin(),
                                       additional_controllers.end());
      }
      else
      {
        return false;
      }
    }
    if (!controllers_to_activate.empty() || !controllers_to_deactivate.empty())
    {
      if (controller_manager_)
      {
        // load controllers to be activated, if needed, and reset the state update cache
        for (const std::string& controller_to_activate : controllers_to_activate)
        {
          ControllerInformation& ci = known_controllers_[controller_to_activate];
          ci.last_update_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        }
        // reset the state update cache
        for (const std::string& controller_to_activate : controllers_to_deactivate)
          known_controllers_[controller_to_activate].last_update_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        return controller_manager_->switchControllers(controllers_to_activate, controllers_to_deactivate);
      }
      else
        return false;
    }
    else
      return true;
  }
  else
  {
    std::set<std::string> originally_active;
    for (std::map<std::string, ControllerInformation>::const_iterator it = known_controllers_.begin();
         it != known_controllers_.end(); ++it)
    {
      if (it->second.state_.active_)
      {
        originally_active.insert(it->first);
      }
    }
    return std::includes(originally_active.begin(), originally_active.end(), controllers.begin(), controllers.end());
  }
}

void TrajectoryExecutionManager::loadControllerParams()
{
  // TODO: Revise XmlRpc parameter lookup
  // XmlRpc::XmlRpcValue controller_list;
  // if (node_->get_parameter("controller_list", controller_list) &&
  //     controller_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
  // {
  //   for (int i = 0; i < controller_list.size(); ++i)  // NOLINT(modernize-loop-convert)
  //   {
  //     XmlRpc::XmlRpcValue& controller = controller_list[i];
  //     if (controller.hasMember("name"))
  //     {
  //       if (controller.hasMember("allowed_execution_duration_scaling"))
  //         controller_allowed_execution_duration_scaling_[std::string(controller["name"])] =
  //             controller["allowed_execution_duration_scaling"];
  //       if (controller.hasMember("allowed_goal_duration_margin"))
  //         controller_allowed_goal_duration_margin_[std::string(controller["name"])] =
  //             controller["allowed_goal_duration_margin"];
  //     }
  //   }
  // }
}
}  // namespace trajectory_execution_manager
