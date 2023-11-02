/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, SRI International
 *  Copyright (c) 2013, Ioan A. Sucan
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

/* Author: Ioan Sucan, Sachin Chitta */

#include <stdexcept>
#include <sstream>
#include <memory>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/srv/query_planner_interfaces.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>
#include <moveit_msgs/srv/grasp_planning.hpp>
#include <moveit_msgs/srv/get_planner_params.hpp>
#include <moveit_msgs/srv/set_planner_params.hpp>
#include <moveit/utils/rclcpp_utils.h>
#include <moveit/utils/logger.hpp>

#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/version.h>

namespace moveit
{
namespace planning_interface
{
const std::string MoveGroupInterface::ROBOT_DESCRIPTION =
    "robot_description";  // name of the robot description (a param name, so it can be changed externally)

const std::string GRASP_PLANNING_SERVICE_NAME = "plan_grasps";  // name of the service that can be used to plan grasps

namespace
{
enum ActiveTargetType
{
  JOINT,
  POSE,
  POSITION,
  ORIENTATION
};

// Function to support both Rolling and Humble on the main branch
// Rolling has deprecated the version of the create_client method that takes
// rmw_qos_profile_services_default for the QoS argument
#if RCLCPP_VERSION_GTE(17, 0, 0)  // Rolling
auto qosDefault()
{
  return rclcpp::SystemDefaultsQoS();
}
#else  // Humble
auto qosDefault()
{
  return rmw_qos_profile_services_default;
}
#endif

}  // namespace

class MoveGroupInterface::MoveGroupInterfaceImpl
{
  friend MoveGroupInterface;

public:
  MoveGroupInterfaceImpl(const rclcpp::Node::SharedPtr& node, const Options& opt,
                         const std::shared_ptr<tf2_ros::Buffer>& tf_buffer, const rclcpp::Duration& wait_for_servers)
    : opt_(opt), node_(node), logger_(moveit::getLogger("move_group_interface")), tf_buffer_(tf_buffer)
  {
    // We have no control on how the passed node is getting executed. To make sure MGI is functional, we're creating
    // our own callback group which is managed in a separate callback thread
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,
                                                   false /* don't spin with node executor */);
    callback_executor_.add_callback_group(callback_group_, node->get_node_base_interface());
    callback_thread_ = std::thread([this]() { callback_executor_.spin(); });

    robot_model_ = opt.robot_model ? opt.robot_model : getSharedRobotModel(node_, opt.robot_description);
    if (!getRobotModel())
    {
      std::string error = "Unable to construct robot model. Please make sure all needed information is on the "
                          "parameter server.";
      RCLCPP_FATAL_STREAM(logger_, error);
      throw std::runtime_error(error);
    }

    if (!getRobotModel()->hasJointModelGroup(opt.group_name))
    {
      std::string error = "Group '" + opt.group_name + "' was not found.";
      RCLCPP_FATAL_STREAM(logger_, error);
      throw std::runtime_error(error);
    }

    joint_model_group_ = getRobotModel()->getJointModelGroup(opt.group_name);

    joint_state_target_ = std::make_shared<moveit::core::RobotState>(getRobotModel());
    joint_state_target_->setToDefaultValues();
    active_target_ = JOINT;
    can_look_ = false;
    look_around_attempts_ = 0;
    can_replan_ = false;
    replan_delay_ = 2.0;
    replan_attempts_ = 1;
    goal_joint_tolerance_ = 1e-4;
    goal_position_tolerance_ = 1e-4;     // 0.1 mm
    goal_orientation_tolerance_ = 1e-3;  // ~0.1 deg
    allowed_planning_time_ = 5.0;
    num_planning_attempts_ = 1;
    node_->get_parameter_or<double>("robot_description_planning.default_velocity_scaling_factor",
                                    max_velocity_scaling_factor_, 0.1);
    node_->get_parameter_or<double>("robot_description_planning.default_acceleration_scaling_factor",
                                    max_acceleration_scaling_factor_, 0.1);
    initializing_constraints_ = false;

    if (joint_model_group_->isChain())
      end_effector_link_ = joint_model_group_->getLinkModelNames().back();
    pose_reference_frame_ = getRobotModel()->getModelFrame();
    // Append the slash between two topic components
    trajectory_event_publisher_ = node_->create_publisher<std_msgs::msg::String>(
        rclcpp::names::append(opt_.move_group_namespace,
                              trajectory_execution_manager::TrajectoryExecutionManager::EXECUTION_EVENT_TOPIC),
        1);
    attached_object_publisher_ = node_->create_publisher<moveit_msgs::msg::AttachedCollisionObject>(
        rclcpp::names::append(opt_.move_group_namespace,
                              planning_scene_monitor::PlanningSceneMonitor::DEFAULT_ATTACHED_COLLISION_OBJECT_TOPIC),
        1);

    current_state_monitor_ = getSharedStateMonitor(node_, robot_model_, tf_buffer_);

    move_action_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
        node_, rclcpp::names::append(opt_.move_group_namespace, move_group::MOVE_ACTION), callback_group_);
    move_action_client_->wait_for_action_server(wait_for_servers.to_chrono<std::chrono::duration<double>>());
    execute_action_client_ = rclcpp_action::create_client<moveit_msgs::action::ExecuteTrajectory>(
        node_, rclcpp::names::append(opt_.move_group_namespace, move_group::EXECUTE_ACTION_NAME), callback_group_);
    execute_action_client_->wait_for_action_server(wait_for_servers.to_chrono<std::chrono::duration<double>>());

    query_service_ = node_->create_client<moveit_msgs::srv::QueryPlannerInterfaces>(
        rclcpp::names::append(opt_.move_group_namespace, move_group::QUERY_PLANNERS_SERVICE_NAME), qosDefault(),
        callback_group_);
    get_params_service_ = node_->create_client<moveit_msgs::srv::GetPlannerParams>(
        rclcpp::names::append(opt_.move_group_namespace, move_group::GET_PLANNER_PARAMS_SERVICE_NAME), qosDefault(),
        callback_group_);
    set_params_service_ = node_->create_client<moveit_msgs::srv::SetPlannerParams>(
        rclcpp::names::append(opt_.move_group_namespace, move_group::SET_PLANNER_PARAMS_SERVICE_NAME), qosDefault(),
        callback_group_);
    cartesian_path_service_ = node_->create_client<moveit_msgs::srv::GetCartesianPath>(
        rclcpp::names::append(opt_.move_group_namespace, move_group::CARTESIAN_PATH_SERVICE_NAME), qosDefault(),
        callback_group_);

    RCLCPP_INFO_STREAM(logger_, "Ready to take commands for planning group " << opt.group_name << '.');
  }

  ~MoveGroupInterfaceImpl()
  {
    if (constraints_init_thread_)
      constraints_init_thread_->join();

    callback_executor_.cancel();

    if (callback_thread_.joinable())
      callback_thread_.join();
  }

  const std::shared_ptr<tf2_ros::Buffer>& getTF() const
  {
    return tf_buffer_;
  }

  const Options& getOptions() const
  {
    return opt_;
  }

  const moveit::core::RobotModelConstPtr& getRobotModel() const
  {
    return robot_model_;
  }

  const moveit::core::JointModelGroup* getJointModelGroup() const
  {
    return joint_model_group_;
  }

  rclcpp_action::Client<moveit_msgs::action::MoveGroup>& getMoveGroupClient() const
  {
    return *move_action_client_;
  }

  bool getInterfaceDescription(moveit_msgs::msg::PlannerInterfaceDescription& desc)
  {
    auto req = std::make_shared<moveit_msgs::srv::QueryPlannerInterfaces::Request>();
    auto future_response = query_service_->async_send_request(req);

    if (future_response.valid())
    {
      const auto& response = future_response.get();
      if (!response->planner_interfaces.empty())
      {
        desc = response->planner_interfaces.front();
        return true;
      }
    }
    return false;
  }

  bool getInterfaceDescriptions(std::vector<moveit_msgs::msg::PlannerInterfaceDescription>& desc)
  {
    auto req = std::make_shared<moveit_msgs::srv::QueryPlannerInterfaces::Request>();
    auto future_response = query_service_->async_send_request(req);
    if (future_response.valid())
    {
      const auto& response = future_response.get();
      if (!response->planner_interfaces.empty())
      {
        desc = response->planner_interfaces;
        return true;
      }
    }
    return false;
  }

  std::map<std::string, std::string> getPlannerParams(const std::string& planner_id, const std::string& group = "")
  {
    auto req = std::make_shared<moveit_msgs::srv::GetPlannerParams::Request>();
    moveit_msgs::srv::GetPlannerParams::Response::SharedPtr response;
    req->planner_config = planner_id;
    req->group = group;
    std::map<std::string, std::string> result;

    auto future_response = get_params_service_->async_send_request(req);
    if (future_response.valid())
    {
      response = future_response.get();
      for (unsigned int i = 0, end = response->params.keys.size(); i < end; ++i)
        result[response->params.keys[i]] = response->params.values[i];
    }
    return result;
  }

  void setPlannerParams(const std::string& planner_id, const std::string& group,
                        const std::map<std::string, std::string>& params, bool replace = false)
  {
    auto req = std::make_shared<moveit_msgs::srv::SetPlannerParams::Request>();
    req->planner_config = planner_id;
    req->group = group;
    req->replace = replace;
    for (const std::pair<const std::string, std::string>& param : params)
    {
      req->params.keys.push_back(param.first);
      req->params.values.push_back(param.second);
    }
    set_params_service_->async_send_request(req);
  }

  std::string getDefaultPlanningPipelineId() const
  {
    std::string default_planning_pipeline;
    node_->get_parameter("move_group.default_planning_pipeline", default_planning_pipeline);
    return default_planning_pipeline;
  }

  void setPlanningPipelineId(const std::string& pipeline_id)
  {
    if (pipeline_id != planning_pipeline_id_)
    {
      planning_pipeline_id_ = pipeline_id;

      // Reset planner_id if planning pipeline changed
      planner_id_ = "";
    }
  }

  const std::string& getPlanningPipelineId() const
  {
    return planning_pipeline_id_;
  }

  std::string getDefaultPlannerId(const std::string& group) const
  {
    // Check what planning pipeline config should be used
    std::string pipeline_id = getDefaultPlanningPipelineId();
    if (!planning_pipeline_id_.empty())
      pipeline_id = planning_pipeline_id_;

    std::stringstream param_name;
    param_name << "move_group";
    if (!pipeline_id.empty())
      param_name << "/planning_pipelines/" << pipeline_id;
    if (!group.empty())
      param_name << '.' << group;
    param_name << ".default_planner_config";

    std::string default_planner_config;
    node_->get_parameter(param_name.str(), default_planner_config);
    return default_planner_config;
  }

  void setPlannerId(const std::string& planner_id)
  {
    planner_id_ = planner_id;
  }

  const std::string& getPlannerId() const
  {
    return planner_id_;
  }

  void setNumPlanningAttempts(unsigned int num_planning_attempts)
  {
    num_planning_attempts_ = num_planning_attempts;
  }

  void setMaxVelocityScalingFactor(double value)
  {
    setMaxScalingFactor(max_velocity_scaling_factor_, value, "velocity_scaling_factor", 0.1);
  }

  void setMaxAccelerationScalingFactor(double value)
  {
    setMaxScalingFactor(max_acceleration_scaling_factor_, value, "acceleration_scaling_factor", 0.1);
  }

  void setMaxScalingFactor(double& variable, const double target_value, const char* factor_name, double fallback_value)
  {
    if (target_value > 1.0)
    {
      RCLCPP_WARN(logger_, "Limiting max_%s (%.2f) to 1.0.", factor_name, target_value);
      variable = 1.0;
    }
    else if (target_value <= 0.0)
    {
      node_->get_parameter_or<double>(std::string("robot_description_planning.default_") + factor_name, variable,
                                      fallback_value);
      if (target_value < 0.0)
      {
        RCLCPP_WARN(logger_, "max_%s < 0.0! Setting to default: %.2f.", factor_name, variable);
      }
    }
    else
    {
      variable = target_value;
    }
  }

  moveit::core::RobotState& getTargetRobotState()
  {
    return *joint_state_target_;
  }

  const moveit::core::RobotState& getTargetRobotState() const
  {
    return *joint_state_target_;
  }

  void setStartState(const moveit::core::RobotState& start_state)
  {
    considered_start_state_ = std::make_shared<moveit::core::RobotState>(start_state);
  }

  void setStartStateToCurrentState()
  {
    considered_start_state_.reset();
  }

  moveit::core::RobotStatePtr getStartState()
  {
    if (considered_start_state_)
    {
      return considered_start_state_;
    }
    else
    {
      moveit::core::RobotStatePtr s;
      getCurrentState(s);
      return s;
    }
  }

  bool setJointValueTarget(const geometry_msgs::msg::Pose& eef_pose, const std::string& end_effector_link,
                           const std::string& frame, bool approx)
  {
    const std::string& eef = end_effector_link.empty() ? getEndEffectorLink() : end_effector_link;
    // this only works if we have an end-effector
    if (!eef.empty())
    {
      // first we set the goal to be the same as the start state
      moveit::core::RobotStatePtr c = getStartState();
      if (c)
      {
        setTargetType(JOINT);
        c->enforceBounds();
        getTargetRobotState() = *c;
        if (!getTargetRobotState().satisfiesBounds(getGoalJointTolerance()))
          return false;
      }
      else
        return false;

      // we may need to do approximate IK
      kinematics::KinematicsQueryOptions o;
      o.return_approximate_solution = approx;

      // if no frame transforms are needed, call IK directly
      if (frame.empty() || moveit::core::Transforms::sameFrame(frame, getRobotModel()->getModelFrame()))
      {
        return getTargetRobotState().setFromIK(getJointModelGroup(), eef_pose, eef, 0.0,
                                               moveit::core::GroupStateValidityCallbackFn(), o);
      }
      else
      {
        // transform the pose into the model frame, then do IK
        bool frame_found;
        const Eigen::Isometry3d& t = getTargetRobotState().getFrameTransform(frame, &frame_found);
        if (frame_found)
        {
          Eigen::Isometry3d p;
          tf2::fromMsg(eef_pose, p);
          return getTargetRobotState().setFromIK(getJointModelGroup(), t * p, eef, 0.0,
                                                 moveit::core::GroupStateValidityCallbackFn(), o);
        }
        else
        {
          RCLCPP_ERROR(logger_, "Unable to transform from frame '%s' to frame '%s'", frame.c_str(),
                       getRobotModel()->getModelFrame().c_str());
          return false;
        }
      }
    }
    else
      return false;
  }

  void setEndEffectorLink(const std::string& end_effector)
  {
    end_effector_link_ = end_effector;
  }

  void clearPoseTarget(const std::string& end_effector_link)
  {
    pose_targets_.erase(end_effector_link);
  }

  void clearPoseTargets()
  {
    pose_targets_.clear();
  }

  const std::string& getEndEffectorLink() const
  {
    return end_effector_link_;
  }

  const std::string& getEndEffector() const
  {
    if (!end_effector_link_.empty())
    {
      const std::vector<std::string>& possible_eefs =
          getRobotModel()->getJointModelGroup(opt_.group_name)->getAttachedEndEffectorNames();
      for (const std::string& possible_eef : possible_eefs)
      {
        if (getRobotModel()->getEndEffector(possible_eef)->hasLinkModel(end_effector_link_))
          return possible_eef;
      }
    }
    static std::string empty;
    return empty;
  }

  bool setPoseTargets(const std::vector<geometry_msgs::msg::PoseStamped>& poses, const std::string& end_effector_link)
  {
    const std::string& eef = end_effector_link.empty() ? end_effector_link_ : end_effector_link;
    if (eef.empty())
    {
      RCLCPP_ERROR(logger_, "No end-effector to set the pose for");
      return false;
    }
    else
    {
      pose_targets_[eef] = poses;
      // make sure we don't store an actual stamp, since that will become stale can potentially cause tf errors
      std::vector<geometry_msgs::msg::PoseStamped>& stored_poses = pose_targets_[eef];
      for (geometry_msgs::msg::PoseStamped& stored_pose : stored_poses)
        stored_pose.header.stamp = rclcpp::Time(0);
    }
    return true;
  }

  bool hasPoseTarget(const std::string& end_effector_link) const
  {
    const std::string& eef = end_effector_link.empty() ? end_effector_link_ : end_effector_link;
    return pose_targets_.find(eef) != pose_targets_.end();
  }

  const geometry_msgs::msg::PoseStamped& getPoseTarget(const std::string& end_effector_link) const
  {
    const std::string& eef = end_effector_link.empty() ? end_effector_link_ : end_effector_link;

    // if multiple pose targets are set, return the first one
    std::map<std::string, std::vector<geometry_msgs::msg::PoseStamped>>::const_iterator jt = pose_targets_.find(eef);
    if (jt != pose_targets_.end())
    {
      if (!jt->second.empty())
        return jt->second.at(0);
    }

    // or return an error
    static const geometry_msgs::msg::PoseStamped UNKNOWN;
    RCLCPP_ERROR(logger_, "Pose for end-effector '%s' not known.", eef.c_str());
    return UNKNOWN;
  }

  const std::vector<geometry_msgs::msg::PoseStamped>& getPoseTargets(const std::string& end_effector_link) const
  {
    const std::string& eef = end_effector_link.empty() ? end_effector_link_ : end_effector_link;

    std::map<std::string, std::vector<geometry_msgs::msg::PoseStamped>>::const_iterator jt = pose_targets_.find(eef);
    if (jt != pose_targets_.end())
    {
      if (!jt->second.empty())
        return jt->second;
    }

    // or return an error
    static const std::vector<geometry_msgs::msg::PoseStamped> EMPTY;
    RCLCPP_ERROR(logger_, "Poses for end-effector '%s' are not known.", eef.c_str());
    return EMPTY;
  }

  void setPoseReferenceFrame(const std::string& pose_reference_frame)
  {
    pose_reference_frame_ = pose_reference_frame;
  }

  void setSupportSurfaceName(const std::string& support_surface)
  {
    support_surface_ = support_surface;
  }

  const std::string& getPoseReferenceFrame() const
  {
    return pose_reference_frame_;
  }

  void setTargetType(ActiveTargetType type)
  {
    active_target_ = type;
  }

  ActiveTargetType getTargetType() const
  {
    return active_target_;
  }

  bool startStateMonitor(double wait)
  {
    if (!current_state_monitor_)
    {
      RCLCPP_ERROR(logger_, "Unable to monitor current robot state");
      return false;
    }

    // if needed, start the monitor and wait up to 1 second for a full robot state
    if (!current_state_monitor_->isActive())
      current_state_monitor_->startStateMonitor();

    current_state_monitor_->waitForCompleteState(opt_.group_name, wait);
    return true;
  }

  bool getCurrentState(moveit::core::RobotStatePtr& current_state, double wait_seconds = 1.0)
  {
    if (!current_state_monitor_)
    {
      RCLCPP_ERROR(logger_, "Unable to get current robot state");
      return false;
    }

    // if needed, start the monitor and wait up to 1 second for a full robot state
    if (!current_state_monitor_->isActive())
      current_state_monitor_->startStateMonitor();

    if (!current_state_monitor_->waitForCurrentState(node_->now(), wait_seconds))
    {
      RCLCPP_ERROR(logger_, "Failed to fetch current robot state");
      return false;
    }

    current_state = current_state_monitor_->getCurrentState();
    return true;
  }

  moveit::core::MoveItErrorCode plan(Plan& plan)
  {
    if (!move_action_client_ || !move_action_client_->action_server_is_ready())
    {
      RCLCPP_INFO_STREAM(logger_, "MoveGroup action client/server not ready");
      return moveit::core::MoveItErrorCode::FAILURE;
    }
    RCLCPP_INFO_STREAM(logger_, "MoveGroup action client/server ready");

    moveit_msgs::action::MoveGroup::Goal goal;
    constructGoal(goal);
    goal.planning_options.plan_only = true;
    goal.planning_options.look_around = false;
    goal.planning_options.replan = false;
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

    bool done = false;
    rclcpp_action::ResultCode code = rclcpp_action::ResultCode::UNKNOWN;
    std::shared_ptr<moveit_msgs::action::MoveGroup::Result> res;
    auto send_goal_opts = rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SendGoalOptions();

    send_goal_opts.goal_response_callback =
        [&](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr& goal_handle) {
          if (!goal_handle)
          {
            done = true;
            RCLCPP_INFO(logger_, "Planning request rejected");
          }
          else
            RCLCPP_INFO(logger_, "Planning request accepted");
        };
    send_goal_opts.result_callback =
        [&](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::WrappedResult& result) {
          res = result.result;
          code = result.code;
          done = true;

          switch (result.code)
          {
            case rclcpp_action::ResultCode::SUCCEEDED:
              RCLCPP_INFO(logger_, "Planning request complete!");
              break;
            case rclcpp_action::ResultCode::ABORTED:
              RCLCPP_INFO(logger_, "Planning request aborted");
              return;
            case rclcpp_action::ResultCode::CANCELED:
              RCLCPP_INFO(logger_, "Planning request canceled");
              return;
            default:
              RCLCPP_INFO(logger_, "Planning request unknown result code");
              return;
          }
        };

    auto goal_handle_future = move_action_client_->async_send_goal(goal, send_goal_opts);

    // wait until send_goal_opts.result_callback is called
    while (!done)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (code != rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_ERROR_STREAM(logger_, "MoveGroupInterface::plan() failed or timeout reached");
      return res->error_code;
    }

    plan.trajectory = res->planned_trajectory;
    plan.start_state = res->trajectory_start;
    plan.planning_time = res->planning_time;
    RCLCPP_INFO(logger_, "time taken to generate plan: %g seconds", plan.planning_time);

    return res->error_code;
  }

  moveit::core::MoveItErrorCode move(bool wait)
  {
    if (!move_action_client_ || !move_action_client_->action_server_is_ready())
    {
      RCLCPP_INFO_STREAM(logger_, "MoveGroup action client/server not ready");
      return moveit::core::MoveItErrorCode::FAILURE;
    }

    moveit_msgs::action::MoveGroup::Goal goal;
    constructGoal(goal);
    goal.planning_options.plan_only = false;
    goal.planning_options.look_around = can_look_;
    goal.planning_options.replan = can_replan_;
    goal.planning_options.replan_delay = replan_delay_;
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

    bool done = false;
    rclcpp_action::ResultCode code = rclcpp_action::ResultCode::UNKNOWN;
    std::shared_ptr<moveit_msgs::action::MoveGroup_Result> res;
    auto send_goal_opts = rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SendGoalOptions();

    send_goal_opts.goal_response_callback =
        [&](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr& goal_handle) {
          if (!goal_handle)
          {
            done = true;
            RCLCPP_INFO(logger_, "Plan and Execute request rejected");
          }
          else
            RCLCPP_INFO(logger_, "Plan and Execute request accepted");
        };
    send_goal_opts.result_callback =
        [&](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::WrappedResult& result) {
          res = result.result;
          code = result.code;
          done = true;

          switch (result.code)
          {
            case rclcpp_action::ResultCode::SUCCEEDED:
              RCLCPP_INFO(logger_, "Plan and Execute request complete!");
              break;
            case rclcpp_action::ResultCode::ABORTED:
              RCLCPP_INFO(logger_, "Plan and Execute request aborted");
              return;
            case rclcpp_action::ResultCode::CANCELED:
              RCLCPP_INFO(logger_, "Plan and Execute request canceled");
              return;
            default:
              RCLCPP_INFO(logger_, "Plan and Execute request unknown result code");
              return;
          }
        };
    auto goal_handle_future = move_action_client_->async_send_goal(goal, send_goal_opts);
    if (!wait)
      return moveit::core::MoveItErrorCode::SUCCESS;

    // wait until send_goal_opts.result_callback is called
    while (!done)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (code != rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_ERROR_STREAM(logger_, "MoveGroupInterface::move() failed or timeout reached");
    }
    return res->error_code;
  }

  moveit::core::MoveItErrorCode execute(const moveit_msgs::msg::RobotTrajectory& trajectory, bool wait,
                                        const std::vector<std::string>& controllers = std::vector<std::string>())
  {
    if (!execute_action_client_ || !execute_action_client_->action_server_is_ready())
    {
      RCLCPP_INFO_STREAM(logger_, "execute_action_client_ client/server not ready");
      return moveit::core::MoveItErrorCode::FAILURE;
    }

    bool done = false;
    rclcpp_action::ResultCode code = rclcpp_action::ResultCode::UNKNOWN;
    std::shared_ptr<moveit_msgs::action::ExecuteTrajectory_Result> res;
    auto send_goal_opts = rclcpp_action::Client<moveit_msgs::action::ExecuteTrajectory>::SendGoalOptions();

    send_goal_opts.goal_response_callback =
        [&](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::ExecuteTrajectory>::SharedPtr& goal_handle) {
          if (!goal_handle)
          {
            done = true;
            RCLCPP_INFO(logger_, "Execute request rejected");
          }
          else
            RCLCPP_INFO(logger_, "Execute request accepted");
        };
    send_goal_opts.result_callback =
        [&](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::ExecuteTrajectory>::WrappedResult& result) {
          res = result.result;
          code = result.code;
          done = true;

          switch (result.code)
          {
            case rclcpp_action::ResultCode::SUCCEEDED:
              RCLCPP_INFO(logger_, "Execute request success!");
              break;
            case rclcpp_action::ResultCode::ABORTED:
              RCLCPP_INFO(logger_, "Execute request aborted");
              return;
            case rclcpp_action::ResultCode::CANCELED:
              RCLCPP_INFO(logger_, "Execute request canceled");
              return;
            default:
              RCLCPP_INFO(logger_, "Execute request unknown result code");
              return;
          }
        };

    moveit_msgs::action::ExecuteTrajectory::Goal goal;
    goal.trajectory = trajectory;
    goal.controller_names = controllers;

    auto goal_handle_future = execute_action_client_->async_send_goal(goal, send_goal_opts);
    if (!wait)
      return moveit::core::MoveItErrorCode::SUCCESS;

    // wait until send_goal_opts.result_callback is called
    while (!done)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (code != rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_ERROR_STREAM(logger_, "MoveGroupInterface::execute() failed or timeout reached");
    }
    return res->error_code;
  }

  double computeCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints, double step,
                              double jump_threshold, moveit_msgs::msg::RobotTrajectory& msg,
                              const moveit_msgs::msg::Constraints& path_constraints, bool avoid_collisions,
                              moveit_msgs::msg::MoveItErrorCodes& error_code)
  {
    auto req = std::make_shared<moveit_msgs::srv::GetCartesianPath::Request>();
    moveit_msgs::srv::GetCartesianPath::Response::SharedPtr response;

    if (considered_start_state_)
    {
      moveit::core::robotStateToRobotStateMsg(*considered_start_state_, req->start_state);
    }
    else
    {
      req->start_state.is_diff = true;
    }

    req->group_name = opt_.group_name;
    req->header.frame_id = getPoseReferenceFrame();
    req->header.stamp = getClock()->now();
    req->waypoints = waypoints;
    req->max_step = step;
    req->jump_threshold = jump_threshold;
    req->path_constraints = path_constraints;
    req->avoid_collisions = avoid_collisions;
    req->link_name = getEndEffectorLink();
    req->max_velocity_scaling_factor = max_velocity_scaling_factor_;
    req->max_acceleration_scaling_factor = max_acceleration_scaling_factor_;

    auto future_response = cartesian_path_service_->async_send_request(req);
    if (future_response.valid())
    {
      response = future_response.get();
      error_code = response->error_code;
      if (response->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
      {
        msg = response->solution;
        return response->fraction;
      }
      else
        return -1.0;
    }
    else
    {
      error_code.val = error_code.FAILURE;
      return -1.0;
    }
  }

  void stop()
  {
    if (trajectory_event_publisher_)
    {
      std_msgs::msg::String event;
      event.data = "stop";
      trajectory_event_publisher_->publish(event);
    }
  }

  bool attachObject(const std::string& object, const std::string& link, const std::vector<std::string>& touch_links)
  {
    std::string l = link.empty() ? getEndEffectorLink() : link;
    if (l.empty())
    {
      const std::vector<std::string>& links = joint_model_group_->getLinkModelNames();
      if (!links.empty())
        l = links[0];
    }
    if (l.empty())
    {
      RCLCPP_ERROR(logger_, "No known link to attach object '%s' to", object.c_str());
      return false;
    }
    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.object.id = object;
    aco.link_name.swap(l);
    if (touch_links.empty())
    {
      aco.touch_links.push_back(aco.link_name);
    }
    else
    {
      aco.touch_links = touch_links;
    }
    aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    attached_object_publisher_->publish(aco);
    return true;
  }

  bool detachObject(const std::string& name)
  {
    moveit_msgs::msg::AttachedCollisionObject aco;
    // if name is a link
    if (!name.empty() && joint_model_group_->hasLinkModel(name))
    {
      aco.link_name = name;
    }
    else
    {
      aco.object.id = name;
    }
    aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    if (aco.link_name.empty() && aco.object.id.empty())
    {
      // we only want to detach objects for this group
      const std::vector<std::string>& lnames = joint_model_group_->getLinkModelNames();
      for (const std::string& lname : lnames)
      {
        aco.link_name = lname;
        attached_object_publisher_->publish(aco);
      }
    }
    else
    {
      attached_object_publisher_->publish(aco);
    }
    return true;
  }

  double getGoalPositionTolerance() const
  {
    return goal_position_tolerance_;
  }

  double getGoalOrientationTolerance() const
  {
    return goal_orientation_tolerance_;
  }

  double getGoalJointTolerance() const
  {
    return goal_joint_tolerance_;
  }

  void setGoalJointTolerance(double tolerance)
  {
    goal_joint_tolerance_ = tolerance;
  }

  void setGoalPositionTolerance(double tolerance)
  {
    goal_position_tolerance_ = tolerance;
  }

  void setGoalOrientationTolerance(double tolerance)
  {
    goal_orientation_tolerance_ = tolerance;
  }

  void setPlanningTime(double seconds)
  {
    if (seconds > 0.0)
      allowed_planning_time_ = seconds;
  }

  double getPlanningTime() const
  {
    return allowed_planning_time_;
  }

  void constructMotionPlanRequest(moveit_msgs::msg::MotionPlanRequest& request) const
  {
    request.group_name = opt_.group_name;
    request.num_planning_attempts = num_planning_attempts_;
    request.max_velocity_scaling_factor = max_velocity_scaling_factor_;
    request.max_acceleration_scaling_factor = max_acceleration_scaling_factor_;
    request.allowed_planning_time = allowed_planning_time_;
    request.pipeline_id = planning_pipeline_id_;
    request.planner_id = planner_id_;
    request.workspace_parameters = workspace_parameters_;

    if (considered_start_state_)
    {
      moveit::core::robotStateToRobotStateMsg(*considered_start_state_, request.start_state);
    }
    else
    {
      request.start_state.is_diff = true;
    }

    if (active_target_ == JOINT)
    {
      request.goal_constraints.resize(1);
      request.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(
          getTargetRobotState(), joint_model_group_, goal_joint_tolerance_);
    }
    else if (active_target_ == POSE || active_target_ == POSITION || active_target_ == ORIENTATION)
    {
      // find out how many goals are specified
      std::size_t goal_count = 0;
      for (const auto& pose_target : pose_targets_)
        goal_count = std::max(goal_count, pose_target.second.size());

      // start filling the goals;
      // each end effector has a number of possible poses (K) as valid goals
      // but there could be multiple end effectors specified, so we want each end effector
      // to reach the goal that corresponds to the goals of the other end effectors
      request.goal_constraints.resize(goal_count);

      for (const auto& pose_target : pose_targets_)
      {
        for (std::size_t i = 0; i < pose_target.second.size(); ++i)
        {
          moveit_msgs::msg::Constraints c = kinematic_constraints::constructGoalConstraints(
              pose_target.first, pose_target.second[i], goal_position_tolerance_, goal_orientation_tolerance_);
          if (active_target_ == ORIENTATION)
            c.position_constraints.clear();
          if (active_target_ == POSITION)
            c.orientation_constraints.clear();
          request.goal_constraints[i] = kinematic_constraints::mergeConstraints(request.goal_constraints[i], c);
        }
      }
    }
    else
      RCLCPP_ERROR(logger_, "Unable to construct MotionPlanRequest representation");

    if (path_constraints_)
      request.path_constraints = *path_constraints_;
    if (trajectory_constraints_)
      request.trajectory_constraints = *trajectory_constraints_;
  }

  void constructGoal(moveit_msgs::action::MoveGroup::Goal& goal) const
  {
    constructMotionPlanRequest(goal.request);
  }

  //  moveit_msgs::action::Pickup::Goal constructPickupGoal(const std::string& object,
  //                                                      std::vector<moveit_msgs::msg::Grasp>&& grasps,
  //                                                      bool plan_only = false) const
  //  {
  //    moveit_msgs::action::Pickup::Goal goal;
  //    goal.target_name = object;
  //    goal.group_name = opt_.group_name;
  //    goal.end_effector = getEndEffector();
  //    goal.support_surface_name = support_surface_;
  //    goal.possible_grasps = std::move(grasps);
  //    if (!support_surface_.empty())
  //      goal.allow_gripper_support_collision = true;
  //
  //    if (path_constraints_)
  //      goal.path_constraints = *path_constraints_;
  //
  //    goal.planner_id = planner_id_;
  //    goal.allowed_planning_time = allowed_planning_time_;
  //
  //    goal.planning_options.plan_only = plan_only;
  //    goal.planning_options.look_around = can_look_;
  //    goal.planning_options.replan = can_replan_;
  //    goal.planning_options.replan_delay = replan_delay_;
  //    goal.planning_options.planning_scene_diff.is_diff = true;
  //    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
  //    return goal;
  //  }

  //  moveit_msgs::action::Place::Goal constructPlaceGoal(const std::string& object,
  //                                                    std::vector<moveit_msgs::msg::PlaceLocation>&& locations,
  //                                                    bool plan_only = false) const
  //  {
  //    moveit_msgs::action::Place::Goal goal;
  //    goal.group_name = opt_.group_name;
  //    goal.attached_object_name = object;
  //    goal.support_surface_name = support_surface_;
  //    goal.place_locations = std::move(locations);
  //    if (!support_surface_.empty())
  //      goal.allow_gripper_support_collision = true;
  //
  //    if (path_constraints_)
  //      goal.path_constraints = *path_constraints_;
  //
  //    goal.planner_id = planner_id_;
  //    goal.allowed_planning_time = allowed_planning_time_;
  //
  //    goal.planning_options.plan_only = plan_only;
  //    goal.planning_options.look_around = can_look_;
  //    goal.planning_options.replan = can_replan_;
  //    goal.planning_options.replan_delay = replan_delay_;
  //    goal.planning_options.planning_scene_diff.is_diff = true;
  //    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
  //    return goal;
  //  }

  void setPathConstraints(const moveit_msgs::msg::Constraints& constraint)
  {
    path_constraints_ = std::make_unique<moveit_msgs::msg::Constraints>(constraint);
  }

  bool setPathConstraints(const std::string& constraint)
  {
    if (constraints_storage_)
    {
      moveit_warehouse::ConstraintsWithMetadata msg_m;
      if (constraints_storage_->getConstraints(msg_m, constraint, robot_model_->getName(), opt_.group_name))
      {
        path_constraints_ =
            std::make_unique<moveit_msgs::msg::Constraints>(static_cast<moveit_msgs::msg::Constraints>(*msg_m));
        return true;
      }
      else
        return false;
    }
    else
      return false;
  }

  void clearPathConstraints()
  {
    path_constraints_.reset();
  }

  void setTrajectoryConstraints(const moveit_msgs::msg::TrajectoryConstraints& constraint)
  {
    trajectory_constraints_ = std::make_unique<moveit_msgs::msg::TrajectoryConstraints>(constraint);
  }

  void clearTrajectoryConstraints()
  {
    trajectory_constraints_.reset();
  }

  std::vector<std::string> getKnownConstraints() const
  {
    while (initializing_constraints_)
    {
      std::chrono::duration<double> d(0.01);
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(d), rclcpp::Context::SharedPtr(nullptr));
    }

    std::vector<std::string> c;
    if (constraints_storage_)
      constraints_storage_->getKnownConstraints(c, robot_model_->getName(), opt_.group_name);

    return c;
  }

  moveit_msgs::msg::Constraints getPathConstraints() const
  {
    if (path_constraints_)
    {
      return *path_constraints_;
    }
    else
    {
      return moveit_msgs::msg::Constraints();
    }
  }

  moveit_msgs::msg::TrajectoryConstraints getTrajectoryConstraints() const
  {
    if (trajectory_constraints_)
    {
      return *trajectory_constraints_;
    }
    else
    {
      return moveit_msgs::msg::TrajectoryConstraints();
    }
  }

  void initializeConstraintsStorage(const std::string& host, unsigned int port)
  {
    initializing_constraints_ = true;
    if (constraints_init_thread_)
      constraints_init_thread_->join();
    constraints_init_thread_ =
        std::make_unique<std::thread>([this, host, port] { initializeConstraintsStorageThread(host, port); });
  }

  void setWorkspace(double minx, double miny, double minz, double maxx, double maxy, double maxz)
  {
    workspace_parameters_.header.frame_id = getRobotModel()->getModelFrame();
    workspace_parameters_.header.stamp = getClock()->now();
    workspace_parameters_.min_corner.x = minx;
    workspace_parameters_.min_corner.y = miny;
    workspace_parameters_.min_corner.z = minz;
    workspace_parameters_.max_corner.x = maxx;
    workspace_parameters_.max_corner.y = maxy;
    workspace_parameters_.max_corner.z = maxz;
  }

  rclcpp::Clock::SharedPtr getClock()
  {
    return node_->get_clock();
  }

private:
  void initializeConstraintsStorageThread(const std::string& host, unsigned int port)
  {
    // Set up db
    try
    {
      warehouse_ros::DatabaseConnection::Ptr conn = moveit_warehouse::loadDatabase(node_);
      conn->setParams(host, port);
      if (conn->connect())
      {
        constraints_storage_ = std::make_unique<moveit_warehouse::ConstraintsStorage>(conn);
      }
    }
    catch (std::exception& ex)
    {
      RCLCPP_ERROR(logger_, "%s", ex.what());
    }
    initializing_constraints_ = false;
  }

  Options opt_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_executor_;
  std::thread callback_thread_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  moveit::core::RobotModelConstPtr robot_model_;
  planning_scene_monitor::CurrentStateMonitorPtr current_state_monitor_;

  std::shared_ptr<rclcpp_action::Client<moveit_msgs::action::MoveGroup>> move_action_client_;
  // std::shared_ptr<rclcpp_action::Client<moveit_msgs::action::Pickup>> pick_action_client_;
  // std::shared_ptr<rclcpp_action::Client<moveit_msgs::action::Place>> place_action_client_;
  std::shared_ptr<rclcpp_action::Client<moveit_msgs::action::ExecuteTrajectory>> execute_action_client_;

  // general planning params
  moveit::core::RobotStatePtr considered_start_state_;
  moveit_msgs::msg::WorkspaceParameters workspace_parameters_;
  double allowed_planning_time_;
  std::string planning_pipeline_id_;
  std::string planner_id_;
  unsigned int num_planning_attempts_;
  double max_velocity_scaling_factor_;
  double max_acceleration_scaling_factor_;
  double goal_joint_tolerance_;
  double goal_position_tolerance_;
  double goal_orientation_tolerance_;
  bool can_look_;
  int32_t look_around_attempts_;
  bool can_replan_;
  int32_t replan_attempts_;
  double replan_delay_;

  // joint state goal
  moveit::core::RobotStatePtr joint_state_target_;
  const moveit::core::JointModelGroup* joint_model_group_;

  // pose goal;
  // for each link we have a set of possible goal locations;
  std::map<std::string, std::vector<geometry_msgs::msg::PoseStamped>> pose_targets_;

  // common properties for goals
  ActiveTargetType active_target_;
  std::unique_ptr<moveit_msgs::msg::Constraints> path_constraints_;
  std::unique_ptr<moveit_msgs::msg::TrajectoryConstraints> trajectory_constraints_;
  std::string end_effector_link_;
  std::string pose_reference_frame_;
  std::string support_surface_;

  // ROS communication
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trajectory_event_publisher_;
  rclcpp::Publisher<moveit_msgs::msg::AttachedCollisionObject>::SharedPtr attached_object_publisher_;
  rclcpp::Client<moveit_msgs::srv::QueryPlannerInterfaces>::SharedPtr query_service_;
  rclcpp::Client<moveit_msgs::srv::GetPlannerParams>::SharedPtr get_params_service_;
  rclcpp::Client<moveit_msgs::srv::SetPlannerParams>::SharedPtr set_params_service_;
  rclcpp::Client<moveit_msgs::srv::GetCartesianPath>::SharedPtr cartesian_path_service_;
  // rclcpp::Client<moveit_msgs::srv::GraspPlanning>::SharedPtr plan_grasps_service_;
  std::unique_ptr<moveit_warehouse::ConstraintsStorage> constraints_storage_;
  std::unique_ptr<std::thread> constraints_init_thread_;
  bool initializing_constraints_;
};

MoveGroupInterface::MoveGroupInterface(const rclcpp::Node::SharedPtr& node, const std::string& group_name,
                                       const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                                       const rclcpp::Duration& wait_for_servers)
  : logger_(moveit::getLogger("move_group_interface"))
{
  if (!rclcpp::ok())
    throw std::runtime_error("ROS does not seem to be running");
  impl_ =
      new MoveGroupInterfaceImpl(node, Options(group_name), tf_buffer ? tf_buffer : getSharedTF(), wait_for_servers);
}

MoveGroupInterface::MoveGroupInterface(const rclcpp::Node::SharedPtr& node, const Options& opt,
                                       const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                                       const rclcpp::Duration& wait_for_servers)
  : logger_(moveit::getLogger("move_group_interface"))
{
  impl_ = new MoveGroupInterfaceImpl(node, opt, tf_buffer ? tf_buffer : getSharedTF(), wait_for_servers);
}

MoveGroupInterface::~MoveGroupInterface()
{
  delete impl_;
}

MoveGroupInterface::MoveGroupInterface(MoveGroupInterface&& other) noexcept
  : remembered_joint_values_(std::move(other.remembered_joint_values_)), impl_(other.impl_), logger_(other.logger_)
{
  other.impl_ = nullptr;
}

MoveGroupInterface& MoveGroupInterface::operator=(MoveGroupInterface&& other) noexcept
{
  if (this != &other)
  {
    delete impl_;
    impl_ = other.impl_;
    logger_ = other.logger_;
    remembered_joint_values_ = std::move(other.remembered_joint_values_);
    other.impl_ = nullptr;
  }

  return *this;
}

const std::string& MoveGroupInterface::getName() const
{
  return impl_->getOptions().group_name;
}

const std::vector<std::string>& MoveGroupInterface::getNamedTargets() const
{
  // The pointer returned by getJointModelGroup is guaranteed by the class
  // constructor to always be non-null
  return impl_->getJointModelGroup()->getDefaultStateNames();
}

moveit::core::RobotModelConstPtr MoveGroupInterface::getRobotModel() const
{
  return impl_->getRobotModel();
}

bool MoveGroupInterface::getInterfaceDescription(moveit_msgs::msg::PlannerInterfaceDescription& desc) const
{
  return impl_->getInterfaceDescription(desc);
}

bool MoveGroupInterface::getInterfaceDescriptions(std::vector<moveit_msgs::msg::PlannerInterfaceDescription>& desc) const
{
  return impl_->getInterfaceDescriptions(desc);
}

std::map<std::string, std::string> MoveGroupInterface::getPlannerParams(const std::string& planner_id,
                                                                        const std::string& group) const
{
  return impl_->getPlannerParams(planner_id, group);
}

void MoveGroupInterface::setPlannerParams(const std::string& planner_id, const std::string& group,
                                          const std::map<std::string, std::string>& params, bool replace)
{
  impl_->setPlannerParams(planner_id, group, params, replace);
}

std::string MoveGroupInterface::getDefaultPlanningPipelineId() const
{
  return impl_->getDefaultPlanningPipelineId();
}

void MoveGroupInterface::setPlanningPipelineId(const std::string& pipeline_id)
{
  impl_->setPlanningPipelineId(pipeline_id);
}

const std::string& MoveGroupInterface::getPlanningPipelineId() const
{
  return impl_->getPlanningPipelineId();
}

std::string MoveGroupInterface::getDefaultPlannerId(const std::string& group) const
{
  return impl_->getDefaultPlannerId(group);
}

void MoveGroupInterface::setPlannerId(const std::string& planner_id)
{
  impl_->setPlannerId(planner_id);
}

const std::string& MoveGroupInterface::getPlannerId() const
{
  return impl_->getPlannerId();
}

void MoveGroupInterface::setNumPlanningAttempts(unsigned int num_planning_attempts)
{
  impl_->setNumPlanningAttempts(num_planning_attempts);
}

void MoveGroupInterface::setMaxVelocityScalingFactor(double max_velocity_scaling_factor)
{
  impl_->setMaxVelocityScalingFactor(max_velocity_scaling_factor);
}

void MoveGroupInterface::setMaxAccelerationScalingFactor(double max_acceleration_scaling_factor)
{
  impl_->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);
}

moveit::core::MoveItErrorCode MoveGroupInterface::asyncMove()
{
  return impl_->move(false);
}

rclcpp_action::Client<moveit_msgs::action::MoveGroup>& MoveGroupInterface::getMoveGroupClient() const
{
  return impl_->getMoveGroupClient();
}

moveit::core::MoveItErrorCode MoveGroupInterface::move()
{
  return impl_->move(true);
}

moveit::core::MoveItErrorCode MoveGroupInterface::asyncExecute(const Plan& plan,
                                                               const std::vector<std::string>& controllers)
{
  return impl_->execute(plan.trajectory, false, controllers);
}

moveit::core::MoveItErrorCode MoveGroupInterface::asyncExecute(const moveit_msgs::msg::RobotTrajectory& trajectory,
                                                               const std::vector<std::string>& controllers)
{
  return impl_->execute(trajectory, false, controllers);
}

moveit::core::MoveItErrorCode MoveGroupInterface::execute(const Plan& plan, const std::vector<std::string>& controllers)
{
  return impl_->execute(plan.trajectory, true, controllers);
}

moveit::core::MoveItErrorCode MoveGroupInterface::execute(const moveit_msgs::msg::RobotTrajectory& trajectory,
                                                          const std::vector<std::string>& controllers)
{
  return impl_->execute(trajectory, true, controllers);
}

moveit::core::MoveItErrorCode MoveGroupInterface::plan(Plan& plan)
{
  return impl_->plan(plan);
}

// moveit_msgs::action::Pickup::Goal MoveGroupInterface::constructPickupGoal(const std::string& object,
//                                                                        std::vector<moveit_msgs::msg::Grasp> grasps,
//                                                                        bool plan_only = false) const
//{
//  return impl_->constructPickupGoal(object, std::move(grasps), plan_only);
//}
//
// moveit_msgs::action::Place::Goal MoveGroupInterface::constructPlaceGoal(
//    const std::string& object, std::vector<moveit_msgs::msg::PlaceLocation> locations, bool plan_only = false) const
//{
//  return impl_->constructPlaceGoal(object, std::move(locations), plan_only);
//}
//
// std::vector<moveit_msgs::msg::PlaceLocation>
// MoveGroupInterface::posesToPlaceLocations(const std::vector<geometry_msgs::msg::PoseStamped>& poses) const
//{
//  return impl_->posesToPlaceLocations(poses);
//}
//
// moveit::core::MoveItErrorCode MoveGroupInterface::pick(const moveit_msgs::action::Pickup::Goal& goal)
//{
//  return impl_->pick(goal);
//}
//
// moveit::core::MoveItErrorCode MoveGroupInterface::planGraspsAndPick(const std::string& object, bool plan_only)
//{
//  return impl_->planGraspsAndPick(object, plan_only);
//}
//
// moveit::core::MoveItErrorCode MoveGroupInterface::planGraspsAndPick(const moveit_msgs::msg::CollisionObject& object,
// bool plan_only)
//{
//  return impl_->planGraspsAndPick(object, plan_only);
//}
//
// moveit::core::MoveItErrorCode MoveGroupInterface::place(const moveit_msgs::action::Place::Goal& goal)
//{
//  return impl_->place(goal);
//}

double MoveGroupInterface::computeCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints, double eef_step,
                                                double jump_threshold, moveit_msgs::msg::RobotTrajectory& trajectory,
                                                bool avoid_collisions, moveit_msgs::msg::MoveItErrorCodes* error_code)
{
  moveit_msgs::msg::Constraints path_constraints_tmp;
  return computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, moveit_msgs::msg::Constraints(),
                              avoid_collisions, error_code);
}

double MoveGroupInterface::computeCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints, double eef_step,
                                                double jump_threshold, moveit_msgs::msg::RobotTrajectory& trajectory,
                                                const moveit_msgs::msg::Constraints& path_constraints,
                                                bool avoid_collisions, moveit_msgs::msg::MoveItErrorCodes* error_code)
{
  if (error_code)
  {
    return impl_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, path_constraints,
                                       avoid_collisions, *error_code);
  }
  else
  {
    moveit_msgs::msg::MoveItErrorCodes err_tmp;
    err_tmp.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    moveit_msgs::msg::MoveItErrorCodes& err = error_code ? *error_code : err_tmp;
    return impl_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, path_constraints,
                                       avoid_collisions, err);
  }
}

void MoveGroupInterface::stop()
{
  impl_->stop();
}

void MoveGroupInterface::setStartState(const moveit_msgs::msg::RobotState& start_state)
{
  moveit::core::RobotStatePtr rs;
  if (start_state.is_diff)
  {
    impl_->getCurrentState(rs);
  }
  else
  {
    rs = std::make_shared<moveit::core::RobotState>(getRobotModel());
    rs->setToDefaultValues();  // initialize robot state values for conversion
  }
  moveit::core::robotStateMsgToRobotState(start_state, *rs);
  setStartState(*rs);
}

void MoveGroupInterface::setStartState(const moveit::core::RobotState& start_state)
{
  impl_->setStartState(start_state);
}

void MoveGroupInterface::setStartStateToCurrentState()
{
  impl_->setStartStateToCurrentState();
}

void MoveGroupInterface::setRandomTarget()
{
  impl_->getTargetRobotState().setToRandomPositions();
  impl_->setTargetType(JOINT);
}

const std::vector<std::string>& MoveGroupInterface::getJointNames() const
{
  return impl_->getJointModelGroup()->getVariableNames();
}

const std::vector<std::string>& MoveGroupInterface::getLinkNames() const
{
  return impl_->getJointModelGroup()->getLinkModelNames();
}

std::map<std::string, double> MoveGroupInterface::getNamedTargetValues(const std::string& name) const
{
  std::map<std::string, std::vector<double>>::const_iterator it = remembered_joint_values_.find(name);
  std::map<std::string, double> positions;

  if (it != remembered_joint_values_.cend())
  {
    std::vector<std::string> names = impl_->getJointModelGroup()->getVariableNames();
    for (size_t x = 0; x < names.size(); ++x)
    {
      positions[names[x]] = it->second[x];
    }
  }
  else
  {
    if (!impl_->getJointModelGroup()->getVariableDefaultPositions(name, positions))
    {
      RCLCPP_ERROR(logger_, "The requested named target '%s' does not exist, returning empty positions.", name.c_str());
    }
  }
  return positions;
}

bool MoveGroupInterface::setNamedTarget(const std::string& name)
{
  std::map<std::string, std::vector<double>>::const_iterator it = remembered_joint_values_.find(name);
  if (it != remembered_joint_values_.end())
  {
    return setJointValueTarget(it->second);
  }
  else
  {
    if (impl_->getTargetRobotState().setToDefaultValues(impl_->getJointModelGroup(), name))
    {
      impl_->setTargetType(JOINT);
      return true;
    }
    RCLCPP_ERROR(logger_, "The requested named target '%s' does not exist", name.c_str());
    return false;
  }
}

void MoveGroupInterface::getJointValueTarget(std::vector<double>& group_variable_values) const
{
  impl_->getTargetRobotState().copyJointGroupPositions(impl_->getJointModelGroup(), group_variable_values);
}

bool MoveGroupInterface::setJointValueTarget(const std::vector<double>& joint_values)
{
  const auto n_group_joints = impl_->getJointModelGroup()->getVariableCount();
  if (joint_values.size() != n_group_joints)
  {
    RCLCPP_DEBUG_STREAM(logger_, "Provided joint value list has length " << joint_values.size() << " but group "
                                                                         << impl_->getJointModelGroup()->getName()
                                                                         << " has " << n_group_joints << " joints");
    return false;
  }
  impl_->setTargetType(JOINT);
  impl_->getTargetRobotState().setJointGroupPositions(impl_->getJointModelGroup(), joint_values);
  return impl_->getTargetRobotState().satisfiesBounds(impl_->getJointModelGroup(), impl_->getGoalJointTolerance());
}

bool MoveGroupInterface::setJointValueTarget(const std::map<std::string, double>& variable_values)
{
  const auto& allowed = impl_->getJointModelGroup()->getVariableNames();
  for (const auto& pair : variable_values)
  {
    if (std::find(allowed.begin(), allowed.end(), pair.first) == allowed.end())
    {
      RCLCPP_ERROR_STREAM(logger_, "joint variable " << pair.first << " is not part of group "
                                                     << impl_->getJointModelGroup()->getName());
      return false;
    }
  }

  impl_->setTargetType(JOINT);
  impl_->getTargetRobotState().setVariablePositions(variable_values);
  return impl_->getTargetRobotState().satisfiesBounds(impl_->getGoalJointTolerance());
}

bool MoveGroupInterface::setJointValueTarget(const std::vector<std::string>& variable_names,
                                             const std::vector<double>& variable_values)
{
  if (variable_names.size() != variable_values.size())
  {
    RCLCPP_ERROR_STREAM(logger_, "sizes of name and position arrays do not match");
    return false;
  }
  const auto& allowed = impl_->getJointModelGroup()->getVariableNames();
  for (const auto& variable_name : variable_names)
  {
    if (std::find(allowed.begin(), allowed.end(), variable_name) == allowed.end())
    {
      RCLCPP_ERROR_STREAM(logger_, "joint variable " << variable_name << " is not part of group "
                                                     << impl_->getJointModelGroup()->getName());
      return false;
    }
  }

  impl_->setTargetType(JOINT);
  impl_->getTargetRobotState().setVariablePositions(variable_names, variable_values);
  return impl_->getTargetRobotState().satisfiesBounds(impl_->getGoalJointTolerance());
}

bool MoveGroupInterface::setJointValueTarget(const moveit::core::RobotState& rstate)
{
  impl_->setTargetType(JOINT);
  impl_->getTargetRobotState() = rstate;
  return impl_->getTargetRobotState().satisfiesBounds(impl_->getGoalJointTolerance());
}

bool MoveGroupInterface::setJointValueTarget(const std::string& joint_name, double value)
{
  std::vector<double> values(1, value);
  return setJointValueTarget(joint_name, values);
}

bool MoveGroupInterface::setJointValueTarget(const std::string& joint_name, const std::vector<double>& values)
{
  impl_->setTargetType(JOINT);
  const moveit::core::JointModel* jm = impl_->getJointModelGroup()->getJointModel(joint_name);
  if (jm && jm->getVariableCount() == values.size())
  {
    impl_->getTargetRobotState().setJointPositions(jm, values);
    return impl_->getTargetRobotState().satisfiesBounds(jm, impl_->getGoalJointTolerance());
  }

  RCLCPP_ERROR_STREAM(logger_,
                      "joint " << joint_name << " is not part of group " << impl_->getJointModelGroup()->getName());
  return false;
}

bool MoveGroupInterface::setJointValueTarget(const sensor_msgs::msg::JointState& state)
{
  return setJointValueTarget(state.name, state.position);
}

bool MoveGroupInterface::setJointValueTarget(const geometry_msgs::msg::Pose& eef_pose,
                                             const std::string& end_effector_link)
{
  return impl_->setJointValueTarget(eef_pose, end_effector_link, "", false);
}

bool MoveGroupInterface::setJointValueTarget(const geometry_msgs::msg::PoseStamped& eef_pose,
                                             const std::string& end_effector_link)
{
  return impl_->setJointValueTarget(eef_pose.pose, end_effector_link, eef_pose.header.frame_id, false);
}

bool MoveGroupInterface::setJointValueTarget(const Eigen::Isometry3d& eef_pose, const std::string& end_effector_link)
{
  geometry_msgs::msg::Pose msg = tf2::toMsg(eef_pose);
  return setJointValueTarget(msg, end_effector_link);
}

bool MoveGroupInterface::setApproximateJointValueTarget(const geometry_msgs::msg::Pose& eef_pose,
                                                        const std::string& end_effector_link)
{
  return impl_->setJointValueTarget(eef_pose, end_effector_link, "", true);
}

bool MoveGroupInterface::setApproximateJointValueTarget(const geometry_msgs::msg::PoseStamped& eef_pose,
                                                        const std::string& end_effector_link)
{
  return impl_->setJointValueTarget(eef_pose.pose, end_effector_link, eef_pose.header.frame_id, true);
}

bool MoveGroupInterface::setApproximateJointValueTarget(const Eigen::Isometry3d& eef_pose,
                                                        const std::string& end_effector_link)
{
  geometry_msgs::msg::Pose msg = tf2::toMsg(eef_pose);
  return setApproximateJointValueTarget(msg, end_effector_link);
}

const moveit::core::RobotState& MoveGroupInterface::getTargetRobotState() const
{
  return impl_->getTargetRobotState();
}

const std::string& MoveGroupInterface::getEndEffectorLink() const
{
  return impl_->getEndEffectorLink();
}

const std::string& MoveGroupInterface::getEndEffector() const
{
  return impl_->getEndEffector();
}

bool MoveGroupInterface::setEndEffectorLink(const std::string& link_name)
{
  if (impl_->getEndEffectorLink().empty() || link_name.empty())
    return false;
  impl_->setEndEffectorLink(link_name);
  impl_->setTargetType(POSE);
  return true;
}

bool MoveGroupInterface::setEndEffector(const std::string& eef_name)
{
  const moveit::core::JointModelGroup* jmg = impl_->getRobotModel()->getEndEffector(eef_name);
  if (jmg)
    return setEndEffectorLink(jmg->getEndEffectorParentGroup().second);
  return false;
}

void MoveGroupInterface::clearPoseTarget(const std::string& end_effector_link)
{
  impl_->clearPoseTarget(end_effector_link);
}

void MoveGroupInterface::clearPoseTargets()
{
  impl_->clearPoseTargets();
}

bool MoveGroupInterface::setPoseTarget(const Eigen::Isometry3d& pose, const std::string& end_effector_link)
{
  std::vector<geometry_msgs::msg::PoseStamped> pose_msg(1);
  pose_msg[0].pose = tf2::toMsg(pose);
  pose_msg[0].header.frame_id = getPoseReferenceFrame();
  pose_msg[0].header.stamp = impl_->getClock()->now();
  return setPoseTargets(pose_msg, end_effector_link);
}

bool MoveGroupInterface::setPoseTarget(const geometry_msgs::msg::Pose& target, const std::string& end_effector_link)
{
  std::vector<geometry_msgs::msg::PoseStamped> pose_msg(1);
  pose_msg[0].pose = target;
  pose_msg[0].header.frame_id = getPoseReferenceFrame();
  pose_msg[0].header.stamp = impl_->getClock()->now();
  return setPoseTargets(pose_msg, end_effector_link);
}

bool MoveGroupInterface::setPoseTarget(const geometry_msgs::msg::PoseStamped& target,
                                       const std::string& end_effector_link)
{
  std::vector<geometry_msgs::msg::PoseStamped> targets(1, target);
  return setPoseTargets(targets, end_effector_link);
}

bool MoveGroupInterface::setPoseTargets(const EigenSTL::vector_Isometry3d& target, const std::string& end_effector_link)
{
  std::vector<geometry_msgs::msg::PoseStamped> pose_out(target.size());
  rclcpp::Time tm = impl_->getClock()->now();
  const std::string& frame_id = getPoseReferenceFrame();
  for (std::size_t i = 0; i < target.size(); ++i)
  {
    pose_out[i].pose = tf2::toMsg(target[i]);
    pose_out[i].header.stamp = tm;
    pose_out[i].header.frame_id = frame_id;
  }
  return setPoseTargets(pose_out, end_effector_link);
}

bool MoveGroupInterface::setPoseTargets(const std::vector<geometry_msgs::msg::Pose>& target,
                                        const std::string& end_effector_link)
{
  std::vector<geometry_msgs::msg::PoseStamped> target_stamped(target.size());
  rclcpp::Time tm = impl_->getClock()->now();
  const std::string& frame_id = getPoseReferenceFrame();
  for (std::size_t i = 0; i < target.size(); ++i)
  {
    target_stamped[i].pose = target[i];
    target_stamped[i].header.stamp = tm;
    target_stamped[i].header.frame_id = frame_id;
  }
  return setPoseTargets(target_stamped, end_effector_link);
}

bool MoveGroupInterface::setPoseTargets(const std::vector<geometry_msgs::msg::PoseStamped>& target,
                                        const std::string& end_effector_link)
{
  if (target.empty())
  {
    RCLCPP_ERROR(logger_, "No pose specified as goal target");
    return false;
  }
  else
  {
    impl_->setTargetType(POSE);
    return impl_->setPoseTargets(target, end_effector_link);
  }
}

const geometry_msgs::msg::PoseStamped& MoveGroupInterface::getPoseTarget(const std::string& end_effector_link) const
{
  return impl_->getPoseTarget(end_effector_link);
}

const std::vector<geometry_msgs::msg::PoseStamped>&
MoveGroupInterface::getPoseTargets(const std::string& end_effector_link) const
{
  return impl_->getPoseTargets(end_effector_link);
}

namespace
{
inline void transformPose(const tf2_ros::Buffer& tf_buffer, const std::string& desired_frame,
                          geometry_msgs::msg::PoseStamped& target)
{
  if (desired_frame != target.header.frame_id)
  {
    geometry_msgs::msg::PoseStamped target_in(target);
    tf_buffer.transform(target_in, target, desired_frame);
    // we leave the stamp to ros::Time(0) on purpose
    target.header.stamp = rclcpp::Time(0);
  }
}
}  // namespace

bool MoveGroupInterface::setPositionTarget(double x, double y, double z, const std::string& end_effector_link)
{
  geometry_msgs::msg::PoseStamped target;
  if (impl_->hasPoseTarget(end_effector_link))
  {
    target = getPoseTarget(end_effector_link);
    transformPose(*impl_->getTF(), impl_->getPoseReferenceFrame(), target);
  }
  else
  {
    target.pose.orientation.x = 0.0;
    target.pose.orientation.y = 0.0;
    target.pose.orientation.z = 0.0;
    target.pose.orientation.w = 1.0;
    target.header.frame_id = impl_->getPoseReferenceFrame();
  }

  target.pose.position.x = x;
  target.pose.position.y = y;
  target.pose.position.z = z;
  bool result = setPoseTarget(target, end_effector_link);
  impl_->setTargetType(POSITION);
  return result;
}

bool MoveGroupInterface::setRPYTarget(double r, double p, double y, const std::string& end_effector_link)
{
  geometry_msgs::msg::PoseStamped target;
  if (impl_->hasPoseTarget(end_effector_link))
  {
    target = getPoseTarget(end_effector_link);
    transformPose(*impl_->getTF(), impl_->getPoseReferenceFrame(), target);
  }
  else
  {
    target.pose.position.x = 0.0;
    target.pose.position.y = 0.0;
    target.pose.position.z = 0.0;
    target.header.frame_id = impl_->getPoseReferenceFrame();
  }
  tf2::Quaternion q;
  q.setRPY(r, p, y);
  target.pose.orientation = tf2::toMsg(q);
  bool result = setPoseTarget(target, end_effector_link);
  impl_->setTargetType(ORIENTATION);
  return result;
}

bool MoveGroupInterface::setOrientationTarget(double x, double y, double z, double w,
                                              const std::string& end_effector_link)
{
  geometry_msgs::msg::PoseStamped target;
  if (impl_->hasPoseTarget(end_effector_link))
  {
    target = getPoseTarget(end_effector_link);
    transformPose(*impl_->getTF(), impl_->getPoseReferenceFrame(), target);
  }
  else
  {
    target.pose.position.x = 0.0;
    target.pose.position.y = 0.0;
    target.pose.position.z = 0.0;
    target.header.frame_id = impl_->getPoseReferenceFrame();
  }

  target.pose.orientation.x = x;
  target.pose.orientation.y = y;
  target.pose.orientation.z = z;
  target.pose.orientation.w = w;
  bool result = setPoseTarget(target, end_effector_link);
  impl_->setTargetType(ORIENTATION);
  return result;
}

void MoveGroupInterface::setPoseReferenceFrame(const std::string& pose_reference_frame)
{
  impl_->setPoseReferenceFrame(pose_reference_frame);
}

const std::string& MoveGroupInterface::getPoseReferenceFrame() const
{
  return impl_->getPoseReferenceFrame();
}

double MoveGroupInterface::getGoalJointTolerance() const
{
  return impl_->getGoalJointTolerance();
}

double MoveGroupInterface::getGoalPositionTolerance() const
{
  return impl_->getGoalPositionTolerance();
}

double MoveGroupInterface::getGoalOrientationTolerance() const
{
  return impl_->getGoalOrientationTolerance();
}

void MoveGroupInterface::setGoalTolerance(double tolerance)
{
  setGoalJointTolerance(tolerance);
  setGoalPositionTolerance(tolerance);
  setGoalOrientationTolerance(tolerance);
}

void MoveGroupInterface::setGoalJointTolerance(double tolerance)
{
  impl_->setGoalJointTolerance(tolerance);
}

void MoveGroupInterface::setGoalPositionTolerance(double tolerance)
{
  impl_->setGoalPositionTolerance(tolerance);
}

void MoveGroupInterface::setGoalOrientationTolerance(double tolerance)
{
  impl_->setGoalOrientationTolerance(tolerance);
}

void MoveGroupInterface::rememberJointValues(const std::string& name)
{
  rememberJointValues(name, getCurrentJointValues());
}

bool MoveGroupInterface::startStateMonitor(double wait)
{
  return impl_->startStateMonitor(wait);
}

std::vector<double> MoveGroupInterface::getCurrentJointValues() const
{
  moveit::core::RobotStatePtr current_state;
  std::vector<double> values;
  if (impl_->getCurrentState(current_state))
    current_state->copyJointGroupPositions(getName(), values);
  return values;
}

std::vector<double> MoveGroupInterface::getRandomJointValues() const
{
  std::vector<double> r;
  impl_->getJointModelGroup()->getVariableRandomPositions(impl_->getTargetRobotState().getRandomNumberGenerator(), r);
  return r;
}

geometry_msgs::msg::PoseStamped MoveGroupInterface::getRandomPose(const std::string& end_effector_link) const
{
  const std::string& eef = end_effector_link.empty() ? getEndEffectorLink() : end_effector_link;
  Eigen::Isometry3d pose;
  pose.setIdentity();
  if (eef.empty())
  {
    RCLCPP_ERROR(logger_, "No end-effector specified");
  }
  else
  {
    moveit::core::RobotStatePtr current_state;
    if (impl_->getCurrentState(current_state))
    {
      current_state->setToRandomPositions(impl_->getJointModelGroup());
      const moveit::core::LinkModel* lm = current_state->getLinkModel(eef);
      if (lm)
        pose = current_state->getGlobalLinkTransform(lm);
    }
  }
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = impl_->getClock()->now();
  pose_msg.header.frame_id = impl_->getRobotModel()->getModelFrame();
  pose_msg.pose = tf2::toMsg(pose);
  return pose_msg;
}

geometry_msgs::msg::PoseStamped MoveGroupInterface::getCurrentPose(const std::string& end_effector_link) const
{
  const std::string& eef = end_effector_link.empty() ? getEndEffectorLink() : end_effector_link;
  Eigen::Isometry3d pose;
  pose.setIdentity();
  if (eef.empty())
  {
    RCLCPP_ERROR(logger_, "No end-effector specified");
  }
  else
  {
    moveit::core::RobotStatePtr current_state;
    if (impl_->getCurrentState(current_state))
    {
      const moveit::core::LinkModel* lm = current_state->getLinkModel(eef);
      if (lm)
        pose = current_state->getGlobalLinkTransform(lm);
    }
  }
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = impl_->getClock()->now();
  pose_msg.header.frame_id = impl_->getRobotModel()->getModelFrame();
  pose_msg.pose = tf2::toMsg(pose);
  return pose_msg;
}

std::vector<double> MoveGroupInterface::getCurrentRPY(const std::string& end_effector_link) const
{
  std::vector<double> result;
  const std::string& eef = end_effector_link.empty() ? getEndEffectorLink() : end_effector_link;
  if (eef.empty())
  {
    RCLCPP_ERROR(logger_, "No end-effector specified");
  }
  else
  {
    moveit::core::RobotStatePtr current_state;
    if (impl_->getCurrentState(current_state))
    {
      const moveit::core::LinkModel* lm = current_state->getLinkModel(eef);
      if (lm)
      {
        result.resize(3);
        geometry_msgs::msg::TransformStamped tfs = tf2::eigenToTransform(current_state->getGlobalLinkTransform(lm));
        double pitch, roll, yaw;
        tf2::getEulerYPR<geometry_msgs::msg::Quaternion>(tfs.transform.rotation, yaw, pitch, roll);
        result[0] = roll;
        result[1] = pitch;
        result[2] = yaw;
      }
    }
  }
  return result;
}

const std::vector<std::string>& MoveGroupInterface::getActiveJoints() const
{
  return impl_->getJointModelGroup()->getActiveJointModelNames();
}

const std::vector<std::string>& MoveGroupInterface::getJoints() const
{
  return impl_->getJointModelGroup()->getJointModelNames();
}

unsigned int MoveGroupInterface::getVariableCount() const
{
  return impl_->getJointModelGroup()->getVariableCount();
}

moveit::core::RobotStatePtr MoveGroupInterface::getCurrentState(double wait) const
{
  moveit::core::RobotStatePtr current_state;
  impl_->getCurrentState(current_state, wait);
  return current_state;
}

void MoveGroupInterface::rememberJointValues(const std::string& name, const std::vector<double>& values)
{
  remembered_joint_values_[name] = values;
}

void MoveGroupInterface::forgetJointValues(const std::string& name)
{
  remembered_joint_values_.erase(name);
}

void MoveGroupInterface::allowLooking(bool flag)
{
  impl_->can_look_ = flag;
  RCLCPP_DEBUG(logger_, "Looking around: %s", flag ? "yes" : "no");
}

void MoveGroupInterface::setLookAroundAttempts(int32_t attempts)
{
  if (attempts < 0)
  {
    RCLCPP_ERROR(logger_, "Tried to set negative number of look-around attempts");
  }
  else
  {
    RCLCPP_DEBUG_STREAM(logger_, "Look around attempts: " << attempts);
    impl_->look_around_attempts_ = attempts;
  }
}

void MoveGroupInterface::allowReplanning(bool flag)
{
  impl_->can_replan_ = flag;
  RCLCPP_DEBUG(logger_, "Replanning: %s", flag ? "yes" : "no");
}

void MoveGroupInterface::setReplanAttempts(int32_t attempts)
{
  if (attempts < 0)
  {
    RCLCPP_ERROR(logger_, "Tried to set negative number of replan attempts");
  }
  else
  {
    RCLCPP_DEBUG_STREAM(logger_, "Replan Attempts: " << attempts);
    impl_->replan_attempts_ = attempts;
  }
}

void MoveGroupInterface::setReplanDelay(double delay)
{
  if (delay < 0.0)
  {
    RCLCPP_ERROR(logger_, "Tried to set negative replan delay");
  }
  else
  {
    RCLCPP_DEBUG_STREAM(logger_, "Replan Delay: " << delay);
    impl_->replan_delay_ = delay;
  }
}

std::vector<std::string> MoveGroupInterface::getKnownConstraints() const
{
  return impl_->getKnownConstraints();
}

moveit_msgs::msg::Constraints MoveGroupInterface::getPathConstraints() const
{
  return impl_->getPathConstraints();
}

bool MoveGroupInterface::setPathConstraints(const std::string& constraint)
{
  return impl_->setPathConstraints(constraint);
}

void MoveGroupInterface::setPathConstraints(const moveit_msgs::msg::Constraints& constraint)
{
  impl_->setPathConstraints(constraint);
}

void MoveGroupInterface::clearPathConstraints()
{
  impl_->clearPathConstraints();
}

moveit_msgs::msg::TrajectoryConstraints MoveGroupInterface::getTrajectoryConstraints() const
{
  return impl_->getTrajectoryConstraints();
}

void MoveGroupInterface::setTrajectoryConstraints(const moveit_msgs::msg::TrajectoryConstraints& constraint)
{
  impl_->setTrajectoryConstraints(constraint);
}

void MoveGroupInterface::clearTrajectoryConstraints()
{
  impl_->clearTrajectoryConstraints();
}

void MoveGroupInterface::setConstraintsDatabase(const std::string& host, unsigned int port)
{
  impl_->initializeConstraintsStorage(host, port);
}

void MoveGroupInterface::setWorkspace(double minx, double miny, double minz, double maxx, double maxy, double maxz)
{
  impl_->setWorkspace(minx, miny, minz, maxx, maxy, maxz);
}

/** \brief Set time allowed to planner to solve problem before aborting */
void MoveGroupInterface::setPlanningTime(double seconds)
{
  impl_->setPlanningTime(seconds);
}

/** \brief Get time allowed to planner to solve problem before aborting */
double MoveGroupInterface::getPlanningTime() const
{
  return impl_->getPlanningTime();
}

void MoveGroupInterface::setSupportSurfaceName(const std::string& name)
{
  impl_->setSupportSurfaceName(name);
}

const std::string& MoveGroupInterface::getPlanningFrame() const
{
  return impl_->getRobotModel()->getModelFrame();
}

const std::vector<std::string>& MoveGroupInterface::getJointModelGroupNames() const
{
  return impl_->getRobotModel()->getJointModelGroupNames();
}

bool MoveGroupInterface::attachObject(const std::string& object, const std::string& link)
{
  return attachObject(object, link, std::vector<std::string>());
}

bool MoveGroupInterface::attachObject(const std::string& object, const std::string& link,
                                      const std::vector<std::string>& touch_links)
{
  return impl_->attachObject(object, link, touch_links);
}

bool MoveGroupInterface::detachObject(const std::string& name)
{
  return impl_->detachObject(name);
}

void MoveGroupInterface::constructMotionPlanRequest(moveit_msgs::msg::MotionPlanRequest& goal_out)
{
  impl_->constructMotionPlanRequest(goal_out);
}

}  // namespace planning_interface
}  // namespace moveit
