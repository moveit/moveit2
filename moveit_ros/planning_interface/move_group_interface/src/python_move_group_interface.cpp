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

/* Author: Siyuan Lu, Ioan Sucan */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/py_bindings_tools/py_conversions.h>
#include <moveit/py_bindings_tools/serialize_msg.h>
#include <moveit/py_bindings_tools/gil_releaser.h>
#include <moveit/utils/moveit_error_code.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
#include <tf2/LinearMath/Quaternion.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <tf2_ros/buffer.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <memory>
#include <stdexcept>

/** @cond IGNORE */

namespace py = pybind11;

using moveit::py_bindings_tools::GILReleaser;

namespace moveit
{
namespace planning_interface
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("python_move_group_interface");

class MoveGroupInterfacePython : public MoveGroupInterface
{
public:
  MoveGroupInterfacePython(const std::string& move_group_node_name, const std::string& group_name,
                           const std::string& robot_description)
  {
    if (!rclcpp::ok())
    {
      throw std::runtime_error("ROS does not seem to be running");
    }

    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.arguments(
        { "--ros-args", "-r",
          "__node:=" + std::string("move_group_interface_") + std::to_string(reinterpret_cast<std::size_t>(this)) });
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("_", options);

    // need to load robot_description, robot_description_semantic and kinematics_solvers parameters for robot_model_loader
    auto param_source = std::make_shared<rclcpp::SyncParametersClient>(node, move_group_node_name);
    while (!param_source->service_is_ready()) {};
    const std::string robot_description_semantic = robot_description + "_semantic";
    std::vector<std::string> interface_params = { robot_description, robot_description_semantic };
    rcl_interfaces::msg::ListParametersResult result =
        param_source->list_parameters({ "robot_description_kinematics" }, 3);
    auto kinematics_keys = result.names;
    for (auto key : kinematics_keys)
    {
      interface_params.push_back(key);
    }

    auto parameters = param_source->get_parameters(interface_params);
    node->set_parameters(parameters);

    private_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    private_executor_->add_node(node);
    // start executor on a different thread now
    private_executor_thread_ = std::thread([this]() { private_executor_->spin(); });
    new (this) MoveGroupInterfacePython(node, group_name, robot_description);
  }

  MoveGroupInterfacePython(const rclcpp::Node::SharedPtr& node, const std::string& group_name,
                           const std::string& robot_description, double wait_for_servers = 5.0)
    : MoveGroupInterface(node, Options(group_name, robot_description), std::shared_ptr<tf2_ros::Buffer>(),
                         rclcpp::Duration::from_seconds(wait_for_servers))
  {
  }

  bool setJointValueTargetPerJointPythonList(const std::string& joint, py::list& values)
  {
    return setJointValueTarget(joint, py_bindings_tools::doubleFromList(values));
  }

  bool setJointValueTargetPythonIterable(py::object& values)
  {
    return setJointValueTarget(py_bindings_tools::doubleFromList(values));
  }

  bool setJointValueTargetPythonDict(py::dict& values)
  {
    std::map<std::string, double> v;
    for (auto it : values)
      v[py::str(it.first)] = (it.second).cast<double>();
    return setJointValueTarget(v);
  }

  bool setJointValueTargetFromPosePython(const py::bytes& pose_str, const std::string& eef, bool approx)
  {
    geometry_msgs::msg::Pose pose_msg;
    py_bindings_tools::deserializeMsg(pose_str, pose_msg);
    return approx ? setApproximateJointValueTarget(pose_msg, eef) : setJointValueTarget(pose_msg, eef);
  }

  bool setJointValueTargetFromPoseStampedPython(const py::bytes& pose_str, const std::string& eef, bool approx)
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    py_bindings_tools::deserializeMsg(pose_str, pose_msg);
    return approx ? setApproximateJointValueTarget(pose_msg, eef) : setJointValueTarget(pose_msg, eef);
  }

  bool setJointValueTargetFromJointStatePython(const py::bytes& js_str)
  {
    sensor_msgs::msg::JointState js_msg;
    py_bindings_tools::deserializeMsg(js_str, js_msg);
    return setJointValueTarget(js_msg);
  }

  py::list getJointValueTargetPythonList()
  {
    std::vector<double> values;
    MoveGroupInterface::getJointValueTarget(values);
    py::list l;
    for (const double value : values)
      l.append(value);
    return l;
  }

  py::bytes getJointValueTarget()
  {
    moveit_msgs::msg::RobotState msg;
    const moveit::core::RobotState state = moveit::planning_interface::MoveGroupInterface::getTargetRobotState();
    moveit::core::robotStateToRobotStateMsg(state, msg);
    return py_bindings_tools::serializeMsg(msg);
  }

  void rememberJointValuesFromPythonList(const std::string& string, py::list& values)
  {
    rememberJointValues(string, py_bindings_tools::doubleFromList(values));
  }

  const char* getPlanningFrameCStr() const
  {
    return getPlanningFrame().c_str();
  }

  py::bytes getInterfaceDescriptionPython()
  {
    moveit_msgs::msg::PlannerInterfaceDescription msg;
    getInterfaceDescription(msg);
    return py_bindings_tools::serializeMsg(msg);
  }

  py::list getActiveJointsList() const
  {
    return py_bindings_tools::listFromString(getActiveJoints());
  }

  py::list getJointsList() const
  {
    return py_bindings_tools::listFromString(getJoints());
  }

  py::list getCurrentJointValuesList()
  {
    return py_bindings_tools::listFromDouble(getCurrentJointValues());
  }

  py::list getRandomJointValuesList()
  {
    return py_bindings_tools::listFromDouble(getRandomJointValues());
  }

  py::dict getRememberedJointValuesPython() const
  {
    const std::map<std::string, std::vector<double>>& rv = getRememberedJointValues();
    py::dict d;
    for (const std::pair<const std::string, std::vector<double>>& it : rv)
      d[py::str(it.first)] = py_bindings_tools::listFromDouble(it.second);
    return d;
  }

  py::list convertPoseToList(const geometry_msgs::msg::Pose& pose) const
  {
    std::vector<double> v(7);
    v[0] = pose.position.x;
    v[1] = pose.position.y;
    v[2] = pose.position.z;
    v[3] = pose.orientation.x;
    v[4] = pose.orientation.y;
    v[5] = pose.orientation.z;
    v[6] = pose.orientation.w;
    return py_bindings_tools::listFromDouble(v);
  }

  py::list convertTransformToList(const geometry_msgs::msg::Transform& tr) const
  {
    std::vector<double> v(7);
    v[0] = tr.translation.x;
    v[1] = tr.translation.y;
    v[2] = tr.translation.z;
    v[3] = tr.rotation.x;
    v[4] = tr.rotation.y;
    v[5] = tr.rotation.z;
    v[6] = tr.rotation.w;
    return py_bindings_tools::listFromDouble(v);
  }

  void convertListToTransform(const py::list& l, geometry_msgs::msg::Transform& tr) const
  {
    std::vector<double> v = py_bindings_tools::doubleFromList(l);
    tr.translation.x = v[0];
    tr.translation.y = v[1];
    tr.translation.z = v[2];
    tr.rotation.x = v[3];
    tr.rotation.y = v[4];
    tr.rotation.z = v[5];
    tr.rotation.w = v[6];
  }

  void convertListToPose(const py::list& l, geometry_msgs::msg::Pose& p) const
  {
    std::vector<double> v = py_bindings_tools::doubleFromList(l);
    p.position.x = v[0];
    p.position.y = v[1];
    p.position.z = v[2];
    p.orientation.x = v[3];
    p.orientation.y = v[4];
    p.orientation.z = v[5];
    p.orientation.w = v[6];
  }

  py::list getCurrentRPYPython(const std::string& end_effector_link = "")
  {
    return py_bindings_tools::listFromDouble(getCurrentRPY(end_effector_link));
  }

  py::list getCurrentPosePython(const std::string& end_effector_link = "")
  {
    geometry_msgs::msg::PoseStamped pose = getCurrentPose(end_effector_link);
    return convertPoseToList(pose.pose);
  }

  py::list getRandomPosePython(const std::string& end_effector_link = "")
  {
    geometry_msgs::msg::PoseStamped pose = getRandomPose(end_effector_link);
    return convertPoseToList(pose.pose);
  }

  py::list getKnownConstraintsList() const
  {
    return py_bindings_tools::listFromString(getKnownConstraints());
  }

  // bool placePose(const std::string& object_name, const py::list& pose, bool plan_only = false)
  // {
  //   geometry_msgs::msg::PoseStamped msg;
  //   convertListToPose(pose, msg.pose);
  //   msg.header.frame_id = getPoseReferenceFrame();
  //   msg.header.stamp = ros::Time::now();
  //   GILReleaser gr;
  //   return place(object_name, msg, plan_only) == moveit::core::MoveItErrorCode::SUCCESS;
  // }

  // bool placePoses(const std::string& object_name, const py::list& poses_list, bool plan_only = false)
  // {
  //   int l = py::len(poses_list);
  //   std::vector<geometry_msgs::msg::PoseStamped> poses(l);
  //   for (int i = 0; i < l; ++i)
  //     py_bindings_tools::deserializeMsg(py::bytes(poses_list[i]), poses[i]);
  //   GILReleaser gr;
  //   return place(object_name, poses, plan_only) == moveit::core::MoveItErrorCode::SUCCESS;
  // }

  // bool placeLocation(const std::string& object_name, const py::bytes& location_str, bool plan_only = false)
  // {
  //   std::vector<moveit_msgs::action::PlaceLocation> locations(1);
  //   py_bindings_tools::deserializeMsg(location_str, locations[0]);
  //   GILReleaser gr;
  //   return place(object_name, std::move(locations), plan_only) == moveit::core::MoveItErrorCode::SUCCESS;
  // }

  // bool placeLocations(const std::string& object_name, const py::list& location_list, bool plan_only = false)
  // {
  //   int l = py::len(location_list);
  //   std::vector<moveit_msgs::PlaceLocation> locations(l);
  //   for (int i = 0; i < l; ++i)
  //     py_bindings_tools::deserializeMsg(py::bytes(location_list[i]), locations[i]);
  //   GILReleaser gr;
  //   return place(object_name, std::move(locations), plan_only) == moveit::core::MoveItErrorCode::SUCCESS;
  // }

  // bool placeAnywhere(const std::string& object_name, bool plan_only = false)
  // {
  //   GILReleaser gr;
  //   return place(object_name, plan_only) == moveit::core::MoveItErrorCode::SUCCESS;
  // }

  void convertListToArrayOfPoses(const py::list& poses, std::vector<geometry_msgs::msg::Pose>& msg)
  {
    int l = py::len(poses);
    for (int i = 0; i < l; ++i)
    {
      const py::list& pose = (poses[i]).cast<py::list>();
      std::vector<double> v = py_bindings_tools::doubleFromList(pose);
      if (v.size() == 6 || v.size() == 7)
      {
        Eigen::Isometry3d p;
        if (v.size() == 6)
        {
          tf2::Quaternion tq;
          tq.setRPY(v[3], v[4], v[5]);
          p = Eigen::Isometry3d(Eigen::Quaterniond(tq.w(), tq.x(), tq.y(), tq.z()));
        }
        else
          p = Eigen::Isometry3d(Eigen::Quaterniond(v[6], v[3], v[4], v[5]));
        p.translation() = Eigen::Vector3d(v[0], v[1], v[2]);
        geometry_msgs::msg::Pose pm = tf2::toMsg(p);
        msg.push_back(pm);
      }
      else
        RCLCPP_WARN(LOGGER, "Incorrect number of values for a pose: %u", (unsigned int)v.size());
    }
  }

  py::dict getCurrentStateBoundedPython()
  {
    moveit::core::RobotStatePtr current = getCurrentState();
    current->enforceBounds();
    moveit_msgs::msg::RobotState rsmv;
    moveit::core::robotStateToRobotStateMsg(*current, rsmv);
    py::dict output;
    for (size_t x = 0; x < rsmv.joint_state.name.size(); ++x)
      output[py::str(rsmv.joint_state.name[x])] = rsmv.joint_state.position[x];
    return output;
  }

  py::bytes getCurrentStatePython(double wait = 1.0)
  {
    moveit::core::RobotStatePtr current_state = getCurrentState(wait);
    moveit_msgs::msg::RobotState state_message;
    if (NULL == current_state)
      return py::none();
    moveit::core::robotStateToRobotStateMsg(*current_state, state_message);
    return py_bindings_tools::serializeMsg(state_message);
  }

  void setStartStatePython(const py::bytes& msg_str)
  {
    moveit_msgs::msg::RobotState msg;
    py_bindings_tools::deserializeMsg(msg_str, msg);
    setStartState(msg);
  }

  bool setPoseTargetsPython(py::list& poses, const std::string& end_effector_link = "")
  {
    std::vector<geometry_msgs::msg::Pose> msg;
    convertListToArrayOfPoses(poses, msg);
    return setPoseTargets(msg, end_effector_link);
  }
  py::bytes getPoseTargetPython(const std::string& end_effector_link)
  {
    geometry_msgs::msg::PoseStamped pose =
        moveit::planning_interface::MoveGroupInterface::getPoseTarget(end_effector_link);
    return py_bindings_tools::serializeMsg(pose);
  }

  bool setPoseTargetPython(py::list& pose, const std::string& end_effector_link = "")
  {
    std::vector<double> v = py_bindings_tools::doubleFromList(pose);
    geometry_msgs::msg::Pose msg;
    if (v.size() == 6)
    {
      tf2::Quaternion q;
      q.setRPY(v[3], v[4], v[5]);
      tf2::convert(q, msg.orientation);
    }
    else if (v.size() == 7)
    {
      msg.orientation.x = v[3];
      msg.orientation.y = v[4];
      msg.orientation.z = v[5];
      msg.orientation.w = v[6];
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Pose description expected to consist of either 6 or 7 values");
      return false;
    }
    msg.position.x = v[0];
    msg.position.y = v[1];
    msg.position.z = v[2];
    return setPoseTarget(msg, end_effector_link);
  }

  const char* getEndEffectorLinkCStr() const
  {
    return getEndEffectorLink().c_str();
  }

  const char* getPoseReferenceFrameCStr() const
  {
    return getPoseReferenceFrame().c_str();
  }

  const char* getNameCStr() const
  {
    return getName().c_str();
  }

  const char* getPlannerIdCStr() const
  {
    return getPlannerId().c_str();
  }

  const char* getPlanningPipelineIdCStr() const
  {
    return getPlanningPipelineId().c_str();
  }

  py::dict getNamedTargetValuesPython(const std::string& name)
  {
    py::dict output;
    std::map<std::string, double> positions = getNamedTargetValues(name);
    std::map<std::string, double>::iterator iterator;

    for (iterator = positions.begin(); iterator != positions.end(); ++iterator)
      output[py::str(iterator->first)] = iterator->second;
    return output;
  }

  py::list getNamedTargetsPython()
  {
    return py_bindings_tools::listFromString(getNamedTargets());
  }

  bool movePython()
  {
    GILReleaser gr;
    return move() == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool asyncMovePython()
  {
    return asyncMove() == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool attachObjectPython(const std::string& object_name, const std::string& link_name, const py::list& touch_links)
  {
    return attachObject(object_name, link_name, py_bindings_tools::stringFromList(touch_links));
  }

  bool executePython(const py::bytes& plan_str)
  {
    MoveGroupInterface::Plan plan;
    py_bindings_tools::deserializeMsg(plan_str, plan.trajectory_);
    GILReleaser gr;
    return execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool asyncExecutePython(const py::bytes& plan_str)
  {
    MoveGroupInterface::Plan plan;
    py_bindings_tools::deserializeMsg(plan_str, plan.trajectory_);
    return asyncExecute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  py::tuple planPython()
  {
    MoveGroupInterface::Plan plan;
    moveit_msgs::msg::MoveItErrorCodes res;
    {
      GILReleaser gr;
      res = MoveGroupInterface::plan(plan);
    }
    return py::make_tuple(py_bindings_tools::serializeMsg(res), py_bindings_tools::serializeMsg(plan.trajectory_),
                          plan.planning_time_);
  }

  py::bytes constructMotionPlanRequestPython()
  {
    moveit_msgs::msg::MotionPlanRequest request;
    constructMotionPlanRequest(request);
    return py_bindings_tools::serializeMsg(request);
  }

  py::tuple computeCartesianPathPython(const py::list& waypoints, double eef_step, double jump_threshold,
                                       bool avoid_collisions)
  {
    moveit_msgs::msg::Constraints path_constraints_tmp;
    return doComputeCartesianPathPython(waypoints, eef_step, jump_threshold, avoid_collisions, path_constraints_tmp);
  }

  py::tuple computeCartesianPathConstrainedPython(const py::list& waypoints, double eef_step, double jump_threshold,
                                                  bool avoid_collisions, const py::bytes& path_constraints_str)
  {
    moveit_msgs::msg::Constraints path_constraints;
    py_bindings_tools::deserializeMsg(path_constraints_str, path_constraints);
    return doComputeCartesianPathPython(waypoints, eef_step, jump_threshold, avoid_collisions, path_constraints);
  }

  py::tuple doComputeCartesianPathPython(const py::list& waypoints, double eef_step, double jump_threshold,
                                         bool avoid_collisions, const moveit_msgs::msg::Constraints& path_constraints)
  {
    std::vector<geometry_msgs::msg::Pose> poses;
    convertListToArrayOfPoses(waypoints, poses);
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction;
    {
      GILReleaser gr;
      fraction = computeCartesianPath(poses, eef_step, jump_threshold, trajectory, path_constraints, avoid_collisions);
    }
    return py::make_tuple(py_bindings_tools::serializeMsg(trajectory), fraction);
  }

  // int pickGrasp(const std::string& object, const py::bytes& grasp_str, bool plan_only = false)
  // {
  //   moveit_msgs::msg::Grasp grasp;
  //   py_bindings_tools::deserializeMsg(grasp_str, grasp);
  //   GILReleaser gr;
  //   return pick(object, grasp, plan_only).val;
  // }

  // int pickGrasps(const std::string& object, const py::list& grasp_list, bool plan_only = false)
  // {
  //   int l = py::len(grasp_list);
  //   std::vector<moveit_msgs::msg::Grasp> grasps(l);
  //   for (int i = 0; i < l; ++i)
  //     py_bindings_tools::deserializeMsg(py::bytes(grasp_list[i]), grasps[i]);
  //   GILReleaser gr;
  //   return pick(object, std::move(grasps), plan_only).val;
  // }

  void setPathConstraintsFromMsg(const py::bytes& constraints_str)
  {
    moveit_msgs::msg::Constraints constraints_msg;
    py_bindings_tools::deserializeMsg(constraints_str, constraints_msg);
    setPathConstraints(constraints_msg);
  }

  py::bytes getPathConstraintsPython()
  {
    moveit_msgs::msg::Constraints constraints_msg(getPathConstraints());
    return py_bindings_tools::serializeMsg(constraints_msg);
  }

  py::bytes retimeTrajectory(const py::bytes& ref_state_str, const py::bytes& traj_str, double velocity_scaling_factor,
                             double acceleration_scaling_factor, const std::string& algorithm)
  {
    // Convert reference state message to object
    moveit_msgs::msg::RobotState ref_state_msg;
    py_bindings_tools::deserializeMsg(ref_state_str, ref_state_msg);
    moveit::core::RobotState ref_state_obj(getRobotModel());
    if (moveit::core::robotStateMsgToRobotState(ref_state_msg, ref_state_obj, true))
    {
      // Convert trajectory message to object
      moveit_msgs::msg::RobotTrajectory traj_msg;
      py_bindings_tools::deserializeMsg(traj_str, traj_msg);
      bool algorithm_found = true;
      {
        GILReleaser gr;
        robot_trajectory::RobotTrajectory traj_obj(getRobotModel(), getName());
        traj_obj.setRobotTrajectoryMsg(ref_state_obj, traj_msg);

        // Do the actual retiming
        if (algorithm == "iterative_time_parameterization")
        {
          trajectory_processing::IterativeParabolicTimeParameterization time_param;
          time_param.computeTimeStamps(traj_obj, velocity_scaling_factor, acceleration_scaling_factor);
        }
        else if (algorithm == "iterative_spline_parameterization")
        {
          trajectory_processing::IterativeSplineParameterization time_param;
          time_param.computeTimeStamps(traj_obj, velocity_scaling_factor, acceleration_scaling_factor);
        }
        else if (algorithm == "time_optimal_trajectory_generation")
        {
          trajectory_processing::TimeOptimalTrajectoryGeneration time_param;
          time_param.computeTimeStamps(traj_obj, velocity_scaling_factor, acceleration_scaling_factor);
        }
        else
        {
          RCLCPP_ERROR_STREAM(LOGGER, "Unknown time parameterization algorithm: " << algorithm);
          algorithm_found = false;
          traj_msg = moveit_msgs::msg::RobotTrajectory();
        }

        if (algorithm_found)
          // Convert the retimed trajectory back into a message
          traj_obj.getRobotTrajectoryMsg(traj_msg);
      }
      return py_bindings_tools::serializeMsg(traj_msg);
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Unable to convert RobotState message to RobotState instance.");
      return py::none();
    }
  }

  Eigen::MatrixXd getJacobianMatrixPython(const py::list& joint_values, const py::object& reference_point = py::object())
  {
    const std::vector<double> v = py_bindings_tools::doubleFromList(joint_values);
    std::vector<double> ref;
    if (reference_point.is_none())
      ref = { 0.0, 0.0, 0.0 };
    else
      ref = py_bindings_tools::doubleFromList(reference_point);
    if (ref.size() != 3)
      throw std::invalid_argument("reference point needs to have 3 elements, got " + std::to_string(ref.size()));

    moveit::core::RobotState state(getRobotModel());
    state.setToDefaultValues();
    auto group = state.getJointModelGroup(getName());
    state.setJointGroupPositions(group, v);
    return state.getJacobian(group, Eigen::Map<Eigen::Vector3d>(&ref[0]));
  }

  py::bytes enforceBoundsPython(const py::bytes& msg_str)
  {
    moveit_msgs::msg::RobotState state_msg;
    py_bindings_tools::deserializeMsg(msg_str, state_msg);
    moveit::core::RobotState state(getRobotModel());
    if (moveit::core::robotStateMsgToRobotState(state_msg, state, true))
    {
      state.enforceBounds();
      moveit::core::robotStateToRobotStateMsg(state, state_msg);
      return py_bindings_tools::serializeMsg(state_msg);
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Unable to convert RobotState message to RobotState instance.");
      return py::none();
    }
  }

private:
  std::thread private_executor_thread_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> private_executor_;
};

}  // namespace planning_interface
}  // namespace moveit

using namespace moveit::planning_interface;

PYBIND11_MODULE(move_group_interface, m)
{
  m.doc() = "MOVEIT2 move_group interface.";
  py::class_<MoveGroupInterfacePython>(m, "MoveGroupInterface")
      .def(py::init<const std::string&, const std::string&, const std::string&>())
      .def("async_move", &MoveGroupInterfacePython::asyncMovePython)
      .def("move", &MoveGroupInterfacePython::movePython)
      .def("execute", &MoveGroupInterfacePython::executePython)
      .def("async_execute", &MoveGroupInterfacePython::asyncExecutePython)
      // .def("pick", py::overload_cast<const std::string&, bool>(&MoveGroupInterfacePython::pick))
      // .def("pick", &MoveGroupInterfacePython::pickGrasp)
      // .def("pick", &MoveGroupInterfacePython::pickGrasps)
      // .def("place", &MoveGroupInterfacePython::placePose)
      // .def("place_poses_list", &MoveGroupInterfacePython::placePoses)
      // .def("place", &MoveGroupInterfacePython::placeLocation)
      // .def("place_locations_list", &MoveGroupInterfacePython::placeLocations)
      // .def("place", &MoveGroupInterfacePython::placeAnywhere)
      .def("stop", &MoveGroupInterfacePython::stop)
      .def("get_name", &MoveGroupInterfacePython::getNameCStr)
      .def("get_planning_frame", &MoveGroupInterfacePython::getPlanningFrameCStr)
      .def("get_interface_description", &MoveGroupInterfacePython::getInterfaceDescriptionPython)
      .def("get_active_joints", &MoveGroupInterfacePython::getActiveJointsList)
      .def("get_joints", &MoveGroupInterfacePython::getJointsList)
      .def("get_variable_count", &MoveGroupInterfacePython::getVariableCount)
      .def("allow_looking", &MoveGroupInterfacePython::allowLooking)
      .def("allow_replanning", &MoveGroupInterfacePython::allowReplanning)
      .def("set_pose_reference_frame", &MoveGroupInterfacePython::setPoseReferenceFrame)
      .def("set_pose_reference_frame", &MoveGroupInterfacePython::setPoseReferenceFrame)
      .def("set_end_effector_link", &MoveGroupInterfacePython::setEndEffectorLink)
      .def("get_end_effector_link", &MoveGroupInterfacePython::getEndEffectorLinkCStr)
      .def("get_pose_reference_frame", &MoveGroupInterfacePython::getPoseReferenceFrameCStr)
      .def("set_pose_target", &MoveGroupInterfacePython::setPoseTargetPython)
      .def("set_pose_targets", &MoveGroupInterfacePython::setPoseTargetsPython)
      .def("set_position_target", &MoveGroupInterfacePython::setPositionTarget)
      .def("set_rpy_target", &MoveGroupInterfacePython::setRPYTarget)
      .def("set_orientation_target", &MoveGroupInterfacePython::setOrientationTarget)
      .def("get_current_pose", &MoveGroupInterfacePython::getCurrentPosePython)
      .def("get_current_rpy", &MoveGroupInterfacePython::getCurrentRPYPython)
      .def("get_random_pose", &MoveGroupInterfacePython::getRandomPosePython)
      .def("clear_pose_target", &MoveGroupInterfacePython::clearPoseTarget)
      .def("clear_pose_targets", &MoveGroupInterfacePython::clearPoseTargets)
      .def("set_joint_value_target", &MoveGroupInterfacePython::setJointValueTargetPythonIterable)
      .def("set_joint_value_target", &MoveGroupInterfacePython::setJointValueTargetPythonDict)
      .def("set_joint_value_target", &MoveGroupInterfacePython::setJointValueTargetPerJointPythonList)
      .def("set_joint_value_target",
           py::overload_cast<const std::string&, double>(&MoveGroupInterfacePython::setJointValueTarget))
      .def("set_joint_value_target_from_pose", &MoveGroupInterfacePython::setJointValueTargetFromPosePython)
      .def("set_joint_value_target_from_pose_stamped",
           &MoveGroupInterfacePython::setJointValueTargetFromPoseStampedPython)
      .def("set_joint_value_target_from_joint_state_message",
           &MoveGroupInterfacePython::setJointValueTargetFromJointStatePython)
      .def("get_joint_value_target", &MoveGroupInterfacePython::getJointValueTargetPythonList)
      .def("set_named_target", &MoveGroupInterfacePython::setNamedTarget)
      .def("set_random_target", &MoveGroupInterfacePython::setRandomTarget)
      .def("remember_joint_values",
           py::overload_cast<const std::string&>(&MoveGroupInterfacePython::rememberJointValues))
      .def("remember_joint_values", &MoveGroupInterfacePython::rememberJointValuesFromPythonList)
      .def("start_state_monitor", &MoveGroupInterfacePython::startStateMonitor)
      .def("get_current_joint_values", &MoveGroupInterfacePython::getCurrentJointValuesList)
      .def("get_random_joint_values", &MoveGroupInterfacePython::getRandomJointValuesList)
      .def("get_remembered_joint_values", &MoveGroupInterfacePython::getRememberedJointValuesPython)
      .def("forget_joint_values", &MoveGroupInterfacePython::forgetJointValues)
      .def("get_goal_joint_tolerance", &MoveGroupInterfacePython::getGoalJointTolerance)
      .def("get_goal_position_tolerance", &MoveGroupInterfacePython::getGoalPositionTolerance)
      .def("get_goal_orientation_tolerance", &MoveGroupInterfacePython::getGoalOrientationTolerance)
      .def("set_goal_joint_tolerance", &MoveGroupInterfacePython::setGoalJointTolerance)
      .def("set_goal_position_tolerance", &MoveGroupInterfacePython::setGoalPositionTolerance)
      .def("set_goal_orientation_tolerance", &MoveGroupInterfacePython::setGoalOrientationTolerance)
      .def("set_goal_tolerance", &MoveGroupInterfacePython::setGoalTolerance)
      .def("set_start_state_to_current_state", &MoveGroupInterfacePython::setStartStateToCurrentState)
      .def("set_start_state", &MoveGroupInterfacePython::setStartStatePython)
      .def("set_path_constraints", py::overload_cast<const std::string&>(&MoveGroupInterfacePython::setPathConstraints))
      .def("set_path_constraints_from_msg", &MoveGroupInterfacePython::setPathConstraintsFromMsg)
      .def("get_path_constraints", &MoveGroupInterfacePython::getPathConstraintsPython)
      .def("clear_path_constraints", &MoveGroupInterfacePython::clearPathConstraints)
      .def("get_known_constraints", &MoveGroupInterfacePython::getKnownConstraintsList)
      .def("set_constraints_database", &MoveGroupInterfacePython::setConstraintsDatabase)
      .def("set_workspace", &MoveGroupInterfacePython::setWorkspace)
      .def("set_planning_time", &MoveGroupInterfacePython::setPlanningTime)
      .def("get_planning_time", &MoveGroupInterfacePython::getPlanningTime)
      .def("set_max_velocity_scaling_factor", &MoveGroupInterfacePython::setMaxVelocityScalingFactor)
      .def("set_max_acceleration_scaling_factor", &MoveGroupInterfacePython::setMaxAccelerationScalingFactor)
      .def("set_planner_id", &MoveGroupInterfacePython::setPlannerId)
      .def("get_planner_id", &MoveGroupInterfacePython::getPlannerIdCStr)
      .def("set_planning_pipeline_id", &MoveGroupInterfacePython::setPlanningPipelineId)
      .def("get_planning_pipeline_id", &MoveGroupInterfacePython::getPlanningPipelineIdCStr)
      .def("set_num_planning_attempts", &MoveGroupInterfacePython::setNumPlanningAttempts)
      .def("plan", &MoveGroupInterfacePython::planPython)
      .def("construct_motion_plan_request", &MoveGroupInterfacePython::constructMotionPlanRequestPython)
      .def("compute_cartesian_path", &MoveGroupInterfacePython::computeCartesianPathPython)
      .def("compute_cartesian_path", &MoveGroupInterfacePython::computeCartesianPathConstrainedPython)
      .def("set_support_surface_name", &MoveGroupInterfacePython::setSupportSurfaceName)
      .def("attach_object", &MoveGroupInterfacePython::attachObjectPython)
      .def("detach_object", &MoveGroupInterfacePython::detachObject)
      .def("retime_trajectory", &MoveGroupInterfacePython::retimeTrajectory)
      .def("get_named_targets", &MoveGroupInterfacePython::getNamedTargetsPython)
      .def("get_named_target_values", &MoveGroupInterfacePython::getNamedTargetValuesPython)
      .def("get_current_state_bounded", &MoveGroupInterfacePython::getCurrentStateBoundedPython)
      .def("get_current_state", &MoveGroupInterfacePython::getCurrentStatePython)
      .def("get_jacobian_matrix", &MoveGroupInterfacePython::getJacobianMatrixPython)
      .def("enforce_bounds", &MoveGroupInterfacePython::enforceBoundsPython);
}

/** @endcond */
