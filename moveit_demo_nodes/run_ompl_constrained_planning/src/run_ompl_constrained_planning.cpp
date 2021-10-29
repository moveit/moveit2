/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik Inc.
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

/* Author: Boston Cleek
   Desc: A simple demo node running ompl constrained planning capabilities for planning and execution
*/

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/macros/console_colors.h>
#include <moveit_msgs/msg/constraints.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ompl_constrained_planning_demo");
static const std::string PLANNING_GROUP = "panda_arm";
static const double PLANNING_TIME_S = 30.0;
static const double PLANNING_ATTEMPTS = 5.0;

class ConstrainedPlanning
{
public:
  ConstrainedPlanning(const rclcpp::Node::SharedPtr& node) : node_(node), marker_count_(0)
  {
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, PLANNING_GROUP);
    move_group_->setPlanningTime(PLANNING_TIME_S);
    move_group_->setNumPlanningAttempts(PLANNING_ATTEMPTS);

    ref_link_ = move_group_->getPoseReferenceFrame();
    ee_link_ = move_group_->getEndEffectorLink();
  }

  void moveToStart()
  {
    // Clear previous goals and constraints
    // May have been set previously
    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();

    RCLCPP_INFO(LOGGER, "moveToStart");

    const moveit_msgs::msg::RobotState goal_state = createRobotState("ready");
    move_group_->setStartStateToCurrentState();
    move_group_->setJointValueTarget(goal_state.joint_state);
    move_group_->move();
  }

  void planBoxConstraints()
  {
    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();

    RCLCPP_INFO(LOGGER, "planBoxConstraints");

    const moveit_msgs::msg::RobotState start_state = createRobotState("ready");
    const geometry_msgs::msg::PoseStamped pose_goal = createPoseGoal(0.0, 0.3, -0.3);
    const moveit_msgs::msg::PositionConstraint pcm = createBoxConstraint();

    moveit_msgs::msg::Constraints path_constraints;
    path_constraints.name = "box constraints";
    path_constraints.position_constraints.emplace_back(pcm);

    move_group_->setStartState(start_state);
    move_group_->setPoseTarget(pose_goal);
    move_group_->setPathConstraints(path_constraints);

    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    const bool plan_success = (move_group_->plan(plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Plan 1 (box constraint) %s", plan_success ? "SUCCEEDED" : "FAILED");

    move_group_->move();
  }

  void planPlaneConstraints()
  {
    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();

    RCLCPP_INFO(LOGGER, "planPlaneConstraints");
    const geometry_msgs::msg::PoseStamped pose_goal = createPoseGoal(0.0, 0.0, -0.3);
    const moveit_msgs::msg::PositionConstraint pcm = verticlePlaneConstraint();

    moveit_msgs::msg::Constraints path_constraints;

    // For equality constraints set to: "use_equality_constraints"
    path_constraints.name = "use_equality_constraints";

    path_constraints.position_constraints.emplace_back(pcm);

    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(pose_goal);
    move_group_->setPathConstraints(path_constraints);

    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    const bool plan_success = (move_group_->plan(plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Plan 2 (plane equality constraint) %s", plan_success ? "SUCCEEDED" : "FAILED");

    move_group_->move();
  }

  void planOrientationConstraints()
  {
    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();

    RCLCPP_INFO(LOGGER, "planOrientationConstraints");

    const std::vector<double> angle_tol = { 1.0, 1.0, 1.0 };  // orientation tolerances (rad)
    const geometry_msgs::msg::PoseStamped pose_goal = createPoseGoal(0.0, 0.5, 0.0);

    RCLCPP_INFO(LOGGER, "Pose goal position [x y z] : %f, %f, %f orientation [x y z w] %f, %f, %f, %f:",
                pose_goal.pose.position.x, pose_goal.pose.position.y, pose_goal.pose.position.z,
                pose_goal.pose.orientation.x, pose_goal.pose.orientation.y, pose_goal.pose.orientation.z,
                pose_goal.pose.orientation.w);

    // Create an orientation constraint
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.header.frame_id = ref_link_;
    ocm.link_name = ee_link_;
    ocm.orientation = pose_goal.pose.orientation;
    ocm.absolute_x_axis_tolerance = angle_tol.at(0);
    ocm.absolute_y_axis_tolerance = angle_tol.at(1);
    ocm.absolute_z_axis_tolerance = angle_tol.at(2);
    ocm.weight = 1.0;

    moveit_msgs::msg::Constraints cm;
    cm.name = "orientation constraints";
    cm.orientation_constraints.emplace_back(ocm);

    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(pose_goal);
    move_group_->setPathConstraints(cm);

    // Plan and move
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    const bool plan_success = (move_group_->plan(plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Plan 1 (box constraint) %s", plan_success ? "SUCCEEDED" : "FAILED");

    move_group_->move();
  }

  void planLineConstraints()
  {
    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();

    const moveit_msgs::msg::RobotState start_state = createRobotState("ready");
    const geometry_msgs::msg::PoseStamped pose_goal = createPoseGoal(0.0, 0.0, -0.3);
    const moveit_msgs::msg::PositionConstraint pcm = createLineConstraint();

    moveit_msgs::msg::Constraints path_constraints;

    // For equality constraints set to: "use_equality_constraints"
    path_constraints.name = "use_equality_constraints";

    path_constraints.position_constraints.emplace_back(pcm);

    move_group_->setStartState(start_state);
    move_group_->setPoseTarget(pose_goal);
    move_group_->setPathConstraints(path_constraints);

    moveit::planning_interface::MoveGroupInterface::Plan plan3;
    const bool plan_success = (move_group_->plan(plan3) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Plan 3 (line equality constraint) %s", plan_success ? "SUCCEEDED" : "FAILED");
  }

  void planVerticlePlaneConstraints()
  {
    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();

    const moveit_msgs::msg::RobotState start_state = createRobotState("ready");
    const geometry_msgs::msg::PoseStamped pose_goal = createPoseGoal(0.1, 0.0, -0.3);
    const moveit_msgs::msg::PositionConstraint pcm = verticlePlaneConstraint();

    moveit_msgs::msg::Constraints path_constraints;

    // For equality constraints set to: "use_equality_constraints"
    path_constraints.name = "use_equality_constraints";

    path_constraints.position_constraints.emplace_back(pcm);

    addObstacle("box1");

    move_group_->setStartState(start_state);
    move_group_->setPoseTarget(pose_goal);
    move_group_->setPathConstraints(path_constraints);

    moveit::planning_interface::MoveGroupInterface::Plan plan4;
    const bool plan_success = (move_group_->plan(plan4) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Plan 4 (verticle plane equality constraint) %s", plan_success ? "SUCCEEDED" : "FAILED");
  }

  void deleteAllMarkers()
  {
    RCLCPP_INFO(LOGGER, "Delete all markers");

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = ref_link_;
    marker.header.stamp = node_->now();
    marker.ns = "/";
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_pub_->publish(marker);
    marker_count_ = 0;
  }

private:
  moveit_msgs::msg::RobotState createRobotState(const std::string& name) const
  {
    RCLCPP_INFO(LOGGER, "createRobotState");

    const std::map<std::string, double> joint_map = move_group_->getNamedTargetValues(name);
    moveit_msgs::msg::RobotState robot_state;
    robot_state.joint_state.header.frame_id = ref_link_;
    robot_state.joint_state.header.stamp = node_->now();

    for (const auto& joint : joint_map)
    {
      RCLCPP_INFO(LOGGER, "Starting State, joint: %s and angle %f:", joint.first.c_str(), joint.second);
      robot_state.joint_state.name.emplace_back(joint.first);
      robot_state.joint_state.position.emplace_back(joint.second);
    }

    return robot_state;
  }

  geometry_msgs::msg::PoseStamped createPoseGoal(double dx, double dy, double dz)
  {
    geometry_msgs::msg::PoseStamped pose = move_group_->getCurrentPose();

    displaySphere(pose.pose, "red");

    pose.pose.position.x += dx;
    pose.pose.position.y += dy;
    pose.pose.position.z += dz;

    displaySphere(pose.pose, "green");

    return pose;
  }

  moveit_msgs::msg::PositionConstraint createBoxConstraint()
  {
    moveit_msgs::msg::PositionConstraint pcm;
    pcm.header.frame_id = ref_link_;
    pcm.link_name = ee_link_;
    pcm.weight = 1.0;

    shape_msgs::msg::SolidPrimitive cbox;
    cbox.type = shape_msgs::msg::SolidPrimitive::BOX;
    cbox.dimensions = { 0.1, 0.4, 0.4 };
    pcm.constraint_region.primitives.emplace_back(cbox);

    geometry_msgs::msg::PoseStamped pose = move_group_->getCurrentPose();

    geometry_msgs::msg::Pose cbox_pose;
    cbox_pose.position.x = pose.pose.position.x;
    cbox_pose.position.y = 0.15;
    cbox_pose.position.z = 0.45;
    pcm.constraint_region.primitive_poses.emplace_back(cbox_pose);

    displayBox(cbox_pose, cbox.dimensions);

    return pcm;
  }

  moveit_msgs::msg::PositionConstraint createPlaneConstraint()
  {
    moveit_msgs::msg::PositionConstraint pcm;
    pcm.header.frame_id = ref_link_;
    pcm.link_name = ee_link_;
    pcm.weight = 1.0;

    shape_msgs::msg::SolidPrimitive cbox;
    cbox.type = shape_msgs::msg::SolidPrimitive::BOX;

    // For equality constraint set box dimension to: 1e-3 > 0.0005 > 1e-4
    cbox.dimensions = { 1.0, 0.0005, 1.0 };
    pcm.constraint_region.primitives.emplace_back(cbox);

    geometry_msgs::msg::PoseStamped pose = move_group_->getCurrentPose();

    geometry_msgs::msg::Pose cbox_pose;
    cbox_pose.position = pose.pose.position;

    // turn the constraint region 45 degrees around the x-axis
    tf2::Quaternion quat;
    quat.setRPY(M_PI / 4.0, 0.0, 0.0);

    cbox_pose.orientation.x = quat.x();
    cbox_pose.orientation.y = quat.y();
    cbox_pose.orientation.z = quat.z();
    cbox_pose.orientation.w = quat.w();

    pcm.constraint_region.primitive_poses.emplace_back(cbox_pose);

    displayBox(cbox_pose, cbox.dimensions);

    return pcm;
  }

  moveit_msgs::msg::PositionConstraint verticlePlaneConstraint()
  {
    moveit_msgs::msg::PositionConstraint pcm;
    pcm.header.frame_id = ref_link_;
    pcm.link_name = ee_link_;
    pcm.weight = 1.0;

    shape_msgs::msg::SolidPrimitive cbox;
    cbox.type = shape_msgs::msg::SolidPrimitive::BOX;

    // For equality constraint set box dimension to: 1e-3 > 0.0005 > 1e-4
    cbox.dimensions = { 1.0, 0.0005, 1.0 };
    pcm.constraint_region.primitives.emplace_back(cbox);

    geometry_msgs::msg::PoseStamped pose = move_group_->getCurrentPose();

    geometry_msgs::msg::Pose cbox_pose;
    cbox_pose.position = pose.pose.position;

    pcm.constraint_region.primitive_poses.emplace_back(cbox_pose);

    displayBox(cbox_pose, cbox.dimensions);

    return pcm;
  }

  moveit_msgs::msg::PositionConstraint createLineConstraint()
  {
    moveit_msgs::msg::PositionConstraint pcm;
    pcm.header.frame_id = ref_link_;
    pcm.link_name = ee_link_;
    pcm.weight = 1.0;

    shape_msgs::msg::SolidPrimitive cbox;
    cbox.type = shape_msgs::msg::SolidPrimitive::BOX;

    // For equality constraint set box dimension to: 1e-3 > 0.0005 > 1e-4
    cbox.dimensions = { 0.0005, 0.0005, 1.0 };
    pcm.constraint_region.primitives.emplace_back(cbox);

    geometry_msgs::msg::PoseStamped pose = move_group_->getCurrentPose();

    geometry_msgs::msg::Pose cbox_pose;
    cbox_pose.position = pose.pose.position;

    // turn the constraint region 45 degrees around the x-axis
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, 0.0);

    cbox_pose.orientation.x = quat.x();
    cbox_pose.orientation.y = quat.y();
    cbox_pose.orientation.z = quat.z();
    cbox_pose.orientation.w = quat.w();

    pcm.constraint_region.primitive_poses.emplace_back(cbox_pose);

    displayBox(cbox_pose, cbox.dimensions);

    return pcm;
  }

  void displayBox(const geometry_msgs::msg::Pose& pose,
                  const rosidl_runtime_cpp::BoundedVector<double, 3, std::allocator<double>>& dimensions)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = ref_link_;
    marker.header.stamp = node_->now();
    marker.ns = "/";
    marker.id = marker_count_;

    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0);

    marker.color.a = 0.5;
    marker.pose = pose;
    marker.scale.x = dimensions.at(0);
    marker.scale.y = dimensions.at(1);
    marker.scale.z = dimensions.at(2);

    marker_pub_->publish(marker);
    marker_count_++;
  }

  void displaySphere(const geometry_msgs::msg::Pose& pose, const std::string& color)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = ref_link_;
    marker.header.stamp = node_->now();
    marker.ns = "/";
    marker.id = marker_count_;

    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0);

    marker.pose = pose;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    if (color == "red")
    {
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
    }
    else if (color == "green")
    {
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Sphere color not specified");
    }

    marker_pub_->publish(marker);
    marker_count_++;
  }

  void addObstacle(const std::string& object_id) const
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = ref_link_;
    collision_object.id = object_id;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.2;
    primitive.dimensions[1] = 0.4;
    primitive.dimensions[2] = 0.1;

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.5;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.5;

    collision_object.primitives.emplace_back(primitive);
    collision_object.primitive_poses.emplace_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    // Now, let's add the collision object into the world
    planning_scene_interface_.addCollisionObjects(collision_objects);
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  std::string ref_link_, ee_link_;
  unsigned int marker_count_;
};

int main(int argc, char** argv)
{
  RCLCPP_INFO(LOGGER, "Initialize node");
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  // This enables loading undeclared parameters
  // best practice would be to declare parameters in the corresponding classes
  // and provide descriptions about expected use
  node_options.automatically_declare_parameters_from_overrides(true);

  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("run_ompl_constrained_planning", node_options);
  ConstrainedPlanning constrained_planning(node);

  std::thread run_demo([&constrained_planning]() {
    // Wait for rviz
    rclcpp::sleep_for(std::chrono::seconds(5));

    // Note: see See moveit_core/kinematic_constraints/src/utils.cpp
    // contains helper functions to automate constructing constraint messages.

    // 1. Box Constraints
    constrained_planning.planBoxConstraints();
    constrained_planning.deleteAllMarkers();

    constrained_planning.moveToStart();

    // 2. Equality Constraints
    // If you make a box really thin along one dimension, you get something plane like.
    // When solving the problem, you can tell the planner to model this really thin box as an equality constraint.
    // This is achieved by setting the name of the constraint to :code:`"use_equality_constraints"`.
    // In addition, any dimension of the box below a treshold of :code:`0.001` will be considered an equality
    // constraint. However, if we make it too small, the box will be thinner that the tolerance used by OMPL to evaluate
    // constraints (:code:`1e-4` by default). MoveIt will use the stricter tolerance (the box width) to check the
    // constraints, and many states will appear invalid. That's where the number :code:`0.0005` comes from, it is
    // between :code:`0.00001` and :code:`0.001`.
    constrained_planning.planPlaneConstraints();
    constrained_planning.deleteAllMarkers();

    constrained_planning.moveToStart();

    // 3. Orientation Constraints
    constrained_planning.planOrientationConstraints();
    constrained_planning.deleteAllMarkers();

    // Additional examples with line constraints and obstacles
    // constrained_planning.planLineConstraints();
    // constrained_planning.planVerticlePlaneConstraints();
  });

  rclcpp::spin(node);
  run_demo.join();

  rclcpp::shutdown();
  return 0;
}
