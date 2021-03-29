#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit/macros/console_colors.h>
// https://ros-planning.github.io/moveit_tutorials/doc/move_group_interface/move_group_interface_tutorial.html

void prompt(const std::string& message)
{
  printf(MOVEIT_CONSOLE_COLOR_GREEN "\n%s" MOVEIT_CONSOLE_COLOR_RESET, message.c_str());
  fflush(stdout);
  while (std::cin.get() != '\n' && rclcpp::ok())
    ;
}

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);
  // For current state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "panda_arm";

  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  prompt("Press 'Enter' to start the demo");

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getRobotModel()->getJointModelGroup(PLANNING_GROUP);

  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  // Planning to a Pose goal
  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.5;
  move_group.setPoseTarget(target_pose1);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(LOGGER, "Plan 1 (pose goal) %s", success ? "SUCCEEDED" : "FAILED");

  prompt("Press 'Enter' to continue the demo");

  // Moving to a pose goal
  move_group.move();

  // Planning to a joint-space goal
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[0] = -1.0;  // radians
  move_group.setJointValueTarget(joint_group_positions);

  move_group.setMaxVelocityScalingFactor(0.05);
  move_group.setMaxAccelerationScalingFactor(0.05);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Plan 2 (joint space goal) %s", success ? "SUCCEEDED" : "FAILED");

  prompt("Press 'Enter' to continue the demo");
  // Planning with Path Constraints
  moveit_msgs::msg::OrientationConstraint ocm;
  ocm.link_name = "panda_link7";
  ocm.header.frame_id = "panda_link0";
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  moveit_msgs::msg::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);

  moveit::core::RobotState start_state(*move_group.getCurrentState());
  geometry_msgs::msg::Pose start_pose2;
  start_pose2.position.x = 0.55;
  start_pose2.position.y = -0.05;
  start_pose2.position.z = 0.8;
  start_state.setFromIK(joint_model_group, start_pose2);
  move_group.setStartState(start_state);

  // Now we will plan to the earlier pose target from the new
  // start state that we have just created.
  move_group.setPoseTarget(target_pose1);

  // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
  // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
  move_group.setPlanningTime(10.0);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Plan 3 (constraints) %s", success ? "SUCCEEDED" : "FAILED");

  prompt("Press 'Enter' to continue the demo");
  // When done with the path constraint be sure to clear it.
  move_group.clearPathConstraints();

  // Cartesian Paths
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(start_pose2);

  geometry_msgs::msg::Pose target_pose3 = start_pose2;

  target_pose3.position.z -= 0.2;
  waypoints.push_back(target_pose3);  // down

  target_pose3.position.y -= 0.2;
  waypoints.push_back(target_pose3);  // right

  target_pose3.position.z += 0.2;
  target_pose3.position.y += 0.2;
  target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3);  // up and left

  // We want the Cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in Cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  prompt("Press 'Enter' to continue the demo");
  // You can execute a trajectory like this.
  move_group.execute(trajectory);

  // Adding/Removing Objects and Attaching/Detaching Objects
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.4;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.4;
  box_pose.position.y = -0.2;
  box_pose.position.z = 1.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  RCLCPP_INFO(LOGGER, "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  prompt("Press 'Enter' to continue the demo");
  // Now when we plan a trajectory it will avoid the obstacle
  move_group.setStartState(*move_group.getCurrentState());
  geometry_msgs::msg::Pose another_pose;
  another_pose.orientation.w = 1.0;
  another_pose.position.x = 0.4;
  another_pose.position.y = -0.4;
  another_pose.position.z = 0.9;
  move_group.setPoseTarget(another_pose);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");

  prompt("Press 'Enter' once the collision object attaches to the robot");
  // Now, let's attach the collision object to the robot.
  RCLCPP_INFO(LOGGER, "Attach the object to the robot");
  move_group.attachObject(collision_object.id);

  prompt("Press 'Enter' once the collision object detaches from the robot");
  // Now, let's detach the collision object from the robot.
  RCLCPP_INFO(LOGGER, "Detach the object from the robot");
  move_group.detachObject(collision_object.id);

  // Now, let's remove the collision object from the world.
  RCLCPP_INFO(LOGGER, "Remove the object from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids);
  prompt("Press 'Enter' once the collision object disappears");

  rclcpp::shutdown();
  return 0;
}
