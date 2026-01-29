cmake_minimum_required(VERSION 3.22)
project(chomp_motion_planner LANGUAGES CXX)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rsl REQUIRED)
find_package(trajectory_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS moveit_core rclcpp rsl trajectory_msgs
                                 visualization_msgs)

include_directories(include)

add_library(
  chomp_motion_planner SHARED
  src/chomp_cost.cpp src/chomp_parameters.cpp src/chomp_trajectory.cpp
  src/chomp_optimizer.cpp src/chomp_planner.cpp)
set_target_properties(chomp_motion_planner
                      PROPERTIES VERSION "${chomp_motion_planner_VERSION}")
target_link_libraries(chomp_motion_planner PUBLIC
      ${trajectory_msgs_TARGETS}
      ${visualization_msgs_TARGETS}
      moveit_core::moveit_collision_detection
      moveit_core::moveit_collision_detection_bullet
      moveit_core::moveit_collision_detection_fcl
      moveit_core::moveit_collision_distance_field
      moveit_core::moveit_constraint_samplers
      moveit_core::moveit_distance_field
      moveit_core::moveit_dynamics_solver
      moveit_core::moveit_exceptions
      moveit_core::moveit_kinematic_constraints
      moveit_core::moveit_kinematics_base
      moveit_core::moveit_kinematics_metrics
      moveit_core::moveit_macros
      moveit_core::moveit_planning_interface
      moveit_core::moveit_planning_scene
      moveit_core::moveit_robot_model
      moveit_core::moveit_robot_state
      moveit_core::moveit_robot_trajectory
      moveit_core::moveit_smoothing_base
      moveit_core::moveit_test_utils
      moveit_core::moveit_trajectory_processing
      moveit_core::moveit_transforms
      moveit_core::moveit_utils
      rclcpp::rclcpp
      rsl::rsl
    )
install(
  TARGETS chomp_motion_planner
  EXPORT chomp_motion_plannerTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/chomp_motion_planner)
install(DIRECTORY include/ DESTINATION include/chomp_motion_planner)

ament_export_targets(chomp_motion_plannerTargets)

ament_package()
