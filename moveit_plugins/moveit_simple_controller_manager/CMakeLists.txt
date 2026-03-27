cmake_minimum_required(VERSION 3.22)
project(moveit_simple_controller_manager LANGUAGES CXX)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

# Finds Boost Components
include(ConfigExtras.cmake)

set(THIS_PACKAGE_INCLUDE_DEPENDS control_msgs rclcpp rclcpp_action moveit_core
                                 pluginlib)

include_directories(include)

add_library(
  moveit_simple_controller_manager SHARED
  src/moveit_simple_controller_manager.cpp
  src/follow_joint_trajectory_controller_handle.cpp)

set_target_properties(
  moveit_simple_controller_manager
  PROPERTIES VERSION "${moveit_simple_controller_manager_VERSION}")

target_link_libraries(moveit_simple_controller_manager PUBLIC
      ${control_msgs_TARGETS}
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
      pluginlib::pluginlib
      rclcpp::rclcpp
      rclcpp_action::rclcpp_action
    )

install(
  TARGETS moveit_simple_controller_manager
  EXPORT moveit_simple_controller_managerTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_simple_controller_manager)
install(DIRECTORY include/ DESTINATION include/moveit_simple_controller_manager)

ament_export_targets(moveit_simple_controller_managerTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

pluginlib_export_plugin_description_file(
  moveit_core moveit_simple_controller_manager_plugin_description.xml)

ament_package(CONFIG_EXTRAS ConfigExtras.cmake)
