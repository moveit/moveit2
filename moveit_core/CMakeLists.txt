cmake_minimum_required(VERSION 3.22)
project(moveit_core LANGUAGES CXX)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(Bullet 2.87 REQUIRED)
find_package(common_interfaces REQUIRED)
find_package(eigen_stl_containers REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(fcl REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(moveit_msgs REQUIRED)
# Enforce system version liboctomap-dev
# https://github.com/moveit/moveit2/issues/2862
find_package(octomap 1.9.7...<1.10.0 REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(osqp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(random_numbers REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rsl REQUIRED)
find_package(ruckig REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(srdfdom REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(visualization_msgs REQUIRED)

# Finds Boost Components
include(ConfigExtras.cmake)

add_subdirectory(collision_detection_bullet)
add_subdirectory(collision_detection_fcl)
add_subdirectory(collision_detection)
add_subdirectory(collision_distance_field)
add_subdirectory(constraint_samplers)
add_subdirectory(controller_manager)
add_subdirectory(distance_field)
add_subdirectory(dynamics_solver)
add_subdirectory(exceptions)
add_subdirectory(kinematic_constraints)
add_subdirectory(kinematics_base)
add_subdirectory(kinematics_metrics)
add_subdirectory(macros)
add_subdirectory(online_signal_smoothing)
add_subdirectory(planning_interface)
add_subdirectory(planning_scene)
add_subdirectory(robot_model)
add_subdirectory(robot_state)
add_subdirectory(robot_trajectory)
add_subdirectory(trajectory_processing)
add_subdirectory(transforms)
add_subdirectory(utils)
add_subdirectory(version)

install(
  TARGETS collision_detector_bullet_plugin
          collision_detector_fcl_plugin
          moveit_acceleration_filter
          moveit_acceleration_filter_parameters
          moveit_butterworth_filter
          moveit_butterworth_filter_parameters
          moveit_collision_detection
          moveit_collision_detection_bullet
          moveit_collision_detection_fcl
          moveit_collision_distance_field
          moveit_constraint_samplers
          moveit_distance_field
          moveit_dynamics_solver
          moveit_exceptions
          moveit_kinematic_constraints
          moveit_kinematics_base
          moveit_kinematics_metrics
          moveit_macros
          moveit_planning_interface
          moveit_planning_scene
          moveit_robot_model
          moveit_robot_state
          moveit_robot_trajectory
          moveit_ruckig_filter
          moveit_ruckig_filter_parameters
          moveit_smoothing_base
          moveit_test_utils
          moveit_trajectory_processing
          moveit_transforms
          moveit_utils
  EXPORT moveit_coreTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(moveit_coreTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  angles
  Bullet
  common_interfaces
  eigen_stl_containers
  Eigen3
  eigen3_cmake_module
  fcl
  generate_parameter_library
  geometric_shapes
  geometry_msgs
  kdl_parser
  moveit_msgs
  octomap
  octomap_msgs
  osqp
  pluginlib
  random_numbers
  rclcpp
  rsl
  ruckig
  sensor_msgs
  shape_msgs
  srdfdom
  std_msgs
  tf2_eigen
  tf2_geometry_msgs
  tf2_kdl
  trajectory_msgs
  urdf
  urdfdom
  urdfdom_headers
  visualization_msgs)

# Plugin exports
pluginlib_export_plugin_description_file(moveit_core
                                         collision_detector_fcl_description.xml)
pluginlib_export_plugin_description_file(
  moveit_core collision_detector_bullet_description.xml)
pluginlib_export_plugin_description_file(moveit_core
                                         filter_plugin_acceleration.xml)
pluginlib_export_plugin_description_file(moveit_core
                                         filter_plugin_butterworth.xml)
pluginlib_export_plugin_description_file(moveit_core filter_plugin_ruckig.xml)

ament_package(CONFIG_EXTRAS ConfigExtras.cmake)
