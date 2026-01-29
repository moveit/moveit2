cmake_minimum_required(VERSION 3.22)
project(moveit_ros_occupancy_map_monitor LANGUAGES CXX)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

if(APPLE)
  find_package(X11 REQUIRED)
endif()

find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
# Enforce system version liboctomap-dev
# https://github.com/moveit/moveit2/issues/2862
find_package(octomap 1.9.7...<1.10.0 REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS} ${X11_INCLUDE_DIR})

set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    moveit_core
    moveit_msgs
    pluginlib
    octomap
    geometric_shapes
    Boost)

add_library(
  moveit_ros_occupancy_map_monitor SHARED
  src/occupancy_map_monitor.cpp src/occupancy_map_monitor_middleware_handle.cpp
  src/occupancy_map_updater.cpp)
set_target_properties(
  moveit_ros_occupancy_map_monitor
  PROPERTIES VERSION "${moveit_ros_occupancy_map_monitor_VERSION}")
target_link_libraries(moveit_ros_occupancy_map_monitor PUBLIC
      ${moveit_msgs_TARGETS}
      geometric_shapes::geometric_shapes
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
    )


add_executable(moveit_ros_occupancy_map_server src/occupancy_map_server.cpp)
target_link_libraries(moveit_ros_occupancy_map_server PUBLIC
      ${moveit_msgs_TARGETS}
      geometric_shapes::geometric_shapes
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
    )
target_link_libraries(moveit_ros_occupancy_map_server
                      PRIVATE moveit_ros_occupancy_map_monitor)

install(
  TARGETS moveit_ros_occupancy_map_monitor
  EXPORT moveit_ros_occupancy_map_monitorTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_ros_occupancy_map_monitor)

install(TARGETS moveit_ros_occupancy_map_server
        DESTINATION lib/moveit_ros_occupancy_map_monitor)
install(DIRECTORY include/ DESTINATION include/moveit_ros_occupancy_map_monitor)

ament_export_targets(moveit_ros_occupancy_map_monitorTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} eigen3_cmake_module)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(occupancy_map_monitor_tests
                  test/occupancy_map_monitor_tests.cpp)
  target_link_libraries(occupancy_map_monitor_tests
                        moveit_ros_occupancy_map_monitor)
endif()

ament_package()
