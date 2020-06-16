cmake_minimum_required(VERSION 3.10.2)
project(moveit_ros_move_group)

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_occupancy_map_monitor)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED system filesystem date_time program_options thread)

add_library(moveit_move_group_capabilities_base src/move_group_context.cpp
  src/move_group_capability.cpp)
set_target_properties(moveit_move_group_capabilities_base
  PROPERTIES
  VERSION "${${PROJECT_NAME}_VERSION}")
target_include_directories(moveit_move_group_capabilities_base PRIVATE include)
ament_target_dependencies(moveit_move_group_capabilities_base 
  rclcpp moveit_core moveit_ros_planning tf2_geometry_msgs)

add_executable(move_group src/move_group.cpp)
target_include_directories(move_group PRIVATE include)
ament_target_dependencies(move_group 
  rclcpp rclcpp_action moveit_core moveit_ros_planning tf2_ros Boost)

add_executable(list_move_group_capabilities src/list_capabilities.cpp)
target_include_directories(list_move_group_capabilities PRIVATE include)
ament_target_dependencies(list_move_group_capabilities 
  rclcpp moveit_core moveit_ros_planning Boost pluginlib)

add_library(moveit_move_group_default_capabilities
  src/default_capabilities/move_action_capability.cpp
  src/default_capabilities/plan_service_capability.cpp
  src/default_capabilities/execute_trajectory_action_capability.cpp
  src/default_capabilities/query_planners_service_capability.cpp
  src/default_capabilities/kinematics_service_capability.cpp
  src/default_capabilities/state_validation_service_capability.cpp
  src/default_capabilities/cartesian_path_service_capability.cpp
  src/default_capabilities/get_planning_scene_service_capability.cpp
  src/default_capabilities/apply_planning_scene_service_capability.cpp
  src/default_capabilities/clear_octomap_service_capability.cpp
  src/default_capabilities/tf_publisher_capability.cpp)
set_target_properties(moveit_move_group_default_capabilities
  PROPERTIES
  VERSION "${${PROJECT_NAME}_VERSION}")
target_include_directories(moveit_move_group_default_capabilities PRIVATE include)
ament_target_dependencies(moveit_move_group_default_capabilities 
  rclcpp rclcpp_action moveit_core moveit_ros_planning)

install(TARGETS move_group list_move_group_capabilities
  RUNTIME
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS moveit_move_group_capabilities_base
  moveit_move_group_default_capabilities
  ARCHIVE
  DESTINATION lib
  LIBRARY
  DESTINATION lib
  RUNTIME
  DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)

install(PROGRAMS scripts/load_map scripts/save_map
  DESTINATION lib/${PROJECT_NAME})

install(FILES default_capabilities_plugin_description.xml
  DESTINATION share/${PROJECT_NAME})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # this test is flaky
  # add_rostest(test/test_cancel_before_plan_execution.test)
endif()

ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_action)
ament_export_dependencies(std_srvs)
ament_export_dependencies(pluginlib)
ament_export_dependencies(tf2)
ament_export_dependencies(moveit_ros_planning)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(moveit_core)
ament_export_dependencies(moveit_ros_occupancy_map_monitor)

ament_package()
