cmake_minimum_required(VERSION 3.22)
project(moveit_ros_move_group)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Finds Boost Components
include(ConfigExtras.cmake)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  rclcpp_action
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  moveit_core
  moveit_ros_occupancy_map_monitor
  moveit_ros_planning
  pluginlib
)

include_directories(include)

add_library(moveit_move_group_capabilities_base SHARED
  src/move_group_context.cpp
  src/move_group_capability.cpp
)

add_library(moveit_move_group_default_capabilities SHARED
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

set_target_properties(moveit_move_group_capabilities_base
  PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

set_target_properties(moveit_move_group_default_capabilities
  PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

ament_target_dependencies(moveit_move_group_capabilities_base ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(move_group src/move_group.cpp)
target_include_directories(move_group PUBLIC include)
ament_target_dependencies(move_group  ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
target_link_libraries(move_group moveit_move_group_capabilities_base)

add_executable(list_move_group_capabilities src/list_capabilities.cpp)
ament_target_dependencies(list_move_group_capabilities  ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

ament_target_dependencies(moveit_move_group_default_capabilities ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(moveit_move_group_default_capabilities moveit_move_group_capabilities_base)

install(
  TARGETS
    move_group
    list_move_group_capabilities
  RUNTIME
  DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS
    moveit_move_group_default_capabilities
    moveit_move_group_capabilities_base
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  PROGRAMS
    scripts/load_map
    scripts/save_map
  DESTINATION lib/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(${PROJECT_NAME} default_capabilities_plugin_description.xml)

if(BUILD_TESTING)
# TODO(henningkayser): enable rostests
#  find_package(rostest REQUIRED) # rostest under development in ROS2 https://github.com/ros-planning/moveit2/issues/23
  # this test is flaky
  # add_rostest(test/test_cancel_before_plan_execution.test)
  # add_rostest(test/test_check_state_validity_in_empty_scene.test)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(CONFIG_EXTRAS ConfigExtras.cmake)
