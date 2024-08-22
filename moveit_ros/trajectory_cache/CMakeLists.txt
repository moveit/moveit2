cmake_minimum_required(VERSION 3.22)
project(moveit_ros_trajectory_cache)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_warehouse REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(warehouse_ros REQUIRED)

include(GenerateExportHeader)
include_directories(include)

set(TRAJECTORY_CACHE_DEPENDENCIES
    geometry_msgs
    moveit_ros_planning_interface
    moveit_ros_warehouse
    rclcpp
    tf2
    tf2_ros
    trajectory_msgs
    warehouse_ros)

# Trajectory cache library
add_library(moveit_ros_trajectory_cache_lib SHARED src/trajectory_cache.cpp)
generate_export_header(moveit_ros_trajectory_cache_lib)
target_include_directories(
  moveit_ros_trajectory_cache_lib
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/moveit_ros_trajectory_cache>)
ament_target_dependencies(moveit_ros_trajectory_cache_lib
                          ${TRAJECTORY_CACHE_DEPENDENCIES})

install(
  TARGETS moveit_ros_trajectory_cache_lib
  EXPORT moveit_ros_trajectory_cacheTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_ros_trajectory_cache)

install(DIRECTORY include/ DESTINATION include/moveit_ros_trajectory_cache)
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_ros_trajectory_cache_lib_export.h
  DESTINATION include/moveit_ros_trajectory_cache)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(rmf_utils REQUIRED)
  find_package(warehouse_ros_sqlite REQUIRED)

  # This test executable is run by the pytest_test, since a node is required for
  # testing move groups
  add_executable(test_trajectory_cache test/test_trajectory_cache.cpp)
  target_link_libraries(test_trajectory_cache moveit_ros_trajectory_cache_lib)
  ament_target_dependencies(test_trajectory_cache warehouse_ros_sqlite)

  install(TARGETS test_trajectory_cache RUNTIME DESTINATION lib/${PROJECT_NAME})

  ament_add_pytest_test(
    test_trajectory_cache_py "test/test_trajectory_cache.py" WORKING_DIRECTORY
    "${CMAKE_CURRENT_BINARY_DIR}")

endif()

ament_export_targets(moveit_ros_trajectory_cacheTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${TRAJECTORY_CACHE_DEPENDENCIES})
ament_package()
