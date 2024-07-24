cmake_minimum_required(VERSION 3.8)
project(moveit_ros_trajectory_cache)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_warehouse REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(warehouse_ros REQUIRED)
find_package(warehouse_ros_sqlite REQUIRED)

include_directories(include)

set(TRAJECTORY_CACHE_DEPENDENCIES
    geometry_msgs
    moveit_ros_planning_interface
    moveit_ros_warehouse
    rclcpp
    tf2
    tf2_ros
    trajectory_msgs
    warehouse_ros
    warehouse_ros_sqlite)

# ==============================================================================

# Motion plan cache library
add_library(trajectory_cache src/trajectory_cache.cpp)
ament_target_dependencies(trajectory_cache ${TRAJECTORY_CACHE_DEPENDENCIES})
target_include_directories(
  trajectory_cache PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)

# ==============================================================================

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(rmf_utils REQUIRED)

  # This test executable is run by the pytest_test, since a node is required for
  # testing move groups
  add_executable(test_trajectory_cache test/test_trajectory_cache.cpp)
  target_link_libraries(test_trajectory_cache trajectory_cache)

  install(TARGETS test_trajectory_cache RUNTIME DESTINATION lib/${PROJECT_NAME})

  ament_add_pytest_test(
    test_trajectory_cache_py "test/test_trajectory_cache.py" WORKING_DIRECTORY
    "${CMAKE_CURRENT_BINARY_DIR}")

endif()

ament_export_dependencies(${TRAJECTORY_CACHE_DEPENDENCIES})
ament_package()
