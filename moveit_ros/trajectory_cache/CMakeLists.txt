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

set(TRAJECTORY_CACHE_LIBRARIES
    moveit_ros_trajectory_cache_utils_lib
    moveit_ros_trajectory_cache_features_lib
    moveit_ros_trajectory_cache_cache_insert_policies_lib
    moveit_ros_trajectory_cache_lib)

# Utils library
add_library(moveit_ros_trajectory_cache_utils_lib SHARED src/utils/utils.cpp)
generate_export_header(moveit_ros_trajectory_cache_utils_lib)
target_include_directories(
  moveit_ros_trajectory_cache_utils_lib
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/moveit_ros_trajectory_cache>)
ament_target_dependencies(moveit_ros_trajectory_cache_utils_lib
                          ${TRAJECTORY_CACHE_DEPENDENCIES})

# Features library
add_library(
  moveit_ros_trajectory_cache_features_lib SHARED
  src/features/motion_plan_request_features.cpp
  src/features/get_cartesian_path_request_features.cpp)
generate_export_header(moveit_ros_trajectory_cache_features_lib)
target_link_libraries(moveit_ros_trajectory_cache_features_lib
                      moveit_ros_trajectory_cache_utils_lib)
target_include_directories(
  moveit_ros_trajectory_cache_features_lib
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/moveit_ros_trajectory_cache>)
ament_target_dependencies(moveit_ros_trajectory_cache_features_lib
                          ${TRAJECTORY_CACHE_DEPENDENCIES})

# Cache insert policies library
add_library(
  moveit_ros_trajectory_cache_cache_insert_policies_lib SHARED
  src/cache_insert_policies/always_insert_never_prune_policy.cpp
  src/cache_insert_policies/best_seen_execution_time_policy.cpp)
generate_export_header(moveit_ros_trajectory_cache_cache_insert_policies_lib)
target_link_libraries(
  moveit_ros_trajectory_cache_cache_insert_policies_lib
  moveit_ros_trajectory_cache_features_lib
  moveit_ros_trajectory_cache_utils_lib)
target_include_directories(
  moveit_ros_trajectory_cache_cache_insert_policies_lib
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/moveit_ros_trajectory_cache>)
ament_target_dependencies(moveit_ros_trajectory_cache_cache_insert_policies_lib
                          ${TRAJECTORY_CACHE_DEPENDENCIES})

# Trajectory cache library
add_library(moveit_ros_trajectory_cache_lib SHARED src/trajectory_cache.cpp)
generate_export_header(moveit_ros_trajectory_cache_lib)
target_link_libraries(
  moveit_ros_trajectory_cache_lib
  moveit_ros_trajectory_cache_cache_insert_policies_lib
  moveit_ros_trajectory_cache_features_lib
  moveit_ros_trajectory_cache_utils_lib)
target_include_directories(
  moveit_ros_trajectory_cache_lib
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/moveit_ros_trajectory_cache>)
ament_target_dependencies(moveit_ros_trajectory_cache_lib
                          ${TRAJECTORY_CACHE_DEPENDENCIES})

install(
  TARGETS ${TRAJECTORY_CACHE_LIBRARIES}
  EXPORT moveit_ros_trajectory_cacheTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_ros_trajectory_cache)

install(DIRECTORY include/ DESTINATION include/moveit_ros_trajectory_cache)

# Install export headers for each library
foreach(lib_target ${TRAJECTORY_CACHE_LIBRARIES})
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${lib_target}_export.h
          DESTINATION include/moveit_ros_trajectory_cache)
endforeach()

add_subdirectory(test)

ament_export_targets(moveit_ros_trajectory_cacheTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${TRAJECTORY_CACHE_DEPENDENCIES})
ament_package()
