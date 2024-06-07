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
ament_target_dependencies(moveit_ros_occupancy_map_monitor
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(moveit_ros_occupancy_map_server src/occupancy_map_server.cpp)
ament_target_dependencies(moveit_ros_occupancy_map_server PUBLIC
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
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
