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
ament_target_dependencies(chomp_motion_planner ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  TARGETS chomp_motion_planner
  EXPORT chomp_motion_plannerTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/chomp_motion_planner)
install(DIRECTORY include/ DESTINATION include/chomp_motion_planner)

ament_export_targets(chomp_motion_plannerTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_EXPORT_DEPENDS})

ament_package()
