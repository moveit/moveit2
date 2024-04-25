cmake_minimum_required(VERSION 3.22)
project(moveit_planners_ompl LANGUAGES CXX)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(
  Boost
  REQUIRED
  system
  filesystem
  date_time
  thread
  serialization)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ompl REQUIRED)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wno-overloaded-virtual)
endif()

include_directories(ompl_interface/include)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})

add_subdirectory(ompl_interface)

install(
  TARGETS moveit_ompl_interface moveit_ompl_planner_plugin
  EXPORT moveit_planners_omplTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_planners_ompl)
ament_export_targets(moveit_planners_omplTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(moveit_core ompl)

pluginlib_export_plugin_description_file(moveit_core
                                         ompl_interface_plugin_description.xml)

ament_package()
