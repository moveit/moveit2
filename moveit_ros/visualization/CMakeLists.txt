cmake_minimum_required(VERSION 3.22)
project(moveit_ros_visualization LANGUAGES CXX)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

# definition needed for boost/math/constants/constants.hpp included by Ogre to
# compile
add_definitions(-DBOOST_MATH_DISABLE_FLOAT128)

find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
# TODO(JafarAbdi): Uncomment when porting each package
# find_package(moveit_ros_perception REQUIRED)
find_package(moveit_ros_robot_interaction REQUIRED)
find_package(moveit_ros_warehouse REQUIRED)
find_package(object_recognition_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(rviz2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

# Finds Boost Components
include(ConfigExtras.cmake)

# Qt Stuff
find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
# Delegate to QT5 macro
macro(QT_WRAP_UI)
  qt5_wrap_ui(${ARGN})
endmacro()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
add_definitions(-DQT_NO_KEYWORDS)

set(THIS_PACKAGE_LIBRARIES
    moveit_motion_planning_rviz_plugin
    moveit_motion_planning_rviz_plugin_core
    moveit_planning_scene_rviz_plugin
    moveit_planning_scene_rviz_plugin_core
    moveit_robot_state_rviz_plugin
    moveit_robot_state_rviz_plugin_core
    moveit_rviz_plugin_render_tools
    moveit_trajectory_rviz_plugin
    moveit_trajectory_rviz_plugin_core)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    class_loader
    geometric_shapes
    interactive_markers
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_msgs
    moveit_core
    moveit_ros_warehouse
    octomap_msgs
    moveit_ros_robot_interaction
    object_recognition_msgs
    pluginlib
    rclcpp
    rclcpp_action
    rclpy
    rviz2
    tf2_eigen
    Eigen3
    rviz_ogre_vendor
    rviz_common
    rviz_default_plugins)

include_directories(
  rviz_plugin_render_tools/include robot_state_rviz_plugin/include
  planning_scene_rviz_plugin/include motion_planning_rviz_plugin/include
  trajectory_rviz_plugin/include)

add_subdirectory(rviz_plugin_render_tools)
add_subdirectory(robot_state_rviz_plugin)
add_subdirectory(planning_scene_rviz_plugin)
add_subdirectory(motion_planning_rviz_plugin)
add_subdirectory(trajectory_rviz_plugin)

install(DIRECTORY icons DESTINATION share/moveit_ros_visualization)

pluginlib_export_plugin_description_file(
  rviz_common motion_planning_rviz_plugin_description.xml)
pluginlib_export_plugin_description_file(rviz_common
                                         trajectory_rviz_plugin_description.xml)
pluginlib_export_plugin_description_file(
  rviz_common planning_scene_rviz_plugin_description.xml)
pluginlib_export_plugin_description_file(
  rviz_common robot_state_rviz_plugin_description.xml)

install(
  TARGETS ${THIS_PACKAGE_LIBRARIES}
  EXPORT moveit_ros_visualizationTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_ros_visualization)

ament_export_targets(moveit_ros_visualizationTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package(CONFIG_EXTRAS ConfigExtras.cmake)

include_directories("${OGRE_PREFIX_DIR}/include")
