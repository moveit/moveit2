cmake_minimum_required(VERSION 3.22)
project(moveit_ros_control_interface LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp_action controller_manager_msgs moveit_core
    moveit_simple_controller_manager pluginlib trajectory_msgs)

foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

# Finds Boost Components
include(ConfigExtras.cmake)

add_library(moveit_ros_control_interface_plugin SHARED
            src/controller_manager_plugin.cpp)
set_target_properties(
  moveit_ros_control_interface_plugin
  PROPERTIES VERSION "${moveit_ros_control_interface_VERSION}")
target_include_directories(moveit_ros_control_interface_plugin PRIVATE include)
ament_target_dependencies(moveit_ros_control_interface_plugin
                          ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

add_library(moveit_ros_control_interface_trajectory_plugin SHARED
            src/joint_trajectory_controller_plugin.cpp)
set_target_properties(
  moveit_ros_control_interface_trajectory_plugin
  PROPERTIES VERSION "${moveit_ros_control_interface_VERSION}")
target_include_directories(moveit_ros_control_interface_trajectory_plugin
                           PRIVATE include)
ament_target_dependencies(moveit_ros_control_interface_trajectory_plugin
                          ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

add_library(moveit_ros_control_interface_gripper_plugin SHARED
            src/gripper_controller_plugin.cpp)
set_target_properties(
  moveit_ros_control_interface_gripper_plugin
  PROPERTIES VERSION "${moveit_ros_control_interface_VERSION}")
target_include_directories(moveit_ros_control_interface_gripper_plugin
                           PRIVATE include)
ament_target_dependencies(moveit_ros_control_interface_gripper_plugin
                          ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

add_library(moveit_ros_control_interface_empty_plugin SHARED
            src/empty_controller_plugin.cpp)
set_target_properties(
  moveit_ros_control_interface_empty_plugin
  PROPERTIES VERSION "${moveit_ros_control_interface_VERSION}")
target_include_directories(moveit_ros_control_interface_empty_plugin
                           PRIVATE include)
ament_target_dependencies(moveit_ros_control_interface_empty_plugin
                          ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

# ##############################################################################
# Install ##
# ##############################################################################

set(TARGET_LIBRARIES
    moveit_ros_control_interface_plugin
    moveit_ros_control_interface_trajectory_plugin
    moveit_ros_control_interface_gripper_plugin
    moveit_ros_control_interface_empty_plugin)

# Mark executables and/or libraries for installation
install(
  TARGETS ${TARGET_LIBRARIES}
  EXPORT moveit_ros_control_interfaceTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_ros_control_interface)

# Mark cpp header files for installation
install(DIRECTORY include/ DESTINATION include/moveit_ros_control_interface)

pluginlib_export_plugin_description_file(moveit_core moveit_core_plugins.xml)
pluginlib_export_plugin_description_file(
  moveit_ros_control_interface moveit_ros_control_interface_plugins.xml)

ament_export_targets(moveit_ros_control_interfaceTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

ament_package(CONFIG_EXTRAS ConfigExtras.cmake)
