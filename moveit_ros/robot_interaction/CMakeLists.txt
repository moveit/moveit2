cmake_minimum_required(VERSION 3.22)
project(moveit_ros_robot_interaction LANGUAGES CXX)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_library(
  moveit_robot_interaction SHARED
  src/interactive_marker_helpers.cpp src/kinematic_options.cpp
  src/kinematic_options_map.cpp src/locked_robot_state.cpp
  src/interaction_handler.cpp src/robot_interaction.cpp)
include(GenerateExportHeader)
generate_export_header(moveit_robot_interaction)
target_include_directories(
  moveit_robot_interaction
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
set_target_properties(
  moveit_robot_interaction PROPERTIES VERSION
                                      "${moveit_ros_robot_interaction_VERSION}")
target_link_libraries(moveit_robot_interaction
                        ${visualization_msgs_TARGETS}
                        tf2_geometry_msgs::tf2_geometry_msgs
                        interactive_markers::interactive_markers
                        moveit_core::moveit_kinematics_base
                        tf2_eigen::tf2_eigen
                        rclcpp::rclcpp)
if(0)#BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(locked_robot_state_test test/locked_robot_state_test.cpp)
  target_link_libraries(locked_robot_state_test moveit_robot_interaction
                          rclcpp::rclcpp
                          )
endif()

install(
  TARGETS moveit_robot_interaction
  EXPORT moveit_ros_robot_interactionTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_ros_robot_interaction)

install(DIRECTORY include/ DESTINATION include/moveit_ros_robot_interaction)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_robot_interaction_export.h
        DESTINATION include/moveit_ros_robot_interaction)

install(DIRECTORY resources DESTINATION share/moveit_ros_robot_interaction)
ament_export_targets(moveit_ros_robot_interactionTargets)

ament_package()
