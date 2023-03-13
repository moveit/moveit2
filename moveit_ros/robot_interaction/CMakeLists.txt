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


set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_core
  moveit_ros_planning
  interactive_markers
  tf2_geometry_msgs
)

include_directories(include)

add_library(moveit_robot_interaction SHARED
  src/interactive_marker_helpers.cpp
  src/kinematic_options.cpp
  src/kinematic_options_map.cpp
  src/locked_robot_state.cpp
  src/interaction_handler.cpp
  src/robot_interaction.cpp
)
include(GenerateExportHeader)
generate_export_header(moveit_robot_interaction)
target_include_directories(moveit_robot_interaction PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
set_target_properties(moveit_robot_interaction PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_robot_interaction ${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(locked_robot_state_test test/locked_robot_state_test.cpp)
  target_link_libraries(locked_robot_state_test moveit_robot_interaction)
endif()

install(
  TARGETS moveit_robot_interaction
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/moveit_ros_robot_interaction
)

install(DIRECTORY include/ DESTINATION include/moveit_ros_robot_interaction)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_robot_interaction_export.h DESTINATION include/moveit_ros_robot_interaction)

install(DIRECTORY resources DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
