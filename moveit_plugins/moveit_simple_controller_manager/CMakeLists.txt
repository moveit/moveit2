cmake_minimum_required(VERSION 3.22)
project(moveit_simple_controller_manager LANGUAGES CXX)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(generate_parameter_library REQUIRED)

# Finds Boost Components
include(ConfigExtras.cmake)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  rclcpp
  rclcpp_action
  moveit_core
  pluginlib
  generate_parameter_library
)

generate_parameter_library(
  moveit_simple_controller_manager_parameters # cmake target name for the parameter library
  src/moveit_simple_controller_manager_parameters.yaml # path to input yaml file
)

include_directories(
  include
)

add_library(moveit_simple_controller_manager SHARED
  src/moveit_simple_controller_manager.cpp
  src/follow_joint_trajectory_controller_handle.cpp
)

set_target_properties(moveit_simple_controller_manager PROPERTIES VERSION "${moveit_simple_controller_manager_VERSION}")

ament_target_dependencies(moveit_simple_controller_manager
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  Boost
)

target_link_libraries(moveit_simple_controller_manager moveit_simple_controller_manager_parameters)

install(
  TARGETS moveit_simple_controller_manager_parameters
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS moveit_simple_controller_manager
  EXPORT moveit_simple_controller_managerTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/moveit_simple_controller_manager
)
install(DIRECTORY include/ DESTINATION include/moveit_simple_controller_manager)

ament_export_targets(moveit_simple_controller_managerTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  Boost
)

pluginlib_export_plugin_description_file(moveit_core moveit_simple_controller_manager_plugin_description.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(CONFIG_EXTRAS ConfigExtras.cmake)
