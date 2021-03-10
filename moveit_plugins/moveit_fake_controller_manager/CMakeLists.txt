cmake_minimum_required(VERSION 3.10.2)
project(moveit_fake_controller_manager)

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

# Finds Boost Components
include(ConfigExtras.cmake)

add_library(${PROJECT_NAME} SHARED
  src/moveit_fake_controller_manager.cpp
  src/moveit_fake_controllers.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${PROJECT_NAME}
  Boost
  moveit_core
  moveit_ros_planning
  pluginlib
  rclcpp
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Boost
  moveit_core
  moveit_ros_planning
  pluginlib
  rclcpp
)

pluginlib_export_plugin_description_file(moveit_core moveit_fake_controller_manager_plugin_description.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(CONFIG_EXTRAS ConfigExtras.cmake)
