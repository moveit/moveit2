cmake_minimum_required(VERSION 3.5)
project(moveit_planners_chomp)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(chomp_motion_planner REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_core
  chomp_motion_planner
  moveit_core
  pluginlib
  rclcpp
)

include_directories(
  include
)

set(MOVEIT_LIB_NAME moveit_chomp_interface)

add_library(${MOVEIT_LIB_NAME} SHARED
  src/chomp_interface.cpp
  src/chomp_planning_context.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_library(moveit_chomp_planner_plugin SHARED src/chomp_plugin.cpp)
ament_target_dependencies(moveit_chomp_planner_plugin ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(moveit_chomp_planner_plugin
  ${MOVEIT_LIB_NAME}
)

install(
  TARGETS ${MOVEIT_LIB_NAME} moveit_chomp_planner_plugin
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(moveit_core chomp_interface_plugin_description.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()
endif()

# TODO: Enable testing ()
#if(CATKIN_ENABLE_TESTING)
  #set(CHOMP_TEST_DEPS moveit_ros_planning_interface)
#else()
  #set(CHOMP_TEST_DEPS)
#endif()

ament_package()
