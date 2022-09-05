cmake_minimum_required(VERSION 3.22)
project(chomp_motion_planner)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_core
  rclcpp
  trajectory_msgs
  visualization_msgs
)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/chomp_cost.cpp
  src/chomp_parameters.cpp
  src/chomp_trajectory.cpp
  src/chomp_optimizer.cpp
  src/chomp_planner.cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

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

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_EXPORT_DEPENDS})

ament_package()
