cmake_minimum_required(VERSION 3.5)
project(moveit_planners_chomp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
#find_package(moveit_ros_planning REQUIRED)
find_package(chomp_motion_planner REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS 
  moveit_core
  #moveit_ros_planning
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
  #chomp_interface
  #moveit_planning_scene
  ${MOVEIT_LIB_NAME}
)

install(
  TARGETS ${MOVEIT_LIB_NAME} moveit_chomp_planner_plugin
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(moveit_core chomp_interface_plugin_description.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
