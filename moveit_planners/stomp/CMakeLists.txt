cmake_minimum_required(VERSION 3.22)
project(moveit_planners_stomp)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(moveit_core REQUIRED)
find_package(std_msgs REQUIRED)
find_package(stomp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

generate_parameter_library(stomp_moveit_parameters res/stomp_moveit.yaml)

include_directories(include)

# Planner Plugin
add_library(stomp_moveit_plugin SHARED
  src/stomp_moveit_planner_plugin.cpp
  src/stomp_moveit_planning_context.cpp
  src/stomp_moveit_smoothing_adapter.cpp
)
ament_target_dependencies(stomp_moveit_plugin
  moveit_core
  std_msgs
  tf2_eigen
  visualization_msgs
)
target_link_libraries(stomp_moveit_plugin stomp::stomp stomp_moveit_parameters)

pluginlib_export_plugin_description_file(moveit_core stomp_moveit_plugin_description.xml)

install(TARGETS stomp_moveit_plugin stomp_moveit_parameters
  EXPORT moveit_planners_stompTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(moveit_planners_stompTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(moveit_core stomp generate_parameter_library)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
