add_library(moveit_parallel_planning_interface SHARED src/parallel_planning_interface.cpp)
include(GenerateExportHeader)
generate_export_header(moveit_parallel_planning_interface)
target_include_directories(moveit_parallel_planning_interface PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
set_target_properties(moveit_parallel_planning_interface PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_parallel_planning_interface
  moveit_core
  moveit_msgs
  moveit_ros_planning
)
