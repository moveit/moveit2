add_library(moveit_pipeline_planning_interface SHARED
  src/pipeline_planning_interface.cpp
  src/plan_responses_container.cpp
  src/solution_selection_functions.cpp
  src/stopping_criterion_function.cpp
)
include(GenerateExportHeader)
generate_export_header(moveit_pipeline_planning_interface)
target_include_directories(moveit_pipeline_planning_interface PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
set_target_properties(moveit_pipeline_planning_interface PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_pipeline_planning_interface
  moveit_core
  moveit_msgs
  moveit_ros_planning
)
