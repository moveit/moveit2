generate_parameter_library(planning_pipeline_parameters res/planning_pipeline_parameters.yaml)

add_library(moveit_planning_pipeline SHARED src/planning_pipeline.cpp)
target_link_libraries(moveit_planning_pipeline planning_pipeline_parameters)
include(GenerateExportHeader)
generate_export_header(moveit_planning_pipeline)
target_include_directories(moveit_planning_pipeline PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
set_target_properties(moveit_planning_pipeline PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

ament_target_dependencies(moveit_planning_pipeline
  moveit_core
  moveit_msgs
  rclcpp
  Boost
  pluginlib
)

install(DIRECTORY include/ DESTINATION include/moveit_ros_planning)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_planning_pipeline_export.h DESTINATION include/moveit_ros_planning)
