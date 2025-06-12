add_library(
  moveit_planning_pipeline_interfaces SHARED
  src/planning_pipeline_interfaces.cpp src/plan_responses_container.cpp
  src/solution_selection_functions.cpp src/stopping_criterion_function.cpp)

include(GenerateExportHeader)
generate_export_header(moveit_planning_pipeline_interfaces)
target_include_directories(
  moveit_planning_pipeline_interfaces
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
set_target_properties(moveit_planning_pipeline_interfaces
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  moveit_planning_pipeline_interfaces
  PUBLIC moveit_planning_pipeline
         ${moveit_msgs_TARGETS}
         moveit_core::moveit_collision_detection
         moveit_core::moveit_planning_interface
         moveit_core::moveit_planning_scene
         rclcpp::rclcpp)

install(DIRECTORY include/ DESTINATION include/moveit_ros_planning)
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_planning_pipeline_interfaces_export.h
  DESTINATION include/moveit_ros_planning)
