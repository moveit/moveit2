add_library(moveit_plan_execution SHARED src/plan_execution.cpp)
set_target_properties(moveit_plan_execution
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  moveit_plan_execution
  PUBLIC moveit_planning_pipeline
         moveit_planning_scene_monitor
         moveit_trajectory_execution_manager
         class_loader::class_loader
         moveit_core::moveit_collision_detection
         pluginlib::pluginlib
         rclcpp::rclcpp)

install(DIRECTORY include/ DESTINATION include/moveit_ros_planning)
