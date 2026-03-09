add_library(moveit_plan_execution SHARED src/plan_execution.cpp)
set_target_properties(moveit_plan_execution
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  moveit_plan_execution
  moveit_planning_pipeline
  moveit_planning_scene_monitor
  moveit_trajectory_execution_manager
  moveit_core::moveit_core
  rclcpp::rclcpp
  Boost::headers
  class_loader::class_loader
  pluginlib::pluginlib)

install(DIRECTORY include/ DESTINATION include/moveit_ros_planning)
