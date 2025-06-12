add_library(motion_planning_pipeline_plugin SHARED
            src/moveit_planning_pipeline.cpp)
set_target_properties(motion_planning_pipeline_plugin
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  motion_planning_pipeline_plugin
  PUBLIC moveit_core::moveit_planning_scene
         moveit_ros_planning::moveit_planning_scene_monitor rclcpp::rclcpp)
