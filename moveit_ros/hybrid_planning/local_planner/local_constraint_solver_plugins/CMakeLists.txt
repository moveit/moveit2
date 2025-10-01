add_library(forward_trajectory_plugin SHARED src/forward_trajectory.cpp)
set_target_properties(forward_trajectory_plugin
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  forward_trajectory_plugin
  PUBLIC moveit_ros_planning::moveit_planning_scene_monitor rclcpp::rclcpp
         rclcpp_action::rclcpp_action)
