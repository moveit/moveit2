# Global planner component
add_library(moveit_global_planner_component SHARED
            src/global_planner_component.cpp)
set_target_properties(moveit_global_planner_component
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  moveit_global_planner_component
  PUBLIC moveit_ros_planning::moveit_planning_scene_monitor
         pluginlib::pluginlib rclcpp::rclcpp rclcpp_action::rclcpp_action
         rclcpp_components::component)
