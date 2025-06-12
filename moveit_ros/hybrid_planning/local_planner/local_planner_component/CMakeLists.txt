# Local_planner_component
generate_parameter_library(local_planner_parameters
                           res/local_planner_parameters.yaml)
add_library(moveit_local_planner_component SHARED
            src/local_planner_component.cpp)
set_target_properties(moveit_local_planner_component
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  moveit_local_planner_component
  PUBLIC local_planner_parameters
         moveit_core::moveit_planning_scene
         moveit_ros_planning::moveit_planning_scene_monitor
         rclcpp_action::rclcpp_action
         pluginlib::pluginlib
         rclcpp_components::component
         ${moveit_msgs_TARGETS})
