# Local_planner_component
generate_parameter_library(local_planner_parameters
                           res/local_planner_parameters.yaml)
add_library(moveit_local_planner_component SHARED
            src/local_planner_component.cpp)
set_target_properties(moveit_local_planner_component
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  moveit_local_planner_component
  local_planner_parameters
  moveit_core::moveit_core
  ${moveit_msgs_TARGETS}
  moveit_ros_planning::moveit_ros_planning
  moveit_ros_planning_interface::moveit_ros_planning_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_components::component
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2_ros::tf2_ros
  ${trajectory_msgs_TARGETS})
