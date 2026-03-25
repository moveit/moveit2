# Hybrid planning manager component
generate_parameter_library(hp_manager_parameters res/hp_manager_parameters.yaml)
add_library(moveit_hybrid_planning_manager SHARED
            src/hybrid_planning_manager.cpp)
set_target_properties(moveit_hybrid_planning_manager
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  moveit_hybrid_planning_manager
  hp_manager_parameters
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
