add_library(single_plan_execution_plugin SHARED src/single_plan_execution.cpp)
set_target_properties(single_plan_execution_plugin
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  single_plan_execution_plugin
  moveit_hybrid_planning_manager
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

add_library(replan_invalidated_trajectory_plugin SHARED
            src/replan_invalidated_trajectory.cpp)
set_target_properties(replan_invalidated_trajectory_plugin
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  replan_invalidated_trajectory_plugin
  moveit_hybrid_planning_manager
  single_plan_execution_plugin
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
