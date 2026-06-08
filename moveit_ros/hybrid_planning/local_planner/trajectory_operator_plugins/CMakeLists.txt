add_library(simple_sampler_plugin SHARED src/simple_sampler.cpp)
set_target_properties(simple_sampler_plugin
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  simple_sampler_plugin
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
