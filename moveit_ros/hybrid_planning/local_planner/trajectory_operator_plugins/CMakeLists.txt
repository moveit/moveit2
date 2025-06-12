add_library(simple_sampler_plugin SHARED src/simple_sampler.cpp)
set_target_properties(simple_sampler_plugin
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  simple_sampler_plugin
  PUBLIC rclcpp::rclcpp rclcpp_action::rclcpp_action
         moveit_core::moveit_robot_state pluginlib::pluginlib
         ${moveit_msgs_TARGETS})
