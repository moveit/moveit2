add_executable(moveit_list_planning_adapter_plugins src/list_planning_adapter_plugins.cpp)
ament_target_dependencies(moveit_list_planning_adapter_plugins
  moveit_core
  rclcpp
  pluginlib
)

install(TARGETS moveit_list_planning_adapter_plugins
  DESTINATION lib/${PROJECT_NAME}
)
