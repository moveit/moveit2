set(MOVEIT_LIB_NAME moveit_collision_plugin_loader)

add_library(${MOVEIT_LIB_NAME} SHARED src/collision_plugin_loader.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
  moveit_core
  rclcpp
  pluginlib
)

install(DIRECTORY include/ DESTINATION include)
