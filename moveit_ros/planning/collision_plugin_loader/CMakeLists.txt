set(MOVEIT_LIB_NAME moveit_collision_plugin_loader)

add_library(${MOVEIT_LIB_NAME} SHARED src/collision_plugin_loader.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
ament_target_dependencies(${MOVEIT_LIB_NAME}
  moveit_core
  rclcpp
  pluginlib
)

install(DIRECTORY include/ DESTINATION include)
