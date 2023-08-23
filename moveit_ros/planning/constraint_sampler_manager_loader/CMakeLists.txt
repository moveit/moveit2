add_library(moveit_constraint_sampler_manager_loader SHARED
    src/constraint_sampler_manager_loader.cpp)

set_target_properties(moveit_constraint_sampler_manager_loader PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_constraint_sampler_manager_loader
  moveit_core
  rclcpp
  Boost
  pluginlib
)
target_link_libraries(moveit_constraint_sampler_manager_loader
  moveit_rdf_loader
)

install(DIRECTORY include/ DESTINATION include/moveit_ros_planning)
