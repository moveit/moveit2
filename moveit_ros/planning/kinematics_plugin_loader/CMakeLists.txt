add_library(moveit_kinematics_plugin_loader SHARED src/kinematics_plugin_loader.cpp)
set_target_properties(moveit_kinematics_plugin_loader PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_kinematics_plugin_loader
  rclcpp
  urdf
  pluginlib
  class_loader
  moveit_core
)

generate_parameter_library(
  kinematics_parameters # cmake target name for the parameter library
  src/kinematics_parameters.yaml # path to input yaml file
)

target_link_libraries(moveit_kinematics_plugin_loader
  moveit_rdf_loader
  kinematics_parameters
)

install(
  TARGETS kinematics_parameters
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/moveit_ros_planning)
