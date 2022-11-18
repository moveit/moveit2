set(MOVEIT_LIB_NAME moveit_kinematics_plugin_loader)

add_library(${MOVEIT_LIB_NAME} SHARED src/kinematics_plugin_loader.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
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

target_link_libraries(${MOVEIT_LIB_NAME}
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

install(DIRECTORY include/ DESTINATION include)
