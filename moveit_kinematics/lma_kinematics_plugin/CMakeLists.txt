generate_parameter_library(
  lma_kinematics_parameters # cmake target name for the parameter library
  src/lma_kinematics_parameters.yaml # path to input yaml file
)

add_library(moveit_lma_kinematics_plugin SHARED src/lma_kinematics_plugin.cpp)
set_target_properties(moveit_lma_kinematics_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

ament_target_dependencies(moveit_lma_kinematics_plugin
  rclcpp
  moveit_core
  moveit_msgs
  tf2_kdl
)

target_link_libraries(moveit_lma_kinematics_plugin
  lma_kinematics_parameters
)

install(DIRECTORY include/ DESTINATION include/moveit_kinematics)
