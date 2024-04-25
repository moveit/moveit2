generate_parameter_library(
  srv_kinematics_parameters # cmake target name for the parameter library
  src/srv_kinematics_parameters.yaml # path to input yaml file
)

add_library(moveit_srv_kinematics_plugin SHARED src/srv_kinematics_plugin.cpp)
set_target_properties(moveit_srv_kinematics_plugin
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

ament_target_dependencies(moveit_srv_kinematics_plugin rclcpp moveit_core
                          moveit_msgs)

target_link_libraries(moveit_srv_kinematics_plugin srv_kinematics_parameters)

install(DIRECTORY include/ DESTINATION include/moveit_kinematics)
