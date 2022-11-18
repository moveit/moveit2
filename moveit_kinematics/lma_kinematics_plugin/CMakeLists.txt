set(MOVEIT_LIB_NAME moveit_lma_kinematics_plugin)

generate_parameter_library(
  lma_kinematics_parameters # cmake target name for the parameter library
  src/lma_kinematics_parameters.yaml # path to input yaml file
)

add_library(${MOVEIT_LIB_NAME} SHARED src/lma_kinematics_plugin.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

ament_target_dependencies(${MOVEIT_LIB_NAME}
  rclcpp
  moveit_core
  moveit_msgs
  tf2_kdl
)

target_link_libraries(${MOVEIT_LIB_NAME}
  lma_kinematics_parameters
)

install(DIRECTORY include/ DESTINATION include)
