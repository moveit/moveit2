set(MOVEIT_LIB_NAME moveit_srv_kinematics_plugin)

add_library(${MOVEIT_LIB_NAME} SHARED src/srv_kinematics_plugin.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

ament_target_dependencies(${MOVEIT_LIB_NAME}
  rclcpp
  moveit_core
  moveit_msgs
)

install(DIRECTORY include/ DESTINATION include)
