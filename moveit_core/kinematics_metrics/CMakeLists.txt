set(MOVEIT_LIB_NAME moveit_kinematics_metrics)

add_library(${MOVEIT_LIB_NAME} SHARED src/kinematics_metrics.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

ament_target_dependencies(${MOVEIT_LIB_NAME}
  urdf
  urdfdom_headers
  visualization_msgs)

target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_robot_model
  moveit_robot_state
)

install(DIRECTORY include/ DESTINATION include)
