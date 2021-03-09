set(MOVEIT_LIB_NAME moveit_dynamics_solver)

add_library(${MOVEIT_LIB_NAME} SHARED src/dynamics_solver.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

ament_target_dependencies(${MOVEIT_LIB_NAME}
  urdf
  urdfdom_headers
  orocos_kdl
  visualization_msgs
  kdl_parser
)
target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_robot_state
)

install(DIRECTORY include/ DESTINATION include)
