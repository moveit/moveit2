add_library(moveit_dynamics_solver SHARED src/dynamics_solver.cpp)
set_target_properties(moveit_dynamics_solver PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

ament_target_dependencies(moveit_dynamics_solver
  urdf
  urdfdom_headers
  orocos_kdl
  visualization_msgs
  kdl_parser
)
target_link_libraries(moveit_dynamics_solver
  moveit_robot_state
)

install(DIRECTORY include/ DESTINATION include)
