target_sources(moveit_core PRIVATE src/dynamics_solver.cpp)
target_include_directories(moveit_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(moveit_core
  urdf
  urdfdom_headers
  orocos_kdl
  visualization_msgs
  kdl_parser
)

install(DIRECTORY include/ DESTINATION include/moveit_core)
