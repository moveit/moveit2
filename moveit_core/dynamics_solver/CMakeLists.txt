add_library(moveit_dynamics_solver SHARED src/dynamics_solver.cpp)
target_include_directories(
  moveit_dynamics_solver
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/moveit_core>)
set_target_properties(moveit_dynamics_solver
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

ament_target_dependencies(moveit_dynamics_solver urdf urdfdom_headers
                          orocos_kdl visualization_msgs kdl_parser)
target_link_libraries(moveit_dynamics_solver moveit_robot_state moveit_utils)

install(DIRECTORY include/ DESTINATION include/moveit_core)
