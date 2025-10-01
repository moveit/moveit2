add_library(moveit_dynamics_solver SHARED src/dynamics_solver.cpp)
target_include_directories(
  moveit_dynamics_solver
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/moveit_core>)
set_target_properties(moveit_dynamics_solver
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(
  moveit_dynamics_solver
  PUBLIC moveit_robot_state moveit_utils ${visualization_msgs_TARGETS}
         kdl_parser::kdl_parser urdf::urdf)

install(DIRECTORY include/ DESTINATION include/moveit_core)
