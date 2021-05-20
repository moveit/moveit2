add_library(forward_trajectory_plugin SHARED
  src/forward_trajectory.cpp
)
set_target_properties(forward_trajectory_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(forward_trajectory_plugin ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(forward_trajectory_plugin local_constraint_solver_interface)

add_library(servo_solver_plugin SHARED
  src/servo_solver.cpp
)
set_target_properties(servo_solver_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(servo_solver_plugin ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(servo_solver_plugin local_constraint_solver_interface)
