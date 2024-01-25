# Forward Trajectory Solver
add_library(forward_trajectory_plugin SHARED src/forward_trajectory.cpp)
set_target_properties(forward_trajectory_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(forward_trajectory_plugin ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Servo Local Solver
add_library(servo_local_solver_plugin SHARED src/servo_solver.cpp)
set_target_properties(servo_local_solver_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(servo_local_solver_plugin ${THIS_PACKAGE_INCLUDE_DEPENDS})
