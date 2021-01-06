# Local_planner_component
add_library(moveit_local_planner_component SHARED
  src/local_planner_component.cpp
)
set_target_properties(moveit_local_planner_component PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_local_planner_component ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Trajectory operator interface
add_library(trajectory_operator_interface SHARED
  src/trajectory_operator_interface.cpp
)
set_target_properties(trajectory_operator_interface PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(trajectory_operator_interface ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Constraint solver interface
add_library(local_constraint_solver_interface SHARED
  src/local_constraint_solver_interface.cpp
)
set_target_properties(local_constraint_solver_interface PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(local_constraint_solver_interface ${THIS_PACKAGE_INCLUDE_DEPENDS})
