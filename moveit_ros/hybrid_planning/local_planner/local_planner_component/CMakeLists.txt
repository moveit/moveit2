# Local_planner_component
add_library(moveit_local_planner_component SHARED
  src/local_planner_component.cpp
)
set_target_properties(moveit_local_planner_component PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_local_planner_component ${THIS_PACKAGE_INCLUDE_DEPENDS})
