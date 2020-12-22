# Trajectory operator interface
add_library(planner_logic_interface SHARED
  src/planner_logic_interface.cpp
)
set_target_properties(planner_logic_interface PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(planner_logic_interface ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Hybrid planning manager component
add_library(moveit_hybrid_planning_manager SHARED src/hybrid_planning_manager.cpp)
set_target_properties(moveit_hybrid_planning_manager PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_hybrid_planning_manager ${THIS_PACKAGE_INCLUDE_DEPENDS})
