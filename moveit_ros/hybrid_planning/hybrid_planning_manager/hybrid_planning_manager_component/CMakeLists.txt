# Hybrid planning manager component
add_library(moveit_hybrid_planning_manager SHARED src/hybrid_planning_manager.cpp)
set_target_properties(moveit_hybrid_planning_manager PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_hybrid_planning_manager ${THIS_PACKAGE_INCLUDE_DEPENDS})
