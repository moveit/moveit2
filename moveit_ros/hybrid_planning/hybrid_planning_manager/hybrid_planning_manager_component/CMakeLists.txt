# Hybrid planning interface
add_library(moveit_hybrid_planning_interface SHARED src/hybrid_planning_interface.cpp)
set_target_properties(moveit_hybrid_planning_interface PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_hybrid_planning_interface ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Hybrid planning manager component
add_library(moveit_hybrid_planning_manager SHARED src/hybrid_planning_manager.cpp)
set_target_properties(moveit_hybrid_planning_manager PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_hybrid_planning_manager ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(moveit_hybrid_planning_manager moveit_hybrid_planning_interface)
