set(MOVEIT_LIB_NAME moveit_hybrid_planning_manager)

add_library(${MOVEIT_LIB_NAME} SHARED src/hybrid_planning_manager.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})