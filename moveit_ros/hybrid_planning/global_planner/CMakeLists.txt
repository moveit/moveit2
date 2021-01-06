set(MOVEIT_LIB_NAME moveit_global_planner_component)

add_library(${MOVEIT_LIB_NAME} SHARED src/global_planner_component.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})