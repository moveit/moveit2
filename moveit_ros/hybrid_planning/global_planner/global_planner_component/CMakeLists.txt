# Global planner component
add_library(moveit_global_planner_component SHARED
            src/global_planner_component.cpp)
set_target_properties(moveit_global_planner_component
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_global_planner_component
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
