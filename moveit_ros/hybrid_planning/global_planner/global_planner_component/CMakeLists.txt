# Global planner component
add_library(moveit_global_planner_component SHARED
            src/global_planner_component.cpp)
set_target_properties(moveit_global_planner_component
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(moveit_global_planner_component PUBLIC
                      ${THIS_PACKAGE_INCLUDE_DEPENDS})
