# Local_planner_component
generate_parameter_library(local_planner_parameters
                           res/local_planner_parameters.yaml)
add_library(moveit_local_planner_component SHARED
            src/local_planner_component.cpp)
set_target_properties(moveit_local_planner_component
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(moveit_local_planner_component local_planner_parameters)
ament_target_dependencies(moveit_local_planner_component
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
