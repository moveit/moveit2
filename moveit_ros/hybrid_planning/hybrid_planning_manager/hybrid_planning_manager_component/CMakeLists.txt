# Hybrid planning manager component
generate_parameter_library(hp_manager_parameters res/hp_manager_parameters.yaml)
add_library(moveit_hybrid_planning_manager SHARED
            src/hybrid_planning_manager.cpp)
set_target_properties(moveit_hybrid_planning_manager
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(moveit_hybrid_planning_manager hp_manager_parameters)
ament_target_dependencies(moveit_hybrid_planning_manager
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
