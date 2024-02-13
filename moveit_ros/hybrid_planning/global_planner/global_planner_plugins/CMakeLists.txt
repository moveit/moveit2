add_library(motion_planning_pipeline_plugin SHARED
            src/moveit_planning_pipeline.cpp)
set_target_properties(motion_planning_pipeline_plugin
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(motion_planning_pipeline_plugin
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
