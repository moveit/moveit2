add_library(single_plan_execution_plugin SHARED src/single_plan_execution.cpp)
set_target_properties(single_plan_execution_plugin
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(single_plan_execution_plugin
                      PUBLIC moveit_hybrid_planning_manager)

add_library(replan_invalidated_trajectory_plugin SHARED
            src/replan_invalidated_trajectory.cpp)
set_target_properties(replan_invalidated_trajectory_plugin
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  replan_invalidated_trajectory_plugin PUBLIC moveit_hybrid_planning_manager
                                              single_plan_execution_plugin)
