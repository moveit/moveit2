set(MOVEIT_LIB_NAME moveit_plan_execution)

add_library(${MOVEIT_LIB_NAME} SHARED
  src/plan_with_sensing.cpp
  src/plan_execution.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME}
    moveit_planning_pipeline
    moveit_planning_scene_monitor
    moveit_trajectory_execution_manager
)
ament_target_dependencies(${MOVEIT_LIB_NAME}
  moveit_core
  rclcpp
  Boost
  class_loader
  pluginlib
)

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)
