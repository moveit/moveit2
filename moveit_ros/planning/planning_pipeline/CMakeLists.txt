set(MOVEIT_LIB_NAME moveit_planning_pipeline)

add_library(${MOVEIT_LIB_NAME} SHARED src/planning_pipeline.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${MOVEIT_LIB_NAME}
  PRIVATE "MOVEIT_PLANNING_PIPELINE_BUILDING_LIBRARY")

ament_target_dependencies(${MOVEIT_LIB_NAME}
  moveit_core
  moveit_msgs
  rclcpp
  Boost
  pluginlib
)

install(DIRECTORY include/ DESTINATION include)
