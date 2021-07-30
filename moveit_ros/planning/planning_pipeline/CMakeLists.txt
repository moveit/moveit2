set(MOVEIT_LIB_NAME moveit_planning_pipeline)

add_library(${MOVEIT_LIB_NAME} SHARED src/planning_pipeline.cpp)
include(GenerateExportHeader)
generate_export_header(${MOVEIT_LIB_NAME})
target_include_directories(${MOVEIT_LIB_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

ament_target_dependencies(${MOVEIT_LIB_NAME}
  moveit_core
  moveit_msgs
  rclcpp
  Boost
  pluginlib
)

install(DIRECTORY include/ DESTINATION include)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${MOVEIT_LIB_NAME}_export.h DESTINATION include)
