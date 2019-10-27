set(MOVEIT_LIB_NAME moveit_default_planning_request_adapter_plugins)

set(SOURCE_FILES
  src/empty.cpp
  src/fix_start_state_bounds.cpp
  src/fix_start_state_collision.cpp
  src/fix_start_state_path_constraints.cpp
  src/fix_workspace_bounds.cpp
  src/add_time_parameterization.cpp
  src/add_iterative_spline_parameterization.cpp
  src/add_time_optimal_parameterization.cpp
  src/resolve_constraint_frames.cpp
)

add_library(${MOVEIT_LIB_NAME} ${SOURCE_FILES})

set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME}
  ${Boost_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${moveit_core_LIBRARIES}
)


install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

add_executable(moveit_list_request_adapter_plugins src/list.cpp)

target_link_libraries(moveit_list_request_adapter_plugins
  ${Boost_LIBRARIES}
  ${pluginlib_LIBRARIES}
  ${rclcpp_LIBRARIES}
)

install(TARGETS ${MOVEIT_LIB_NAME} moveit_list_request_adapter_plugins
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
