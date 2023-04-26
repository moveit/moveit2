add_library(moveit_cost_functions SHARED
  src/cost_functions.cpp
)
target_include_directories(moveit_cost_functions PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/moveit_core>
)
set_target_properties(moveit_cost_functions PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(moveit_cost_functions
  moveit_planning_interface
  moveit_planning_scene
  moveit_robot_state
)

install(DIRECTORY include/ DESTINATION include/moveit_core)


#if(BUILD_TESTING)
# TODO(sjahr): Add tests
#endif()
