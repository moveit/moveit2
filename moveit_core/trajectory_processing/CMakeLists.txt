target_sources(moveit_core PRIVATE
  src/ruckig_traj_smoothing.cpp
  src/trajectory_tools.cpp
  src/time_optimal_trajectory_generation.cpp
)
target_include_directories(moveit_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(DIRECTORY include/ DESTINATION include/moveit_core)

if(BUILD_TESTING)
  if(WIN32)
    # TODO add windows paths
    # set(append_library_dirs "$<TARGET_FILE_DIR:${PROJECT_NAME}>;$<TARGET_FILE_DIR:${PROJECT_NAME}_TestPlugins1>")
  else()
    set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR};${CMAKE_CURRENT_BINARY_DIR}/../robot_trajectory;${CMAKE_CURRENT_BINARY_DIR}/../utils")
  endif()

  ament_add_gtest(test_time_optimal_trajectory_generation test/test_time_optimal_trajectory_generation.cpp)
  target_link_libraries(test_time_optimal_trajectory_generation moveit::core)

  ament_add_gtest(test_ruckig_traj_smoothing test/test_ruckig_traj_smoothing.cpp)
  target_link_libraries(test_ruckig_traj_smoothing moveit::core)
endif()
