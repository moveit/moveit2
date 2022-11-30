add_library(moveit_trajectory_processing SHARED
  src/iterative_time_parameterization.cpp
  src/iterative_spline_parameterization.cpp
  src/ruckig_traj_smoothing.cpp
  src/trajectory_tools.cpp
  src/time_optimal_trajectory_generation.cpp
)

set_target_properties(moveit_trajectory_processing PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_trajectory_processing
  rclcpp
  rmw_implementation
  urdf
  urdfdom
  urdfdom_headers
  visualization_msgs
  Boost
)
target_link_libraries(moveit_trajectory_processing
  moveit_robot_state
  moveit_robot_trajectory
  ruckig::ruckig
)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  if(WIN32)
    # TODO add windows paths
    # set(append_library_dirs "$<TARGET_FILE_DIR:${PROJECT_NAME}>;$<TARGET_FILE_DIR:${PROJECT_NAME}_TestPlugins1>")
  else()
    set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR};${CMAKE_CURRENT_BINARY_DIR}/../robot_trajectory;${CMAKE_CURRENT_BINARY_DIR}/../utils")
  endif()

  ament_add_gtest(test_time_parameterization test/test_time_parameterization.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  target_link_libraries(test_time_parameterization moveit_test_utils moveit_trajectory_processing)

  ament_add_gtest(test_time_optimal_trajectory_generation test/test_time_optimal_trajectory_generation.cpp)
  target_link_libraries(test_time_optimal_trajectory_generation moveit_test_utils moveit_trajectory_processing)

  ament_add_gtest(test_ruckig_traj_smoothing test/test_ruckig_traj_smoothing.cpp)
  target_link_libraries(test_ruckig_traj_smoothing
    moveit_trajectory_processing
    moveit_test_utils
  )
endif()
