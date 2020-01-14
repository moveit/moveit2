set(MOVEIT_LIB_NAME moveit_trajectory_processing)

add_library(${MOVEIT_LIB_NAME} SHARED
  src/iterative_time_parameterization.cpp
  src/iterative_spline_parameterization.cpp
  src/trajectory_tools.cpp
  src/time_optimal_trajectory_generation.cpp
)

set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
  rclcpp
  rmw_implementation
  urdf
  urdfdom
  urdfdom_headers
  visualization_msgs
  Boost
)
target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_robot_state
  moveit_robot_trajectory
)

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(moveit_resources REQUIRED)
  include_directories(${moveit_resources_INCLUDE_DIRS})
  find_package(resource_retriever REQUIRED)

  if(WIN32)
    # TODO add windows paths
    # set(append_library_dirs "$<TARGET_FILE_DIR:${PROJECT_NAME}>;$<TARGET_FILE_DIR:${PROJECT_NAME}_TestPlugins1>")
  else()
    set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR};${CMAKE_CURRENT_BINARY_DIR}/../robot_trajectory;${CMAKE_CURRENT_BINARY_DIR}/../utils")
  endif()

  ament_add_gtest(test_time_parameterization test/test_time_parameterization.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")

  target_include_directories(test_time_parameterization PUBLIC
    ${geometric_shapes_INCLUDE_DIRS}
  )

  target_link_libraries(test_time_parameterization
    moveit_test_utils
    moveit_robot_trajectory
    moveit_trajectory_processing
    ${urdfdom_LIBRARIES}
    ${srdfdom_LIBRARIES}
    ${urdfdom_headers_LIBRARIES}
    ${MOVEIT_LIB_NAME}
    ${geometric_shapes_LIBRARIES}
    resource_retriever::resource_retriever
  )
endif()
