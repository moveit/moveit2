set(MOVEIT_LIB_NAME moveit_collision_detection)

add_library(${MOVEIT_LIB_NAME} SHARED
  src/allvalid/collision_env_allvalid.cpp
  src/collision_matrix.cpp
  src/collision_octomap_filter.cpp
  src/collision_tools.cpp
  src/world.cpp
  src/world_diff.cpp
  src/collision_env.cpp
)

set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
ament_target_dependencies(${MOVEIT_LIB_NAME}
  rclcpp
  rmw_implementation
  urdf
  urdfdom
  urdfdom_headers
  visualization_msgs
  tf2_eigen
  geometric_shapes
  Boost
  octomap
)

target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_robot_state
)

# unit tests
if(BUILD_TESTING)
  find_package(resource_retriever REQUIRED)
  if(WIN32)
    # TODO add window paths
  else()
    set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR};${CMAKE_CURRENT_BINARY_DIR}/../robot_model;${CMAKE_CURRENT_BINARY_DIR}/../utils")
  endif()

  ament_add_gtest(test_world test/test_world.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  target_link_libraries(test_world
    ${MOVEIT_LIB_NAME}
    ${urdfdom}
    ${urdfdom_headers}
    ${Boost_LIBRARIES}
    ${geometric_shapes_LIBRARIES}
  )

  ament_add_gtest(test_world_diff test/test_world_diff.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  target_link_libraries(test_world_diff
    ${MOVEIT_LIB_NAME}
    ${urdfdom}
    ${urdfdom_headers}
    ${Boost_LIBRARIES}
    ${geometric_shapes_LIBRARIES}
  )

  ament_add_gtest(test_all_valid test/test_all_valid.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  target_link_libraries(test_all_valid
    ${MOVEIT_LIB_NAME}
    moveit_robot_model
    ${urdfdom}
    ${urdfdom_headers}
    ${Boost_LIBRARIES}
    ${geometric_shapes_LIBRARIES}
    resource_retriever::resource_retriever
  )
endif()

install(TARGETS ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
