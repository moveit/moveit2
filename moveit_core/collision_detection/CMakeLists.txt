target_sources(moveit_core PRIVATE
  src/allvalid/collision_env_allvalid.cpp
  src/collision_common.cpp
  src/collision_matrix.cpp
  src/collision_octomap_filter.cpp
  src/collision_tools.cpp
  src/world.cpp
  src/world_diff.cpp
  src/collision_env.cpp
  src/collision_plugin_cache.cpp
)
target_include_directories(moveit_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# unit tests
if(BUILD_TESTING)
  if(WIN32)
    # TODO add window paths
  else()
    set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR};${CMAKE_CURRENT_BINARY_DIR}/../robot_model;${CMAKE_CURRENT_BINARY_DIR}/../utils")
  endif()

  ament_add_gtest(test_world test/test_world.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  target_link_libraries(test_world moveit::core)

  ament_add_gtest(test_world_diff test/test_world_diff.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  target_link_libraries(test_world_diff moveit::core)

  ament_add_gtest(test_all_valid test/test_all_valid.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  target_link_libraries(test_all_valid moveit::core)
endif()

install(DIRECTORY include/ DESTINATION include/moveit_core)
