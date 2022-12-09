target_sources(moveit_core PRIVATE
  allvalid/collision_env_allvalid.cpp
  collision_common.cpp
  collision_matrix.cpp
  collision_octomap_filter.cpp
  collision_tools.cpp
  world.cpp
  world_diff.cpp
  collision_env.cpp
  collision_plugin_cache.cpp
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
