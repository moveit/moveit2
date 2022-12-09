target_sources(moveit_core PRIVATE planning_scene.cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  if(UNIX OR APPLE)
    set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}:${CMAKE_CURRENT_BINARY_DIR}/../utils:${CMAKE_CURRENT_BINARY_DIR}/../collision_detection_fcl:${CMAKE_CURRENT_BINARY_DIR}/../collision_detection")
  endif()

  ament_add_gtest(test_planning_scene test/test_planning_scene.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  ament_target_dependencies(test_planning_scene
    geometric_shapes
    srdfdom
  )
  target_link_libraries(test_planning_scene moveit::core)

  ament_add_gtest(test_collision_objects test/test_collision_objects.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  target_link_libraries(test_collision_objects moveit::core)

  ament_add_gtest(test_multi_threaded test/test_multi_threaded.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  target_link_libraries(test_multi_threaded moveit::core)
endif()
