target_sources(moveit_core PRIVATE
  src/collision_distance_field_types.cpp
  src/collision_common_distance_field.cpp
  src/collision_env_distance_field.cpp
  src/collision_env_hybrid.cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(resource_retriever REQUIRED)

  ament_add_gtest(test_collision_distance_field test/test_collision_distance_field.cpp)
  ament_target_dependencies(test_collision_distance_field
    geometric_shapes
    OCTOMAP
    srdfdom
    resource_retriever
  )
  target_link_libraries(test_collision_distance_field moveit::core)
endif()
