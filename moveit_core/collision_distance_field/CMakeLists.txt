add_library(moveit_collision_distance_field SHARED
  src/collision_distance_field_types.cpp
  src/collision_common_distance_field.cpp
  src/collision_env_distance_field.cpp
  src/collision_env_hybrid.cpp
)
target_include_directories(moveit_collision_distance_field PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/moveit_core>
)
target_include_directories(moveit_collision_distance_field PUBLIC $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>)

ament_target_dependencies(moveit_collision_distance_field
  urdf
  visualization_msgs
  tf2_eigen
  geometric_shapes
  OCTOMAP
)

target_link_libraries(moveit_collision_distance_field
  moveit_planning_scene
  moveit_distance_field
  moveit_collision_detection
  moveit_robot_state
)

install(DIRECTORY include/ DESTINATION include/moveit_core)

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
  target_link_libraries(test_collision_distance_field
    moveit_collision_distance_field
    moveit_collision_detection
    moveit_distance_field
    moveit_robot_model
    moveit_robot_state
    moveit_test_utils
    moveit_transforms
    moveit_planning_scene
  )
endif()
