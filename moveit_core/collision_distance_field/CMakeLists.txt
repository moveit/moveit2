set(MOVEIT_LIB_NAME moveit_collision_distance_field)

add_library(${MOVEIT_LIB_NAME} SHARED
  src/collision_distance_field_types.cpp
  src/collision_common_distance_field.cpp
  src/collision_env_distance_field.cpp
  src/collision_env_hybrid.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

ament_target_dependencies(${MOVEIT_LIB_NAME}
  urdf
  visualization_msgs
  tf2_eigen
  geometric_shapes
  OCTOMAP
)

target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_planning_scene
  moveit_distance_field
  moveit_collision_detection
  moveit_robot_state
)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(resource_retriever REQUIRED)

  if(WIN32)
    # TODO add windows paths
  else()
    set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR};${CMAKE_CURRENT_BINARY_DIR}/../planning_scene;${CMAKE_CURRENT_BINARY_DIR}/../distance_field;${CMAKE_CURRENT_BINARY_DIR}/../collision_detection;${CMAKE_CURRENT_BINARY_DIR}/../robot_state;${CMAKE_CURRENT_BINARY_DIR}/../robot_model;${CMAKE_CURRENT_BINARY_DIR}/../utils")
  endif()

  ament_add_gtest(test_collision_distance_field test/test_collision_distance_field.cpp)
  target_link_libraries(test_collision_distance_field
    ${MOVEIT_LIB_NAME}
    moveit_collision_detection
    moveit_collision_distance_field
    moveit_robot_state
    moveit_test_utils
    ${geometric_shapes_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
    ${srdfdom_LIBRARIES}
    resource_retriever::resource_retriever
    moveit_distance_field
    moveit_planning_scene
  )
endif()
