set(MOVEIT_LIB_NAME moveit_collision_distance_field)

add_library(${MOVEIT_LIB_NAME} SHARED
  src/collision_distance_field_types.cpp
  src/collision_common_distance_field.cpp
  src/collision_env_distance_field.cpp
  src/collision_env_hybrid.cpp
)
include(GenerateExportHeader)
generate_export_header(${MOVEIT_LIB_NAME})
target_include_directories(${MOVEIT_LIB_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
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
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${MOVEIT_LIB_NAME}_export.h DESTINATION include)

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
    ${MOVEIT_LIB_NAME}
    moveit_collision_detection
    moveit_distance_field
    moveit_robot_model
    moveit_robot_state
    moveit_test_utils
    moveit_transforms
    moveit_planning_scene
  )
endif()
