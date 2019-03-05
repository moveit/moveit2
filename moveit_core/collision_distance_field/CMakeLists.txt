set(MOVEIT_LIB_NAME moveit_collision_distance_field)

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  # clang is picky about new virtual functions hiding existing ones
  add_compile_options(-Wno-overloaded-virtual)
endif()

add_library(${MOVEIT_LIB_NAME}
  src/collision_distance_field_types.cpp
  src/collision_common_distance_field.cpp
  src/collision_robot_distance_field.cpp
  src/collision_world_distance_field.cpp
  src/collision_robot_hybrid.cpp
  src/collision_world_hybrid.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

ament_target_dependencies(${MOVEIT_LIB_NAME}
  moveit_planning_scene
  moveit_distance_field
  urdf
  visualization_msgs
  tf2_eigen
)

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)

# if(BUILD_TESTING)
#   ament_add_gtest(test_voxel_grid test/test_voxel_grid.cpp)
#   find_package(moveit_resources REQUIRED)
#   include_directories(${moveit_resources_INCLUDE_DIRS})
#
#   amet_add_gtest(test_collision_distance_field test/test_collision_distance_field.cpp)
#   target_link_libraries(test_collision_distance_field  ${MOVEIT_LIB_NAME})
# endif()
#
