set(MOVEIT_LIB_NAME moveit_distance_field)

add_library(${MOVEIT_LIB_NAME}
  src/distance_field.cpp
  src/find_internal_points.cpp
  src/propagation_distance_field.cpp
  )
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

ament_target_dependencies(${MOVEIT_LIB_NAME}
  urdfdom
  urdfdom_headers
  visualization_msgs
  tf2_eigen)

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   ament_add_gtest(test_voxel_grid test/test_voxel_grid.cpp)
#   ament_target_dependencies(test_voxel_grid
#     ${MOVEIT_LIB_NAME})
#   ament_add_gtest(test_distance_field test/test_distance_field.cpp)
#   ament_target_dependencies(test_distance_field
#     ${MOVEIT_LIB_NAME}
#     visualization_msgs
#     tf2_eigen)
# endif()
