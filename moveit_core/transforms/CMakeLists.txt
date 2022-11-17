add_library(moveit_transforms SHARED src/transforms.cpp)
set_target_properties(moveit_transforms PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_transforms
  geometric_shapes
  tf2_eigen
  rclcpp
  rmw_implementation
  urdfdom
  urdfdom_headers
  Boost
)

install(DIRECTORY include/ DESTINATION include)

# Unit tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_transforms test/test_transforms.cpp)
  target_link_libraries(test_transforms moveit_transforms)
endif()
