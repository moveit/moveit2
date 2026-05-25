add_library(moveit_transforms SHARED src/transforms.cpp)
target_include_directories(
  moveit_transforms
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/moveit_core>)
target_link_libraries(
  moveit_transforms
  moveit_macros
  moveit_utils
  geometric_shapes::geometric_shapes
  tf2_eigen::tf2_eigen
  rclcpp::rclcpp
  rmw_implementation::rmw_implementation
  urdfdom::urdf_parser
  urdfdom_headers::urdfdom_headers
  Boost::headers)
set_target_properties(moveit_transforms PROPERTIES VERSION
                                                   "${${PROJECT_NAME}_VERSION}")

install(DIRECTORY include/ DESTINATION include/moveit_core)

# Unit tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_transforms test/test_transforms.cpp)
  target_link_libraries(test_transforms moveit_transforms)
endif()
