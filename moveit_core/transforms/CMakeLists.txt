add_library(moveit_transforms SHARED src/transforms.cpp)
target_include_directories(
  moveit_transforms
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/moveit_core>)

set_target_properties(moveit_transforms PROPERTIES VERSION
                                                   "${${PROJECT_NAME}_VERSION}")
target_link_libraries(moveit_transforms
      PRIVATE
      moveit_macros moveit_utils
      PUBLIC
      geometric_shapes::geometric_shapes
      rclcpp::rclcpp
      rmw_implementation::rmw_implementation
      tf2_eigen::tf2_eigen
      urdfdom::urdfdom_model
      urdfdom::urdfdom_sensor
      urdfdom::urdfdom_world
      urdfdom_headers::urdfdom_headers
    )

install(DIRECTORY include/ DESTINATION include/moveit_core)

# Unit tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_transforms test/test_transforms.cpp)
  target_link_libraries(test_transforms moveit_transforms moveit_macros)
endif()
