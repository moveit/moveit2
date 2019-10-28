set(MOVEIT_LIB_NAME moveit_transforms)

add_library(${MOVEIT_LIB_NAME} SHARED src/transforms.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME} ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})
ament_target_dependencies(${MOVEIT_LIB_NAME}
            tf2_eigen)


install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

# Unit tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_transforms test/test_transforms.cpp)
  target_link_libraries(test_transforms ${MOVEIT_LIB_NAME})
endif()
