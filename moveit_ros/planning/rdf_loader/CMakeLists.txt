add_library(moveit_rdf_loader SHARED src/rdf_loader.cpp src/synchronized_string_parameter.cpp)
set_target_properties(moveit_rdf_loader PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_rdf_loader
      rclcpp
      ament_index_cpp
      urdf
      srdfdom
      moveit_core)

install(DIRECTORY include/ DESTINATION include/moveit_ros_planning)
install(DIRECTORY
            test/data
            test/launch
        DESTINATION share/${PROJECT_NAME}/rdf_loader/test
)

# TODO(sjahr) Debug and re-enable
#if(BUILD_TESTING)
#  find_package(ament_cmake_gtest REQUIRED)
#  find_package(ros_testing REQUIRED)
#
#  ament_add_gtest_executable(test_rdf_integration
#      test/test_rdf_integration.cpp
#  )
#  target_link_libraries(test_rdf_integration moveit_rdf_loader)
#  add_ros_test(test/launch/test_rdf_integration.test.py TIMEOUT 120)
#
#  add_executable(boring_string_publisher test/boring_string_publisher.cpp)
#  target_link_libraries(boring_string_publisher moveit_rdf_loader)
#
#  install(
#    TARGETS
#      test_rdf_integration boring_string_publisher
#    ARCHIVE DESTINATION lib
#    LIBRARY DESTINATION lib
#    RUNTIME DESTINATION lib/${PROJECT_NAME}
#  )
#endif()
