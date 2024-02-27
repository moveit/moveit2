add_library(srdf_publisher_node SHARED src/srdf_publisher_node.cpp)
ament_target_dependencies(srdf_publisher_node PUBLIC std_msgs rclcpp
                          rclcpp_components)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(test/srdf_publisher_test.py TARGET test-srdf_publisher)
endif()
