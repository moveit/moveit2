add_library(srdf_publisher_node SHARED src/srdf_publisher_node.cpp)
target_link_libraries(
  srdf_publisher_node
  PUBLIC ${std_msgs_TARGETS} rclcpp::rclcpp rclcpp_components::component
         rclcpp_components::component_manager)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(test/srdf_publisher_test.py TARGET test-srdf_publisher)
endif()
