
find_package(launch_testing_ament_cmake)

add_executable(logger_test_node logger_test_node.cpp)
target_link_libraries(logger_test_node rclcpp::rclcpp moveit_utils)
install(TARGETS logger_test_node
    DESTINATION lib/${PROJECT_NAME})


# Runs logget_test_node and observes ros logging
add_launch_test(logger_launch_test.py)
