# Build devices under test
add_executable(logger_dut logger_dut.cpp)
target_link_libraries(logger_dut rclcpp::rclcpp moveit_utils)

add_executable(logger_from_child_dut logger_from_child_dut.cpp)
target_link_libraries(logger_from_child_dut rclcpp::rclcpp moveit_utils)

# Install is needed to run these as launch tests
install(
  TARGETS
    logger_dut
    logger_from_child_dut
  DESTINATION lib/${PROJECT_NAME}
)

# Add the launch tests
find_package(launch_testing_ament_cmake)
add_launch_test(rosout_publish_test.py
    TARGET test-node_logging
    ARGS "dut:=logger_dut"
)
add_launch_test(rosout_publish_test.py
    TARGET test-node_logging_from_child
    ARGS "dut:=logger_from_child_dut"
)
