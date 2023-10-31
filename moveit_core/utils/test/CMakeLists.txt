# Build devices under test
add_executable(logger_dut logger_dut.cpp)
target_link_libraries(logger_dut rclcpp::rclcpp moveit_utils)

add_executable(logger_from_child_dut logger_from_child_dut.cpp)
target_link_libraries(logger_from_child_dut rclcpp::rclcpp moveit_utils)

add_executable(logger_from_only_child_dut logger_from_only_child_dut.cpp)
target_link_libraries(logger_from_only_child_dut rclcpp::rclcpp moveit_utils)

# Install is needed to run these as launch tests
install(
  TARGETS
    logger_dut
    logger_from_child_dut
    logger_from_only_child_dut
  DESTINATION lib/${PROJECT_NAME}
)

find_package(launch_testing_ament_cmake)

# Test node logger to rosout
add_launch_test(rosout_publish_test.py
    TARGET test-node_logging
    ARGS "dut:=logger_dut"
)

# These tests do not work on Humble as /rosout logging from child loggers
# does not work.
# ROS_DISTRO is not defined on rolling
if((NOT DEFINED ENV{ROS_DISTRO}) OR
   (NOT $ENV{ROS_DISTRO} STREQUAL "humble"))
  # Test init node logging then log from child logger to rosout
  add_launch_test(rosout_publish_test.py
      TARGET test-node_logging_from_child
      ARGS "dut:=logger_from_child_dut"
  )

  # Test init only creating child logger and logging goes to rosout
  add_launch_test(rosout_publish_test.py
      TARGET test-logger_from_only_child_dut
      ARGS "dut:=logger_from_only_child_dut"
  )
endif()
