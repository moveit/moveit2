# Build devices under test
add_executable(logger_dut logger_dut.cpp)
target_link_libraries(logger_dut rclcpp::rclcpp moveit_utils)

# Install is needed to for launchtest to execute
install(TARGETS logger_dut DESTINATION lib/${PROJECT_NAME})

find_package(launch_testing_ament_cmake)

# These tests do not work on Humble as /rosout logging from child loggers does
# not work. ROS_DISTRO is not defined on rolling
if((NOT DEFINED ENV{ROS_DISTRO}) OR (NOT $ENV{ROS_DISTRO} STREQUAL "humble"))
  add_launch_test(rosout_publish_test.py TARGET test-node_logging ARGS
                  "dut:=logger_dut")
endif()
