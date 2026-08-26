# Build devices under test
add_executable(logger_dut logger_dut.cpp)
target_link_libraries(logger_dut rclcpp::rclcpp moveit_utils)

# Install is needed to for launchtest to execute
install(TARGETS logger_dut DESTINATION lib/${PROJECT_NAME})

# Unit test for the logger's node lifecycle (regression test for
# moveit/moveit2#3827). Links against the real moveit_utils library; reaches
# registerNodeResetOnPreShutdown() via the internal (non-installed)
# src/logger_detail.hpp shared with logger.cpp, without exposing it through the
# public moveit/utils/logger.hpp API.
find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(test_logger test_logger.cpp)
target_link_libraries(test_logger moveit_utils)

find_package(launch_testing_ament_cmake)

# These tests do not work on Humble as /rosout logging from child loggers does
# not work. ROS_DISTRO is not defined on rolling
if((NOT DEFINED ENV{ROS_DISTRO}) OR (NOT $ENV{ROS_DISTRO} STREQUAL "humble"))
  add_launch_test(rosout_publish_test.py TARGET test-node_logging ARGS
                  "dut:=logger_dut")
endif()
