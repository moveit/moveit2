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
ament_add_gtest(test_logger test_logger.cpp TIMEOUT 60)
target_link_libraries(test_logger moveit_utils)

# The executable does not link moveit_utils: it loads a bridge which links the
# actual library, exercises one logger entry point, then drops that loader
# reference. Each combination runs in its own process for static logger state.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  add_library(logger_unload_dut SHARED logger_unload_dut.cpp)
  target_link_libraries(logger_unload_dut moveit_utils)
  ament_add_gtest_executable(test_logger_unload test_logger_unload.cpp)
  target_link_libraries(test_logger_unload rclcpp::rclcpp ${CMAKE_DL_LIBS})
  target_compile_definitions(
    test_logger_unload
    PRIVATE MOVEIT_UTILS_PATH="$<TARGET_FILE:moveit_utils>"
            LOGGER_UNLOAD_DUT_PATH="$<TARGET_FILE:logger_unload_dut>")
  foreach(entry get set)
    foreach(order unload-first shutdown-first no-unload)
      ament_add_gtest_test(
        test_logger_unload
        TEST_NAME
        test_logger_unload_${entry}_${order}
        TIMEOUT
        60
        ENV
        "MOVEIT_LOGGER_UNLOAD_ENTRY=${entry}"
        "MOVEIT_LOGGER_UNLOAD_ORDER=${order}")
    endforeach()
  endforeach()
endif()

# Regression test for getGlobalRootLogger()'s before-rclcpp::init() fallback.
# Deliberately its own single-test executable/process -- see the file comment in
# test_logger_before_init.cpp for why this can't be a test case inside
# test_logger.cpp above.
ament_add_gtest(test_logger_before_init test_logger_before_init.cpp
                SKIP_LINKING_MAIN_LIBRARIES)
target_link_libraries(test_logger_before_init moveit_utils)

find_package(launch_testing_ament_cmake)

# These tests do not work on Humble as /rosout logging from child loggers does
# not work. ROS_DISTRO is not defined on rolling
if((NOT DEFINED ENV{ROS_DISTRO}) OR (NOT $ENV{ROS_DISTRO} STREQUAL "humble"))
  add_launch_test(rosout_publish_test.py TARGET test-node_logging ARGS
                  "dut:=logger_dut")
endif()
