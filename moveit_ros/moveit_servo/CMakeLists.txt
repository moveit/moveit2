cmake_minimum_required(VERSION 3.22)
project(moveit_servo LANGUAGES CXX)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    control_msgs
    geometry_msgs
    moveit_core
    moveit_msgs
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    rclcpp
    rclcpp_components
    realtime_tools
    sensor_msgs
    std_msgs
    std_srvs
    tf2_eigen
    trajectory_msgs)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include)

# ##############################################################################
# C++ Libraries ##
# ##############################################################################

# This library provides a way of loading parameters for servo
generate_parameter_library(moveit_servo_lib_parameters
                           config/servo_parameters.yaml)

# This library provides a C++ interface for sending realtime twist or joint
# commands to a robot
add_library(
  moveit_servo_lib_cpp SHARED src/collision_monitor.cpp src/servo.cpp
                              src/utils/common.cpp src/utils/command.cpp)
set_target_properties(moveit_servo_lib_cpp PROPERTIES VERSION
                                                      "${moveit_servo_VERSION}")
target_link_libraries(moveit_servo_lib_cpp moveit_servo_lib_parameters)
ament_target_dependencies(moveit_servo_lib_cpp ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_library(moveit_servo_lib_ros SHARED src/servo_node.cpp)
set_target_properties(moveit_servo_lib_ros PROPERTIES VERSION
                                                      "${moveit_servo_VERSION}")
target_link_libraries(moveit_servo_lib_ros moveit_servo_lib_cpp)
ament_target_dependencies(moveit_servo_lib_ros ${THIS_PACKAGE_INCLUDE_DEPENDS})

# ##############################################################################
# Components ##
# ##############################################################################

rclcpp_components_register_node(moveit_servo_lib_ros PLUGIN
                                "moveit_servo::ServoNode" EXECUTABLE servo_node)

# ##############################################################################
# Executable Nodes ##
# ##############################################################################

# Executable node for the joint jog demo
add_executable(demo_joint_jog demos/cpp_interface/demo_joint_jog.cpp)
target_link_libraries(demo_joint_jog moveit_servo_lib_cpp)
ament_target_dependencies(demo_joint_jog ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Executable node for the twist demo
add_executable(demo_twist demos/cpp_interface/demo_twist.cpp)
target_link_libraries(demo_twist moveit_servo_lib_cpp)
ament_target_dependencies(demo_twist ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Executable node for the pose demo
add_executable(demo_pose demos/cpp_interface/demo_pose.cpp)
target_link_libraries(demo_pose moveit_servo_lib_cpp)
ament_target_dependencies(demo_pose ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Keyboard control example for servo
add_executable(servo_keyboard_input demos/servo_keyboard_input.cpp)
target_include_directories(servo_keyboard_input PUBLIC include)
ament_target_dependencies(servo_keyboard_input ${THIS_PACKAGE_INCLUDE_DEPENDS})

# ##############################################################################
# Install ##
# ##############################################################################

# Install Libraries
install(
  TARGETS moveit_servo_lib_cpp moveit_servo_lib_ros moveit_servo_lib_parameters
  EXPORT moveit_servoTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_servo)

# Install Binaries
install(
  TARGETS demo_joint_jog demo_twist demo_pose servo_node servo_keyboard_input
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/moveit_servo)

# Install include, launch, config directories
install(DIRECTORY include/ DESTINATION include/moveit_servo)
install(DIRECTORY launch DESTINATION share/moveit_servo)
install(DIRECTORY config DESTINATION share/moveit_servo)

ament_export_targets(moveit_servoTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)

  ament_add_gtest_executable(moveit_servo_utils_test tests/test_utils.cpp)
  target_link_libraries(moveit_servo_utils_test moveit_servo_lib_cpp)
  ament_target_dependencies(moveit_servo_utils_test
                            ${THIS_PACKAGE_INCLUDE_DEPENDS})
  add_ros_test(tests/launch/servo_utils.test.py TIMEOUT 30 ARGS
               "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  ament_add_gtest_executable(
    moveit_servo_cpp_integration_test tests/test_integration.cpp
    tests/servo_cpp_fixture.hpp)
  target_link_libraries(moveit_servo_cpp_integration_test moveit_servo_lib_cpp)
  ament_target_dependencies(moveit_servo_cpp_integration_test
                            ${THIS_PACKAGE_INCLUDE_DEPENDS})
  add_ros_test(tests/launch/servo_cpp_integration.test.py TIMEOUT 30 ARGS
               "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  ament_add_gtest_executable(
    moveit_servo_ros_integration_test tests/test_ros_integration.cpp
    tests/servo_ros_fixture.hpp)
  ament_target_dependencies(moveit_servo_ros_integration_test
                            ${THIS_PACKAGE_INCLUDE_DEPENDS})
  add_ros_test(tests/launch/servo_ros_integration.test.py TIMEOUT 120 ARGS
               "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

endif()

ament_package()
