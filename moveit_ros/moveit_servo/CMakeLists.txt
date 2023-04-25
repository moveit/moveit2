cmake_minimum_required(VERSION 3.22)
project(moveit_servo LANGUAGES CXX)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  control_toolbox
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  pluginlib
  rclcpp
  rclcpp_components
  realtime_tools
  sensor_msgs
  std_msgs
  std_srvs
  tf2_eigen
  trajectory_msgs
)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
  include
)

###################
## C++ Libraries ##
###################

# This library provides a way of loading parameters for servo
generate_parameter_library(
  moveit_servo_lib_parameters
  src/servo_parameters.yaml
)

# This library provides a C++ interface for sending realtime twist or joint commands to a robot
add_library(moveit_servo_lib SHARED
  src/collision_check.cpp
  src/enforce_limits.cpp
  src/servo.cpp
  src/servo_calcs.cpp
  src/utilities.cpp
)
set_target_properties(moveit_servo_lib PROPERTIES VERSION "${moveit_servo_VERSION}")
ament_target_dependencies(moveit_servo_lib ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(moveit_servo_lib moveit_servo_lib_parameters)

add_library(pose_tracking SHARED src/pose_tracking.cpp)
ament_target_dependencies(pose_tracking ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(pose_tracking moveit_servo_lib)

#####################
## Component Nodes ##
#####################

# Add and export library to run as a ROS node component, and receive commands via topics
add_library(servo_node SHARED src/servo_node.cpp)
ament_target_dependencies(servo_node ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(servo_node moveit_servo_lib)
rclcpp_components_register_nodes(servo_node "moveit_servo::ServoNode")

# Add executable for using a controller
add_library(servo_controller_input SHARED src/teleop_demo/joystick_servo_example.cpp)
ament_target_dependencies(servo_controller_input PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(servo_controller_input "moveit_servo::JoyToServoPub")

######################
## Executable Nodes ##
######################

# An executable node for the servo server
add_executable(servo_node_main src/servo_node_main.cpp)
target_link_libraries(servo_node_main servo_node)
ament_target_dependencies(servo_node_main ${THIS_PACKAGE_INCLUDE_DEPENDS})

# An example of pose tracking
add_executable(servo_pose_tracking_demo src/cpp_interface_demo/pose_tracking_demo.cpp)
target_link_libraries(servo_pose_tracking_demo pose_tracking)
ament_target_dependencies(servo_pose_tracking_demo ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Add executable to publish fake servo commands for testing/demo purposes
add_executable(fake_command_publisher test/publish_fake_jog_commands.cpp)
ament_target_dependencies(fake_command_publisher
  rclcpp
  geometry_msgs
  std_srvs
)

#############
## Install ##
#############

# Install Libraries
install(
  TARGETS
    moveit_servo_lib
    moveit_servo_lib_parameters
    pose_tracking
    servo_node
    servo_controller_input
  EXPORT moveit_servoTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/moveit_servo
)

# Install Binaries
install(
  TARGETS
    servo_node_main
    servo_pose_tracking_demo
    fake_command_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/moveit_servo
)

# Install include, launch, config directories
install(DIRECTORY include/ DESTINATION include/moveit_servo)
install(DIRECTORY launch DESTINATION share/moveit_servo)
install(DIRECTORY config DESTINATION share/moveit_servo)

ament_export_targets(moveit_servoTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

#############
## TESTING ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)
  find_package(Boost REQUIRED COMPONENTS filesystem)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()

  # Servo integration launch test
  ament_add_gtest_executable(test_servo_integration
      test/test_servo_interface.cpp
      test/servo_launch_test_common.hpp
  )
  target_link_libraries(test_servo_integration moveit_servo_lib_parameters)
  ament_target_dependencies(test_servo_integration ${THIS_PACKAGE_INCLUDE_DEPENDS})
  add_ros_test(test/launch/test_servo_integration.test.py TIMEOUT 120 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  # Servo collision checking integration test
  ament_add_gtest_executable(test_servo_collision
        test/test_servo_collision.cpp
        test/servo_launch_test_common.hpp
  )
  target_link_libraries(test_servo_collision moveit_servo_lib_parameters)
  ament_target_dependencies(test_servo_collision ${THIS_PACKAGE_INCLUDE_DEPENDS})
  add_ros_test(test/launch/test_servo_collision.test.py TIMEOUT 120 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  # pose_tracking
  ament_add_gtest_executable(test_servo_pose_tracking
        test/pose_tracking_test.cpp
  )
  ament_target_dependencies(test_servo_pose_tracking ${THIS_PACKAGE_INCLUDE_DEPENDS})
  target_link_libraries(test_servo_pose_tracking pose_tracking)
  add_ros_test(test/launch/test_servo_pose_tracking.test.py TIMEOUT 120 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  # Unit tests
  ament_add_gtest(servo_calcs_unit_tests
    test/servo_calcs_unit_tests.cpp
  )
  target_link_libraries(servo_calcs_unit_tests moveit_servo_lib)

endif()

ament_package()
