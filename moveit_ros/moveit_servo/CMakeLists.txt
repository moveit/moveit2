cmake_minimum_required(VERSION 3.5)
project(moveit_servo)

set(SERVO_LIB_NAME moveit_servo)
set(COMPOSABLE_NODE_NAME servo_server)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  sensor_msgs
  control_msgs
  control_toolbox
  moveit_msgs
  geometry_msgs
  tf2_eigen
  trajectory_msgs
  moveit_ros_planning
)

include_directories(
  include
)

#########################################################
## Library to process realtime twist or joint commands ##
#########################################################

# This library provides an interface for sending realtime twist or joint commands to a robot
add_library(${SERVO_LIB_NAME} SHARED
  # These files are used to produce differential motion
  src/servo.cpp
  src/servo_calcs.cpp
  src/collision_check.cpp
  src/low_pass_filter.cpp
  include/moveit_servo/servo_parameters.cpp
)
set_target_properties(${SERVO_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${SERVO_LIB_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Add and export library for composable node server
add_library(${COMPOSABLE_NODE_NAME} SHARED src/servo_server.cpp)
ament_target_dependencies(${COMPOSABLE_NODE_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${COMPOSABLE_NODE_NAME} ${SERVO_LIB_NAME})
rclcpp_components_register_nodes(${COMPOSABLE_NODE_NAME} "moveit_servo::ServoServer")

# Add cpp interface example executable
set(CPP_DEMO_NAME servo_cpp_demo)
add_executable(${CPP_DEMO_NAME} src/cpp_interface_demo/servo_cpp_interface_demo.cpp)
ament_target_dependencies(${CPP_DEMO_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${CPP_DEMO_NAME} ${SERVO_LIB_NAME})


########################################
## Library for servoing toward a pose ##
########################################

# TODO(henningkayser): Port pose tracking and demos in cpp_interface_example to ROS 2
# add_library(${POSE_TRACKING} SHARED src/pose_tracking_demo.cpp)
# ament_target_dependencies(${POSE_TRACKING} ${THIS_PACKAGE_INCLUDE_DEPENDS})
# add_dependencies(${POSE_TRACKING} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${POSE_TRACKING} ${SERVO_LIB_NAME})
#
# An example of pose tracking
# set(POSE_TRACKING_DEMO_NAME servo_pose_tracking_demo)
# add_executable(${POSE_TRACKING_DEMO_NAME} src/cpp_interface_example/pose_tracking_demo.cpp)
# ament_target_dependencies(${POSE_TRACKING_DEMO_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Add executable for using a controller
set(SERVO_CONTROLLER_INPUT servo_controller_input)
add_library(${SERVO_CONTROLLER_INPUT} SHARED src/teleop_demo/joystick_servo_example.cpp)
ament_target_dependencies(${SERVO_CONTROLLER_INPUT} ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(${SERVO_CONTROLLER_INPUT} "moveit_servo::JoyToServoPub")

# Add executable to publish fake servo commands for testing/demo purposes
set(FAKE_SERVO_CMDS_NAME fake_command_publisher)
add_executable(${FAKE_SERVO_CMDS_NAME} test/publish_fake_jog_commands.cpp)
ament_target_dependencies(${FAKE_SERVO_CMDS_NAME}
  rclcpp
  geometry_msgs
  std_srvs
)

# Install Libraries
install(
  TARGETS
    ${SERVO_LIB_NAME}
    ${COMPOSABLE_NODE_NAME}
    ${SERVO_CONTROLLER_INPUT}
    ${CPP_DEMO_NAME}
    ${FAKE_SERVO_CMDS_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME})

#############
## TESTING ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)
  find_package(Boost REQUIRED COMPONENTS filesystem)

  # Lowpass filter unit test
  ament_add_gtest(test_low_pass_filter test/test_low_pass_filter.cpp)
  target_link_libraries(test_low_pass_filter ${SERVO_LIB_NAME})

  # Basic unit test ServoCalcs
  ament_add_gtest(
    test_servo
    test/unit_test_servo_calcs.cpp
    test/test_parameter_struct.hpp
  )
  target_link_libraries(test_servo ${SERVO_LIB_NAME})
  ament_target_dependencies(test_servo ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost ament_cmake_gtest ros_testing)

  # Servo parameters launch test
  add_executable(test_servo_parameters_node
    test/test_servo_parameters_node.cpp
    test/test_parameter_struct.hpp)
  ament_target_dependencies(test_servo_parameters_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

  ament_add_gtest_executable(test_servo_parameters test/test_servo_parameters.cpp)
  ament_target_dependencies(test_servo_parameters ${THIS_PACKAGE_INCLUDE_DEPENDS})
  target_link_libraries(test_servo_parameters ${SERVO_LIB_NAME})
  add_ros_test(test/launch/test_servo_parameters.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
  # End servo parameters launch test

  # Servo integration launch test
  ament_add_gtest_executable(test_servo_integration
      test/test_servo_interface.cpp
      test/test_parameter_struct.hpp
      test/servo_launch_test_common.hpp
  )
  ament_target_dependencies(test_servo_integration ${THIS_PACKAGE_INCLUDE_DEPENDS})
  add_ros_test(test/launch/test_servo_integration.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
  # end servo integration launch test

  # Servo collision checking integration test
  ament_add_gtest_executable(test_servo_collision
        test/test_servo_collision.cpp
        test/test_parameter_struct.hpp
        test/servo_launch_test_common.hpp
  )
  ament_target_dependencies(test_servo_collision ${THIS_PACKAGE_INCLUDE_DEPENDS})
  add_ros_test(test/launch/test_servo_collision.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
  # end servo collision checking integration test

  # Servo singularity checking integration test
  ament_add_gtest_executable(test_servo_singularity
        test/test_servo_singularity.cpp
        test/test_parameter_struct.hpp
        test/servo_launch_test_common.hpp
  )
  ament_target_dependencies(test_servo_singularity ${THIS_PACKAGE_INCLUDE_DEPENDS})
  add_ros_test(test/launch/test_servo_singularity.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
  # end servo singularity checking integration test

  # TODO(henningkayser): Port tests to ROS2
  # servo_cpp_interface
  # add_rostest_gtest(servo_cpp_interface_test
  #   test/servo_cpp_interface_test.test
  #   test/servo_cpp_interface_test.cpp
  # )
  # target_link_libraries(servo_cpp_interface_test
  #   ${SERVO_LIB_NAME}
  #   ${catkin_LIBRARIES}
  # )

  # # pose_tracking
  # add_rostest_gtest(pose_tracking_test
  #   test/pose_tracking_test.test
  #   test/pose_tracking_test.cpp
  # )
  # target_link_libraries(pose_tracking_test
  #   pose_tracking
  #   ${catkin_LIBRARIES}
  # )

endif()

# Install include, launch, config directories
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})


ament_export_include_directories(include)
ament_export_libraries(${SERVO_LIB_NAME})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
