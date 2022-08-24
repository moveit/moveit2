cmake_minimum_required(VERSION 3.22)
project(moveit_servo)

# C++ Libraries #################################################

# Core C++ library for calculations and collision checking.
# Provides interface used by the component node.
set(SERVO_LIB_NAME moveit_servo_lib)
# C++ library containing the parameters initialization
# - This is intended to use with the component node when you
# -   need to read the parameters loaded into servo in a separate
# -   node.
set(SERVO_PARAM_LIB_NAME ${SERVO_LIB_NAME}_parameters)

# Pose Tracking
set(POSE_TRACKING pose_tracking)

# Component Nodes (Shared libraries) ############################
set(SERVO_COMPONENT_NODE servo_node)
set(SERVO_CONTROLLER_INPUT servo_controller_input)

# Executable Nodes ##############################################
set(SERVO_NODE_MAIN_NAME servo_node_main)
set(POSE_TRACKING_DEMO_NAME servo_pose_tracking_demo)
set(FAKE_SERVO_CMDS_NAME fake_command_publisher)

#################################################################

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
  sensor_msgs
  std_msgs
  std_srvs
  tf2_eigen
  trajectory_msgs
)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
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
add_library(${SERVO_PARAM_LIB_NAME} SHARED
  src/servo_parameters.cpp
  src/parameter_descriptor_builder.cpp
)
set_target_properties(${SERVO_PARAM_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${SERVO_PARAM_LIB_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

# This library provides a C++ interface for sending realtime twist or joint commands to a robot
add_library(${SERVO_LIB_NAME} SHARED
  # These files are used to produce differential motion
  src/collision_check.cpp
  src/enforce_limits.cpp
  src/servo.cpp
  src/servo_calcs.cpp
)
set_target_properties(${SERVO_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${SERVO_LIB_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${SERVO_LIB_NAME} ${SERVO_PARAM_LIB_NAME})

add_library(${POSE_TRACKING} SHARED src/pose_tracking.cpp)
ament_target_dependencies(${POSE_TRACKING} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${POSE_TRACKING} ${SERVO_LIB_NAME})

#####################
## Component Nodes ##
#####################

# Add and export library to run as a ROS node component, and receive commands via topics
add_library(${SERVO_COMPONENT_NODE} SHARED src/servo_node.cpp)
ament_target_dependencies(${SERVO_COMPONENT_NODE} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${SERVO_COMPONENT_NODE} ${SERVO_LIB_NAME})
rclcpp_components_register_nodes(${SERVO_COMPONENT_NODE} "moveit_servo::ServoNode")

# Add executable for using a controller
add_library(${SERVO_CONTROLLER_INPUT} SHARED src/teleop_demo/joystick_servo_example.cpp)
ament_target_dependencies(${SERVO_CONTROLLER_INPUT} ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(${SERVO_CONTROLLER_INPUT} "moveit_servo::JoyToServoPub")

######################
## Executable Nodes ##
######################

# An executable node for the servo server
add_executable(${SERVO_NODE_MAIN_NAME} src/servo_node_main.cpp)
target_link_libraries(${SERVO_NODE_MAIN_NAME} ${SERVO_COMPONENT_NODE})
ament_target_dependencies(${SERVO_NODE_MAIN_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

# An example of pose tracking
add_executable(${POSE_TRACKING_DEMO_NAME} src/cpp_interface_demo/pose_tracking_demo.cpp)
target_link_libraries(${POSE_TRACKING_DEMO_NAME} ${POSE_TRACKING})
ament_target_dependencies(${POSE_TRACKING_DEMO_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Add executable to publish fake servo commands for testing/demo purposes
add_executable(${FAKE_SERVO_CMDS_NAME} test/publish_fake_jog_commands.cpp)
ament_target_dependencies(${FAKE_SERVO_CMDS_NAME}
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
    ${SERVO_LIB_NAME}
    ${SERVO_LIB_NAME}_parameters
    ${POSE_TRACKING}
    ${SERVO_COMPONENT_NODE}
    ${SERVO_CONTROLLER_INPUT}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install Binaries
install(
  TARGETS
    ${SERVO_NODE_MAIN_NAME}
    ${CPP_DEMO_NAME}
    ${POSE_TRACKING_DEMO_NAME}
    ${FAKE_SERVO_CMDS_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include, launch, config directories
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
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

  # Unit test for ServoCalcs
  # TODO (vatanaksoytezer): Rewrite this similar to enforce limits test
  # ament_add_gtest_executable(
  #   unit_test_servo_calcs
  #   test/unit_test_servo_calcs.cpp
  # )
  # target_link_libraries(unit_test_servo_calcs ${SERVO_LIB_NAME})
  # ament_target_dependencies(unit_test_servo_calcs ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
  # add_ros_test(test/launch/unit_test_servo_calcs.test.py TIMEOUT 120 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  # Servo integration launch test
  ament_add_gtest_executable(test_servo_integration
      test/test_servo_interface.cpp
      test/servo_launch_test_common.hpp
  )
  target_link_libraries(test_servo_integration ${SERVO_PARAM_LIB_NAME})
  ament_target_dependencies(test_servo_integration ${THIS_PACKAGE_INCLUDE_DEPENDS})
  add_ros_test(test/launch/test_servo_integration.test.py TIMEOUT 120 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  # Servo collision checking integration test
  ament_add_gtest_executable(test_servo_collision
        test/test_servo_collision.cpp
        test/servo_launch_test_common.hpp
  )
  target_link_libraries(test_servo_collision ${SERVO_PARAM_LIB_NAME})
  ament_target_dependencies(test_servo_collision ${THIS_PACKAGE_INCLUDE_DEPENDS})
  add_ros_test(test/launch/test_servo_collision.test.py TIMEOUT 120 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  # TODO(sjahr): Debug and re-enable this test
  # Servo singularity checking integration test
  #ament_add_gtest_executable(test_servo_singularity
  #      test/test_servo_singularity.cpp
  #      test/servo_launch_test_common.hpp
  #)
  #target_link_libraries(test_servo_singularity ${SERVO_PARAM_LIB_NAME})
  #ament_target_dependencies(test_servo_singularity ${THIS_PACKAGE_INCLUDE_DEPENDS})
  #add_ros_test(test/launch/test_servo_singularity.test.py TIMEOUT 120 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  # pose_tracking
  ament_add_gtest_executable(test_servo_pose_tracking
        test/pose_tracking_test.cpp
  )
  ament_target_dependencies(test_servo_pose_tracking ${THIS_PACKAGE_INCLUDE_DEPENDS})
  target_link_libraries(test_servo_pose_tracking ${POSE_TRACKING})
  add_ros_test(test/launch/test_servo_pose_tracking.test.py TIMEOUT 120 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  # basic_servo_tests
  ament_add_gtest_executable(basic_servo_tests
        test/basic_servo_tests.cpp
  )
  ament_target_dependencies(basic_servo_tests ${THIS_PACKAGE_INCLUDE_DEPENDS})
  target_link_libraries(basic_servo_tests ${POSE_TRACKING})
  add_ros_test(test/launch/test_basic_servo.test.py TIMEOUT 120 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  # Enforce limits unit tests
  ament_add_gtest(enforce_limits_tests
    test/enforce_limits_tests.cpp
  )
  target_link_libraries(enforce_limits_tests ${SERVO_LIB_NAME})

endif()

ament_package()
