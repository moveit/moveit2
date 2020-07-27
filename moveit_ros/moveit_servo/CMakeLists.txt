cmake_minimum_required(VERSION 3.5)
project(moveit_servo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(LIBRARY_NAME moveit_servo)
set(COMPOSABLE_NODE_NAME servo_server)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  sensor_msgs
  control_msgs
  moveit_msgs
  geometry_msgs
  trajectory_msgs
  moveit_ros_planning
)

include_directories(
  include
)

add_library(${LIBRARY_NAME} SHARED
   src/servo.cpp
   src/servo_calcs.cpp
   src/collision_check.cpp
   src/low_pass_filter.cpp
   include/moveit_servo/servo_parameters.cpp
)
set_target_properties(${LIBRARY_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${LIBRARY_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Add and export library for composable node server
add_library(${COMPOSABLE_NODE_NAME} SHARED src/servo_server.cpp)
ament_target_dependencies(${COMPOSABLE_NODE_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${COMPOSABLE_NODE_NAME} ${LIBRARY_NAME})
rclcpp_components_register_nodes(${COMPOSABLE_NODE_NAME} "moveit_servo::ServoServer")

# Add cpp interface example executable
set(CPP_DEMO_NAME servo_demo)
add_executable(${CPP_DEMO_NAME} src/ros2_demo/servo_cpp_interface_demo.cpp)
ament_target_dependencies(${CPP_DEMO_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${CPP_DEMO_NAME} ${LIBRARY_NAME})

# Add executable to publish fake servo commands for testing/demo purposes
set(FAKE_SERVO_CMDS_NAME fake_command_publisher)
add_executable(${FAKE_SERVO_CMDS_NAME} test/publish_fake_jog_commands.cpp)
ament_target_dependencies(${FAKE_SERVO_CMDS_NAME} 
  rclcpp
  geometry_msgs
  std_srvs
)

# Install Libraries
install(TARGETS ${LIBRARY_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  ${COMPOSABLE_NODE_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  ${CPP_DEMO_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  ${FAKE_SERVO_CMDS_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)
  find_package(Boost REQUIRED COMPONENTS filesystem)

  if(UNIX OR APPLE)
    set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}:${CMAKE_CURRENT_BINARY_DIR}/../utils:${CMAKE_CURRENT_BINARY_DIR}/../collision_detection_fcl:${CMAKE_CURRENT_BINARY_DIR}/../collision_detection")
  endif()

  # Lowpass filter unit test
  ament_add_gtest(test_low_pass_filter test/test_low_pass_filter.cpp)
  target_link_libraries( test_low_pass_filter ${LIBRARY_NAME})

  # Basic unit test Servo standup
  ament_add_gtest(test_servo test/test_standup_servo.cpp test/test_parameter_struct.hpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  ament_target_dependencies(test_servo ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
  target_link_libraries( test_servo ${LIBRARY_NAME})


  # Servo parameters launch test
  add_executable(test_servo_parameters_node 
    test/test_servo_parameters_node.cpp
    test/test_parameter_struct.hpp)
  ament_target_dependencies(test_servo_parameters_node ${THIS_PACKAGE_INCLUDE_DEPENDS})
  ament_add_gtest_executable(test_servo_parameters test/test_servo_parameters.cpp)
  ament_target_dependencies(test_servo_parameters ${THIS_PACKAGE_INCLUDE_DEPENDS})
  target_link_libraries(test_servo_parameters ${LIBRARY_NAME})
  add_ros_test(test/launch/test_servo_parameters.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

endif()

# Install include, launch, config directories
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${LIBRARY_NAME})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
