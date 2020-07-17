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

# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   ament_add_gtest(read_parameters_test test/servo_parameters_test.cpp)
#   target_link_libraries( read_parameters_test
#     servo_library
#   )
# endif()

# Install include, launch, config directories
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${LIBRARY_NAME})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
