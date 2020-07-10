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

set(LIBRARY_NAME moveit_servo_cpp_api)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)

include_directories(
  include
)

# Add composable node libraries
# add_library(collision_check_component SHARED
#    src/collision_check.cpp)
# ament_target_dependencies(collision_check_component
#   "rclcpp"
#   "rclcpp_components"
#   "std_msgs"
#   "sensor_msgs"
#   "moveit_ros_planning")
# rclcpp_components_register_nodes(collision_check_component "moveit_servo::CollisionCheck")

# add_library(servo_calcs_component SHARED
#    src/servo_calcs.cpp)
# ament_target_dependencies(servo_calcs_component
#   "rclcpp"
#   "rclcpp_components"
#   "std_msgs"
#   "sensor_msgs"
#   "control_msgs"
#   "moveit_msgs"
#   "geometry_msgs"
#   "trajectory_msgs"
#   "moveit_ros_planning")
# rclcpp_components_register_nodes(servo_calcs_component "moveit_servo::JogCalcs")

add_library(servo_library SHARED
   src/servo.cpp
   src/servo_calcs.cpp
   src/collision_check.cpp
   src/low_pass_filter.cpp
   include/moveit_servo/servo_parameters.cpp)

ament_target_dependencies(servo_library
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "sensor_msgs"
  "control_msgs"
  "moveit_msgs"
  "geometry_msgs"
  "trajectory_msgs"
  "moveit_ros_planning")

install(
  DIRECTORY include/
  DESTINATION include
)

# Install Libraries
install(TARGETS
  # collision_check_component
  servo_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_executable(servo_demo src/ros2_demo/servo_cpp_interface_demo.cpp src/servo.cpp src/servo_calcs.cpp src/collision_check.cpp src/low_pass_filter.cpp)
ament_target_dependencies(servo_demo 
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "sensor_msgs"
  "control_msgs"
  "moveit_msgs"
  "geometry_msgs"
  "trajectory_msgs"
  "moveit_ros_planning")
install(TARGETS
  servo_demo
  DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   ament_add_gtest(read_parameters_test test/servo_parameters_test.cpp)
#   target_link_libraries( read_parameters_test
#     servo_library
#   )
# endif()

# Install Launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
