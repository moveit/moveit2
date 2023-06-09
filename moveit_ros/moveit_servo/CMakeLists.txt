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
  config/servo_parameters.yaml
)

# This library provides a C++ interface for sending realtime twist or joint commands to a robot
add_library(moveit_servo_lib SHARED
  src/collision_monitor.cpp
  src/command_processor.cpp
  src/servo.cpp
  src/utils.cpp

)
set_target_properties(moveit_servo_lib PROPERTIES VERSION "${moveit_servo_VERSION}")
ament_target_dependencies(moveit_servo_lib ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(moveit_servo_lib moveit_servo_lib_parameters)

######################
## Executable Nodes ##
######################

# An executable node for the joint jog demo
add_executable(demo_joint_jog demos/cpp_interface/demo_joint_jog.cpp )
target_link_libraries(demo_joint_jog moveit_servo_lib)
ament_target_dependencies(demo_joint_jog ${THIS_PACKAGE_INCLUDE_DEPENDS})

# An executable node for the twist demo
add_executable(demo_twist demos/cpp_interface/demo_twist.cpp )
target_link_libraries(demo_twist moveit_servo_lib)
ament_target_dependencies(demo_twist ${THIS_PACKAGE_INCLUDE_DEPENDS})

# An executable node for the pose demo
add_executable(demo_pose demos/cpp_interface/demo_pose.cpp )
target_link_libraries(demo_pose moveit_servo_lib)
ament_target_dependencies(demo_pose ${THIS_PACKAGE_INCLUDE_DEPENDS})

#############
## Install ##
#############

# Install Libraries
install(
  TARGETS
    moveit_servo_lib
    moveit_servo_lib_parameters
  EXPORT moveit_servoTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/moveit_servo
)

# Install Binaries
install(
  TARGETS
  demo_joint_jog
  demo_twist
  demo_pose
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

ament_package()
