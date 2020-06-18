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

# if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
#   set(CMAKE_BUILD_TYPE Release)
# endif()

# find_package(Boost REQUIRED)

# find_package(catkin REQUIRED COMPONENTS
#   control_msgs
#   geometry_msgs
#   moveit_msgs
#   moveit_ros_planning_interface
#   rosparam_shortcuts
#   sensor_msgs
#   std_msgs
#   trajectory_msgs
# )
# find_package(Eigen3 REQUIRED)

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

# catkin_package(
#   INCLUDE_DIRS
#     include
#   LIBRARIES
#     ${LIBRARY_NAME}
#   CATKIN_DEPENDS
#     control_msgs
#     geometry_msgs
#     moveit_msgs
#     moveit_ros_planning_interface
#     rosparam_shortcuts
#     sensor_msgs
#     std_msgs
#     trajectory_msgs
#   DEPENDS
#     EIGEN3
# )

# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${EIGEN3_INCLUDE_DIRS}
#   ${Boost_INCLUDE_DIR}
# )

include_directories(
  include
)

# Add composable node libraries
add_library(collision_check_component SHARED
   src/collision_check.cpp)
ament_target_dependencies(collision_check_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "sensor_msgs"
  "moveit_ros_planning")
rclcpp_components_register_nodes(collision_check_component "moveit_servo::CollisionCheck")

# add_library(servo_calcs_component SHARED
#    src/servo_calcs.cpp)
# ament_target_dependencies(servo_calcs_component
#   "rclcpp"
#   "rclcpp_components"
#   "std_msgs"
#   "sensor_msgs"
#   "moveit_ros_planning")
# rclcpp_components_register_nodes(servo_calcs_component "moveit_servo::CollisionCheck")

# Install Libraries
install(TARGETS
  collision_check_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
#########################################
## A library providing a C++ interface ##
#########################################

# add_library(${LIBRARY_NAME} SHARED
#   src/collision_check.cpp
#   src/servo_calcs.cpp
#   src/servo.cpp
#   src/joint_state_subscriber.cpp
#   src/low_pass_filter.cpp
# )
# add_dependencies(${LIBRARY_NAME} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${LIBRARY_NAME}
#   ${catkin_LIBRARIES}
#   ${Eigen_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

# # An example of using the C++ library
# add_executable(cpp_interface_example
#   src/cpp_interface_example/cpp_interface_example.cpp
# )
# add_dependencies(cpp_interface_example ${catkin_EXPORTED_TARGETS})
# target_link_libraries(cpp_interface_example
#   ${catkin_LIBRARIES}
#   ${Eigen_LIBRARIES}
#   ${Boost_LIBRARIES}
#   ${LIBRARY_NAME}
# )

############################
## ROS message-based node ##
############################

# add_executable(servo_server
#   src/servo_server.cpp
# )
# add_dependencies(servo_server ${catkin_EXPORTED_TARGETS})
# target_link_libraries(servo_server
#   ${LIBRARY_NAME}
#   ${catkin_LIBRARIES}
#   ${Eigen_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

################################################
## An example of converting joystick commands ##
################################################

# add_executable(spacenav_to_twist
#   src/teleop_examples/spacenav_to_twist.cpp
# )
# add_dependencies(spacenav_to_twist ${catkin_EXPORTED_TARGETS})
# target_link_libraries(spacenav_to_twist ${catkin_LIBRARIES})

##################
## INSTALLATION ##
##################

# install(
#   TARGETS
#     ${LIBRARY_NAME}
#     servo_server
#     spacenav_to_twist
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

#############
## TESTING ##
#############

# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
#   find_package(ros_pytest REQUIRED)

#   # Python integration tests
#   add_rostest(test/launch/servo_halt_msg_test.test)
#   add_rostest(test/launch/servo_msg_reception_test.test)
#   add_rostest(test/launch/servo_vel_accel_limits_test.test)
#   add_rostest(test/launch/servo_drift_dimensions_service_test.test)

#   # servo_cpp_interface
#   add_rostest_gtest(servo_cpp_interface_test
#     test/servo_cpp_interface_test.test
#     test/servo_cpp_interface_test.cpp
#   )
#   target_link_libraries(servo_cpp_interface_test
#     ${LIBRARY_NAME}
#     ${catkin_LIBRARIES}
#     ${Eigen_LIBRARIES}
#     ${Boost_LIBRARIES}
#   )

# Install Launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
