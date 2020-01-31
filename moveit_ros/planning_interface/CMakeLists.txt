cmake_minimum_required(VERSION 3.10.2)
project(moveit_ros_planning_interface)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
# find_package(moveit_ros_warehouse REQUIRED)
# find_package(moveit_ros_manipulation REQUIRED)
# find_package(moveit_ros_move_group REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)

find_package(PythonInterp REQUIRED)
find_package(PythonLibs "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}" REQUIRED)

find_package(Boost REQUIRED)
if(Boost_VERSION LESS 106700)
  set(BOOST_PYTHON_COMPONENT python)
elseif()
  set(BOOST_PYTHON_COMPONENT python${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR})
endif()

find_package(Boost REQUIRED COMPONENTS
  date_time
  filesystem
  program_options
  ${BOOST_PYTHON_COMPONENT}
  system
  thread
)
find_package(Eigen3 REQUIRED)
# find_package(eigenpy REQUIRED)

set(THIS_PACKAGE_INCLUDE_DIRS
#  py_bindings_tools/include
  common_planning_interface_objects/include
#  planning_scene_interface/include
#  move_group_interface/include
#  moveit_cpp/include
)

# catkin_python_setup()


include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

# add_subdirectory(py_bindings_tools)
add_subdirectory(common_planning_interface_objects)
# add_subdirectory(planning_scene_interface)
# add_subdirectory(move_group_interface)
# add_subdirectory(robot_interface)
# add_subdirectory(test)
# add_subdirectory(moveit_cpp)

ament_export_include_directories(include)
ament_export_dependencies(moveit_msgs)
ament_export_dependencies(moveit_ros_planning)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_eigen)
ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_action)
ament_export_dependencies(rclpy)
ament_export_dependencies(PythonInterp)
ament_export_dependencies(PythonLibs)
ament_export_dependencies(Eigen3)
ament_export_dependencies(Boost)
ament_package()
