cmake_minimum_required(VERSION 3.10.2)
project(moveit_ros_planning_interface)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
# find_package(moveit_ros_warehouse REQUIRED)
# find_package(moveit_ros_manipulation REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)

# find_package(PythonInterp REQUIRED)
# find_package(PythonLibs "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}" REQUIRED)

find_package(Boost REQUIRED)
#if(Boost_VERSION LESS 106700)
#  set(BOOST_PYTHON_COMPONENT python)
#else()
#  set(BOOST_PYTHON_COMPONENT python${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR})
#endif()

find_package(Boost REQUIRED COMPONENTS
  date_time
  filesystem
  program_options
  #${BOOST_PYTHON_COMPONENT}
  system
  thread
)
find_package(Eigen3 REQUIRED)
# find_package(eigenpy REQUIRED)

set(THIS_PACKAGE_INCLUDE_DIRS
#  py_bindings_tools/include
  common_planning_interface_objects/include
  planning_scene_interface/include
  move_group_interface/include
  moveit_cpp/include
)

set(THIS_PACKAGE_LIBRARIES
  moveit_common_planning_interface_objects
  moveit_planning_scene_interface
  moveit_move_group_interface
  moveit_cpp
#  moveit_py_bindings_tools
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  geometry_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
#  moveit_ros_warehouse
#  moveit_ros_manipulation
  moveit_ros_move_group
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  rclcpp
  rclcpp_action
  Eigen3
  Boost
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

# add_subdirectory(py_bindings_tools)
add_subdirectory(common_planning_interface_objects)
add_subdirectory(planning_scene_interface)
add_subdirectory(move_group_interface)
# add_subdirectory(robot_interface)
# add_subdirectory(test)
add_subdirectory(moveit_cpp)

ament_export_include_directories(include)
ament_export_libraries(${THIS_PACKAGE_LIBRARIES})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
