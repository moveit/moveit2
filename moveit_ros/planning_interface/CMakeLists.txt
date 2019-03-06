cmake_minimum_required(VERSION 3.10.2)
project(moveit_ros_planning_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_msgs REQUIRED)
# TODO: uncomment this as submodules become available
# find_package(moveit_ros_planning REQUIRED)
# find_package(moveit_ros_warehouse REQUIRED)
# find_package(moveit_ros_manipulation REQUIRED)
# find_package(moveit_ros_move_group REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(Boost REQUIRED COMPONENTS
  date_time
  filesystem
  program_options
  ${BOOST_PYTHON_COMPONENT}
  system
  thread
)

find_package(Eigen3 REQUIRED)

find_package(PythonInterp REQUIRED)
find_package(PythonLibs "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}" REQUIRED)

find_package(Boost REQUIRED)
if(Boost_VERSION LESS 106700)
  set(BOOST_PYTHON_COMPONENT python)
elseif()
  set(BOOST_PYTHON_COMPONENT python${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR})
endif()

set(THIS_PACKAGE_INCLUDE_DIRS
  py_bindings_tools/include
  common_planning_interface_objects/include
  planning_scene_interface/include
  move_group_interface/include
)

set(THIS_PACKAGE_INCLUDE_LIBRARIES
      moveit_common_planning_interface_objects
      moveit_planning_scene_interface
      moveit_move_group_interface
)

set(THIS_PACKAGE_INCLUDE_DEPENDENCIES
      moveit_ros_planning
      moveit_ros_warehouse
      moveit_ros_manipulation
      moveit_ros_move_group
      geometry_msgs
      moveit_msgs
      tf2_geometry_msgs
      EIGEN3
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS})

include_directories(SYSTEM
                    ${EIGEN3_INCLUDE_DIRS}
                    ${PYTHON_INCLUDE_DIRS})

ament_export_libraries(${THIS_PACKAGE_INCLUDE_LIBRARIES})
ament_export_include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDENCIES})

# add_subdirectory(py_bindings_tools)
# add_subdirectory(common_planning_interface_objects)
# add_subdirectory(planning_scene_interface)
# add_subdirectory(move_group_interface)
# add_subdirectory(robot_interface)
# add_subdirectory(test)

ament_package()
