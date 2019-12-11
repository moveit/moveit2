cmake_minimum_required(VERSION 3.10.2)
project(moveit_planners_ompl)

# At least C++11 required for OMPL
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED system filesystem date_time thread serialization)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ompl REQUIRED)

  #dynamic_reconfigure
#generate_dynamic_reconfigure_options("ompl_interface/cfg/OMPLDynamicReconfigure.cfg")

#catkin_package(
#  LIBRARIES
#    moveit_ompl_interface
#  INCLUDE_DIRS
#    ompl_interface/include
#  CATKIN_DEPENDS
#    moveit_core
#  DEPENDS
#    OMPL
#)

include_directories(ompl_interface/include
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${OMPL_INCLUDE_DIRS})

add_subdirectory(ompl_interface)

#install(FILES ompl_interface_plugin_description.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
pluginlib_export_plugin_description_file(moveit_ompl_interface ompl_interface_plugin_description.xml)

ament_export_include_directories(include)
ament_export_libraries(moveit_ompl_interface)
ament_export_dependencies(moveit_core OMPL)

ament_package()
